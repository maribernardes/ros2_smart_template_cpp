#include "smart_template_node.hpp"
#include <rclcpp/executors/multi_threaded_executor.hpp>

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "tinyxml2.h"  // for URDF parsing
#include <algorithm>
#include <Eigen/Core>
#include <Eigen/Dense>

#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

/*#########################################################################
#
# Smart Template C++
#
# Description:
# This node implements a node for the SmartTemplate 3DOF needle guide robot
# Implements the robot service and action servers
#
# Publishes:   
# '/stage/state/guide_pose'     (geometry_msgs.msg.PointStamped)  - [mm] robot frame
# '/joint_states'               (sensor_msgs.msg.JointState)      - [m] robot frame
#
# Subscribe:
# '/desired_position'           (geometry_msgs.msg.Point)  - [mm] robot frame
# '/desired_command'            (std_msgs.msg.String)
#
# Action/Service clients:
# '/stage/move_and_observe'     (smart_template_interfaces.action.MoveAndObserve) - robot frame
# '/stage/move'                 (smart_template_interfaces.srv.Move) - robot frame
# '/stage/command'              (smart_template_interfaces.srv.Command) - robot frame
# '/stage/get_position'         (smart_template_interfaces.srv.GetPoint) - robot frame
# 
#########################################################################*/

namespace smart_template_cpp
{

SmartTemplateNode::SmartTemplateNode()
: Node("smart_template_node"),
  abort_(false)
{
  // Load joint info
  declare_parameter<std::string>("robot_description", "");
  std::string urdf;
  get_parameter("robot_description", urdf);

  if (!parse_joint_info_from_urdf(urdf)) {
    RCLCPP_FATAL(this->get_logger(), "Failed to parse joint info from robot_description.");
    rclcpp::shutdown();
    return;
  }
  joint_positions_ = Eigen::VectorXd::Zero(joint_info_.size());
  last_joint_command_ = Eigen::VectorXd::Zero(joint_info_.names.size());

  // Initialize subscribers
  joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 10,
    std::bind(&SmartTemplateNode::joint_state_callback, this, std::placeholders::_1));

  desired_position_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
    "/desired_position", 10,
    std::bind(&SmartTemplateNode::desired_position_callback, this, std::placeholders::_1));
  
  desired_command_sub_ = this->create_subscription<std_msgs::msg::String>(
    "/desired_command", 10,
    std::bind(&SmartTemplateNode::desired_command_callback, this, std::placeholders::_1));
    
  // Initialize publishers
  joint_command_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/position_controller/commands", 10);
  stage_pose_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/stage/state/guide_pose", 10);
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(200),
    std::bind(&SmartTemplateNode::timer_stage_pose_callback, this));
  
  // Initialize service servers
  command_server_ = this->create_service<smart_template_interfaces::srv::Command>(
    "/stage/command",
    [this](const std::shared_ptr<smart_template_interfaces::srv::Command::Request> request,
            std::shared_ptr<smart_template_interfaces::srv::Command::Response> response) {
      this->command_callback(request, response);
    });
    
  move_server_ = this->create_service<smart_template_interfaces::srv::Move>(
    "/stage/move",
    [this](const std::shared_ptr<smart_template_interfaces::srv::Move::Request> request,
            std::shared_ptr<smart_template_interfaces::srv::Move::Response> response) {
      this->move_callback(request, response);
    });
  
  current_position_server_ = this->create_service<smart_template_interfaces::srv::GetPoint>(
    "/stage/get_position",
    [this](const std::shared_ptr<smart_template_interfaces::srv::GetPoint::Request> request,
            std::shared_ptr<smart_template_interfaces::srv::GetPoint::Response> response) {
      this->current_position_callback(request, response);
    });

  // Initialize action server
  action_callback_group_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);  
  action_server_ = rclcpp_action::create_server<MoveAndObserve>(
    this,
    "/stage/move_and_observe",
    std::bind(&SmartTemplateNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&SmartTemplateNode::handle_cancel, this, std::placeholders::_1),
    std::bind(&SmartTemplateNode::handle_accepted, this, std::placeholders::_1));
  
}

SmartTemplateNode::~SmartTemplateNode()
{
  // Galil hardware now managed by the hardware interface
}

//#### Kinematic model ###################################################

// Forward-kinematics funtion
Eigen::Vector3d SmartTemplateNode::compute_fk(const Eigen::VectorXd& joints_mm) const
{
  if (joints_mm.size() != 3) {
    RCLCPP_ERROR(this->get_logger(), "FK: Expected 3 joint values, got %ld", joints_mm.size());
    return Eigen::Vector3d::Zero();
  }
  // Order: [horizontal, insertion, vertical]
  double x = joints_mm[0];
  double y = joints_mm[1];
  double z = joints_mm[2];
  return Eigen::Vector3d(x, y, z);
}

// Inverse-kinematics function
Eigen::VectorXd SmartTemplateNode::compute_ik(const Eigen::Vector3d& position_mm) const
{
  Eigen::VectorXd joints_mm(3);
  joints_mm[0] = position_mm[0];  // horizontal
  joints_mm[1] = position_mm[1];  // insertion
  joints_mm[2] = position_mm[2];  // vertical
  return check_limits(joints_mm); // Clamp values to joint limits
}

//#### Internal functions ###################################################

// Parse joints info from robot_description
bool SmartTemplateNode::parse_joint_info_from_urdf(const std::string& urdf_str) {
  tinyxml2::XMLDocument doc;
  if (doc.Parse(urdf_str.c_str()) != tinyxml2::XML_SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "Failed to parse URDF XML.");
    return false;
  }
  auto* robot = doc.FirstChildElement("robot");
  if (!robot) {
    RCLCPP_ERROR(this->get_logger(), "No <robot> element found in URDF.");
    return false;
  }
  std::unordered_map<std::string, std::string> joint_channels;
  std::unordered_map<std::string, std::pair<double, double>> joint_limits;
  std::unordered_map<std::string, double> joint_mmtocount;
  for (auto* joint = robot->FirstChildElement("joint"); joint; joint = joint->NextSiblingElement("joint")) {
    const char* name = joint->Attribute("name");
    if (!name) continue;
    std::string joint_name(name);
    auto* channel = joint->FirstChildElement("channel");
    auto* limit = joint->FirstChildElement("limit");
    auto* mm_elem = joint->FirstChildElement("mm_to_count");
    if (channel && channel->GetText()) {
      joint_channels[joint_name] = channel->GetText();
    }
    if (limit) {
      double lower = limit->DoubleAttribute("lower") * 1000.0;  // m to mm
      double upper = limit->DoubleAttribute("upper") * 1000.0;
      joint_limits[joint_name] = {lower, upper};
    }
    if (mm_elem && mm_elem->GetText()) {
      joint_mmtocount[joint_name] = std::stod(mm_elem->GetText());
    }
  }
  // Sort by channel name (for consistency)
  std::vector<std::pair<std::string, std::string>> sorted;
  for (const auto& [name, ch] : joint_channels) {
    sorted.emplace_back(name, ch);
  }
  std::sort(sorted.begin(), sorted.end(), [](const auto& a, const auto& b) {
    return a.second < b.second;
  });
  for (const auto& [name, ch] : sorted) {
    double lower = joint_limits.count(name) ? joint_limits[name].first : -100.0;
    double upper = joint_limits.count(name) ? joint_limits[name].second : 100.0;
    double mm_to_count = joint_mmtocount.count(name) ? joint_mmtocount[name] : 1.0;
    joint_info_.add(name, ch, lower, upper, mm_to_count);
    RCLCPP_INFO(this->get_logger(), "Joint %s — Channel: %s, Limits: [%.2f, %.2f] mm, mm_to_count: %.3f",
                name.c_str(), ch.c_str(), lower, upper, mm_to_count);
  }
  joint_info_.finalize(); //build fast index lookup
  return true;
}

// Get current robot joint values [mm]
Eigen::VectorXd SmartTemplateNode::get_joints() const {
  return joint_positions_;
}

// Get current joints error [mm]
Eigen::VectorXd SmartTemplateNode::get_joints_err() const {
  if (joint_positions_.size() != last_joint_command_.size()) {
    RCLCPP_WARN(this->get_logger(), "get_joints_err(): Size mismatch (positions = %ld, commands = %ld)",
                joint_positions_.size(), last_joint_command_.size());
    return Eigen::VectorXd::Zero(joint_positions_.size());
  }
  RCLCPP_INFO(this->get_logger(), "Joint Pos [mm] = (%.2f, %.2f, %.2f)", joint_positions_[0], joint_positions_[1], joint_positions_[2]);
  Eigen::VectorXd error = last_joint_command_ - joint_positions_;
  RCLCPP_INFO(this->get_logger(), "Joint Err [mm] = (%.2f, %.2f, %.2f)", error[0], error[1], error[2]);
  return error;
}

// Get current robot position (end-effector)
Eigen::Vector3d SmartTemplateNode::get_position() const {
  if (joint_positions_.size() != 3) {
    RCLCPP_WARN(this->get_logger(), "get_position(): Joint vector has wrong size (%ld)", joint_positions_.size());
    return Eigen::Vector3d::Zero();
  }
  return compute_fk(joint_positions_);
}

// Get error (euclidean distance) between two 3D coordinates
double SmartTemplateNode::error_3d(const Eigen::Vector3d& a, const Eigen::Vector3d& b) const {
  return (a - b).norm();
}

//Check joints limits
Eigen::VectorXd SmartTemplateNode::check_limits(const Eigen::VectorXd& joint_values) const {
  if (joint_values.size() != static_cast<int>(joint_info_.size())) {
    RCLCPP_ERROR(this->get_logger(), "check_limits: joint count mismatch (%ld != %zu)",
                 joint_values.size(), joint_info_.size());
    throw std::runtime_error("check_limits: vector size mismatch");
  }
  Eigen::VectorXd capped_values = joint_values;
  for (int i = 0; i < capped_values.size(); ++i) {
    double lower = joint_info_.limits_lower[i];
    double upper = joint_info_.limits_upper[i];
    double value = joint_values[i];
    if (value < lower || value > upper) {
      RCLCPP_WARN(this->get_logger(),
                  "%s value %.2f mm out of bounds [%.2f, %.2f] — clipping.",
                  joint_info_.names[i].c_str(), value, lower, upper);
    }
    capped_values[i] = std::clamp(value, lower, upper);
  }
  return capped_values;
}

// Abort motion
void SmartTemplateNode::abort_motion()
{
  RCLCPP_WARN(this->get_logger(), "ABORT was requested - TO BE IMPLEMENTED IN GalilSystemHardwareInterface");
  /*try {
    char buffer[1024];
    GSize bytes_returned;
    GCommand(galil_, "SH", buffer, sizeof(buffer), &bytes_returned);
    RCLCPP_WARN(this->get_logger(), "ABORT sent to Galil");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Error aborting motion: %s", e.what());
  }*/
}

//#### Subscriber callbacks ###################################################

// Receives a joint_state message
void SmartTemplateNode::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
  RCLCPP_DEBUG(this->get_logger(), "Received JointState with %zu joints", msg->name.size());
  for (size_t i = 0; i < msg->name.size(); ++i) {
    const std::string& joint_name = msg->name[i];
    int idx = joint_info_.index(joint_name);
    if (idx >= 0 && idx < static_cast<int>(joint_positions_.size())) {
      joint_positions_[idx] = 1000*msg->position[i]; // Convert from m to mm
      RCLCPP_DEBUG(this->get_logger(), "Updated %s:  %.3f", joint_name.c_str(),  joint_positions_[idx]);
    } else {
      RCLCPP_WARN(this->get_logger(),
        "Received joint '%s' which is not in joint_info_ or index out of bounds (index=%d)",
        joint_name.c_str(), idx);
    }
  }
}

// Receives a request message for desired command
void SmartTemplateNode::desired_command_callback(const std_msgs::msg::String::SharedPtr msg)
{
  std::string command = msg->data;
  RCLCPP_INFO(this->get_logger(), "Command %s SENT", command.c_str());
  if (command == "HOME") {
    // Go to origin in joint space
    Eigen::VectorXd q_cmd = Eigen::VectorXd::Zero(3);
    send_joint_command(q_cmd);
  }
  else if (command == "RETRACT") {
    // Read current position, then retract insertion axis (Y)
    Eigen::VectorXd q_cmd = get_joints();
    RCLCPP_DEBUG(this->get_logger(), "Current joints %f, %f, %f", q_cmd[0],q_cmd[1],q_cmd[2] );
    q_cmd[1] = 0.0;  // set insertion (Y) to 0
    send_joint_command(q_cmd);
  }
  else if (command == "ABORT") {
    abort_ = true;
    abort_motion();
  }
  else if (command == "RESUME") {
    abort_ = false;
  }
  else {
    RCLCPP_WARN(this->get_logger(), "Unknown command: %s", command.c_str());
  }
}


// Receives a request message for desired position
void SmartTemplateNode::desired_position_callback(const geometry_msgs::msg::Point::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Received position request: x=%.2f, y=%.2f, z=%.2f",
            msg->x, msg->y, msg->z);
  Eigen::Vector3d goal_position;
  goal_position << msg->x, msg->y, msg->z;
  Eigen::VectorXd q_cmd = compute_ik(goal_position);
  send_joint_command(q_cmd);
}

//#### Publishing functions/callbacks ###################################################

// Publish joint_commands to ros2_control
void SmartTemplateNode::send_joint_command(const Eigen::VectorXd& q_cmd) {
  last_joint_command_ = q_cmd;
  std_msgs::msg::Float64MultiArray cmd_msg;
  // Fill message with joint commands
  cmd_msg.data.resize(q_cmd.size());
  for (int i = 0; i < q_cmd.size(); ++i) { // Assuming q_cmd is with correct joint order
    cmd_msg.data[i] = 0.001*q_cmd[i]; // Convert from mm to m
  }
  // Publish message
  joint_command_pub_->publish(cmd_msg);
  RCLCPP_DEBUG(this->get_logger(), "Published joint position command [mm]: (%f, %f, %f)", q_cmd[0],q_cmd[1],q_cmd[2]);
}

// Publish robot end-effector pose
void SmartTemplateNode::timer_stage_pose_callback() {
  // Get current joint values [mm]
  Eigen::VectorXd joints_mm = get_joints();
  if (joints_mm.size() == 0) {
    RCLCPP_WARN(this->get_logger(), "Failed to read joint positions.");
    return;
  }
  // Forward kinematics to get stage position
  Eigen::Vector3d pos = compute_fk(joints_mm);

  // Publish stage pose
  geometry_msgs::msg::PointStamped msg;
  msg.header.stamp = this->now();
  msg.header.frame_id = "stage";
  msg.point.x = pos.x();
  msg.point.y = pos.y();
  msg.point.z = pos.z();

  stage_pose_pub_->publish(msg);
  RCLCPP_DEBUG(this->get_logger(),
               "stage_pose [mm]: x=%.2f, y=%.2f, z=%.2f in %s frame",
               msg.point.x, msg.point.y, msg.point.z,
               msg.header.frame_id.c_str());
}

//#### Service functions ###################################################

// Current position service request
void SmartTemplateNode::current_position_callback(
  const std::shared_ptr<smart_template_interfaces::srv::GetPoint::Request> /*request*/,
  std::shared_ptr<smart_template_interfaces::srv::GetPoint::Response> response)
{
  RCLCPP_DEBUG(this->get_logger(), "Received current position request");
  try {
    auto current_position = get_position();
    response->valid = true;
    response->x = current_position[0];
    response->y = current_position[1];
    response->z = current_position[2];
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Error getting current position: %s", e.what());
    response->valid = false;
  }
}

// Command service request
void SmartTemplateNode::command_callback(
  const std::shared_ptr<smart_template_interfaces::srv::Command::Request> request,
  std::shared_ptr<smart_template_interfaces::srv::Command::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Received command request: %s", request->command.c_str());
  if (request->command == "HOME") {
    Eigen::VectorXd q_cmd = Eigen::VectorXd::Zero(3);
    send_joint_command(q_cmd);
    response->response = "Command HOME sent";
  } 
  else if (request->command == "RETRACT") {
    Eigen::VectorXd q_cmd = get_joints();
    RCLCPP_INFO(this->get_logger(), "Current joints %f, %f, %f", q_cmd[0],q_cmd[1],q_cmd[2] );
    q_cmd[1] = 0.0;
    send_joint_command(q_cmd);
    response->response = "Command RETRACT sent";
  } 
  else if (request->command == "ABORT") {
    abort_ = true;
    abort_motion();
    response->response = "Command ABORT sent";
  } 
  else if (request->command == "RESUME") {
    abort_ = false;
    response->response = "Command RESUME sent";
    RCLCPP_INFO(this->get_logger(), "System resumed. Motion re-enabled.");
  } 
  else {
    response->response = "Unknown command: " + request->command;
  }
}

// Move robot service request
void SmartTemplateNode::move_callback(
  const std::shared_ptr<smart_template_interfaces::srv::Move::Request> request,
  std::shared_ptr<smart_template_interfaces::srv::Move::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Received move request: x=%f, y=%f, z=%f, eps=%f",
    request->x, request->y, request->z, request->eps);
  try {
    if (request->eps < 0.0) {
      throw std::invalid_argument("Epsilon cannot be negative");
    }
    Eigen::Vector3d goal_position(request->x, request->y, request->z);
    Eigen::VectorXd q_cmd = compute_ik(goal_position);
    send_joint_command(q_cmd);
    response->response = "Success: Robot moved to the specified position.";
  } catch (const std::exception & e) {
    response->response = "Error: " + std::string(e.what());
  }
}

//#### Action functions ###################################################

rclcpp_action::GoalResponse SmartTemplateNode::handle_goal(
  const rclcpp_action::GoalUUID & /*uuid*/,
  std::shared_ptr<const MoveAndObserve::Goal> /*goal*/)
{
  // Lock mutex and check if there is active goal
  std::lock_guard<std::mutex> lock(goal_mutex_);
  if (abort_) {
    RCLCPP_WARN(this->get_logger(), "Rejecting goal: ABORT active");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (current_goal_handle_ && current_goal_handle_->is_active()) {
    RCLCPP_WARN(this->get_logger(), "Preempting current goal");
    current_goal_handle_->abort(std::make_shared<MoveAndObserve::Result>());
  }
  RCLCPP_INFO(this->get_logger(), "Goal accepted and will be executed");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse SmartTemplateNode::handle_cancel(
  const std::shared_ptr<GoalHandleMoveAndObserve> /*goal_handle*/)
{
  RCLCPP_INFO(this->get_logger(), "Received cancel request");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void SmartTemplateNode::handle_accepted(const std::shared_ptr<GoalHandleMoveAndObserve> goal_handle)
{
  {
    // Lock mutex
    std::lock_guard<std::mutex> lock(goal_mutex_);
    // Abort existing goal if still active
    if (current_goal_handle_ && current_goal_handle_->is_active()) {
      RCLCPP_WARN(this->get_logger(), "Preempting previous goal");
      auto result = std::make_shared<MoveAndObserve::Result>();
      result->error_code = 2;  // Optional: your code for 'preempted'
      current_goal_handle_->abort(result);
    }
    // Cancel previous goal timer
    if (goal_timer_) {
      goal_timer_->cancel();
    }
    current_goal_handle_ = goal_handle;
  }
  RCLCPP_INFO(this->get_logger(), "New goal accepted, scheduling execution...");
  auto timer = this->create_wall_timer(
    std::chrono::milliseconds(1),
    [this, goal_handle]() {
      if (goal_timer_) goal_timer_->cancel();  // Cleanup after firing
      this->execute_goal(goal_handle);
    },
    action_callback_group_);
  goal_timer_ = timer;
}

void SmartTemplateNode::execute_goal(
  const std::shared_ptr<GoalHandleMoveAndObserve> goal_handle)
{
  RCLCPP_DEBUG(this->get_logger(), "Executing move_and_observe goal");
  // Cancel old goals if existant
  {
    std::lock_guard<std::mutex> lock(goal_mutex_);
    if (goal_handle != current_goal_handle_) {
      RCLCPP_WARN(this->get_logger(), "Skipping execution of outdated goal");
      return;
    }
  }  
  auto result = std::make_shared<MoveAndObserve::Result>();
  auto feedback = std::make_shared<MoveAndObserve::Feedback>();
  // Get goal parameters
  const auto goal = goal_handle->get_goal();
  Eigen::Vector3d goal_position(goal->x, goal->y, goal->z);
  Eigen::VectorXd q_cmd = compute_ik(goal_position);
  RCLCPP_INFO(this->get_logger(), "Goal [mm]: x=%f, y=%f, z=%f, eps=%f", goal->x, goal->y, goal->z, goal->eps);
  // Send movement command
  send_joint_command(q_cmd);
  // Start timeout timer
  auto start_time = this->now();
  const std::chrono::seconds timeout(static_cast<int>(TIMEOUT));
  result->error_code = 0;
  // Feedback loop (while goal is not reached or timeout)
  while (rclcpp::ok()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    // Check if aborted
    if (abort_) {
      goal_handle->abort(result);
      result->error_code = 2; // abort
      RCLCPP_WARN(this->get_logger(), "Goal aborted");
      break;
    }    
    // Check for cancelation
    if (goal_handle->is_canceling()) {
      abort_motion(); // Not setting abort_ to true (no need to resume, just stopping)
      result->error_code = 2; // abort
      goal_handle->canceled(result);
      RCLCPP_WARN(this->get_logger(), "Goal canceled during execution");
      break;
    }
    // Check current joints error
    Eigen::VectorXd err_joints = get_joints_err();
    bool goal_reached = true;
    for (double e : err_joints) {
      if (std::abs(e) > goal->eps) {
        goal_reached = false;
        break;
      }
    }
    if (goal_reached) {
      result->error_code = 0;
      goal_handle->succeed(result);    
      RCLCPP_INFO(this->get_logger(), "Goal reached successfully");
      break;
    }
    // Publish feedback
    Eigen::Vector3d  current_position = get_position();
    feedback->x = current_position[0];
    feedback->y = current_position[1];
    feedback->z = current_position[2];
    feedback->error = error_3d(goal_position, current_position);
    goal_handle->publish_feedback(feedback);
    // Check for timeout
    if ((this->now() - start_time) > timeout) {
      abort_motion(); // Not setting abort_ to true (no need to resume, just stopping)
      result->error_code = 1; // timeout
      goal_handle->abort(result);
      RCLCPP_WARN(this->get_logger(), "Goal timed out");
      break;
    }
  }
  // Publish result
  Eigen::Vector3d current_position = get_position();
  result->x = current_position[0];
  result->y = current_position[1];
  result->z = current_position[2];
  result->error = error_3d(goal_position, current_position);
  result->time = (this->now() - start_time).seconds();
  // Cleanup: reset current goal handle
  {
    std::lock_guard<std::mutex> lock(goal_mutex_);
    current_goal_handle_.reset();
  }  
  RCLCPP_INFO(this->get_logger(), "Finished move_and_observe action");
}

} // namespace smart_template_cpp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto smart_template_node = std::make_shared<smart_template_cpp::SmartTemplateNode>();
  // Use a multithreaded executor to enable processing goals concurrently
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(smart_template_node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
