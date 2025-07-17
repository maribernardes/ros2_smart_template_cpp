#include "smart_template_node.hpp"
#include <rclcpp/executors/multi_threaded_executor.hpp>

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <gclib.h>
#include <gclib_errors.h>
#include <gclib_record.h>

#include "tinyxml2.h"  // for URDF parsing
#include <algorithm>
#include <unordered_map>

extern "C" {
  GReturn GOpen(GCStringIn connection_string, GCon* g_connection);
  GReturn GClose(GCon g_connection);
  GReturn GCommand(GCon g_connection, GCStringIn command, GCStringOut buffer, GSize buffer_len, GSize* bytes_returned);
}

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

  // Initialize publishers
  publisher_stage_pose_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
    "/stage/state/guide_pose", 10);
  publisher_joint_states_ = this->create_publisher<sensor_msgs::msg::JointState>(
    "/joint_states", 10);
  
  // Initialize timer
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
  
  // Initialize Galil connection
  initialize_galil();
}

SmartTemplateNode::~SmartTemplateNode()
{
  if (galil_) {
    GClose(galil_);
  }
}

// Initialize Galil connection
void SmartTemplateNode::initialize_galil() {
  try {
    GOpen("192.168.0.99", &galil_);
    char buffer[1024];
    GSize bytes_returned;
    // Zero encoder positions
    GCommand(galil_, "DPA=0", buffer, sizeof(buffer), &bytes_returned);
    GCommand(galil_, "DPB=0", buffer, sizeof(buffer), &bytes_returned);
    GCommand(galil_, "DPC=0", buffer, sizeof(buffer), &bytes_returned);
    // Set PTA/B/C = 1 (position tracking)
    GCommand(galil_, "PTA=1", buffer, sizeof(buffer), &bytes_returned);
    GCommand(galil_, "PTB=1", buffer, sizeof(buffer), &bytes_returned);
    GCommand(galil_, "PTC=1", buffer, sizeof(buffer), &bytes_returned);
    RCLCPP_INFO(this->get_logger(), "Galil connection initialized successfully.");
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize Galil connection: %s", e.what());
  }
}

//#### Kinematic model ###################################################

// Forward-kinematics funtion
std::array<double, 3> SmartTemplateNode::fk_model(const std::vector<double>& joints_mm) const {
  if (joints_mm.size() != 3) {
    RCLCPP_ERROR(this->get_logger(), "FK: Expected 3 joint values, got %zu", joints_mm.size());
    return {0.0, 0.0, 0.0};
  }
  // Order: [horizontal, insertion, vertical]
  double x = joints_mm[0];
  double y = joints_mm[1];
  double z = joints_mm[2];
  return {x, y, z};
}

// Inverse-kinematics function
std::vector<double> SmartTemplateNode::ik_model(const std::array<double, 3>& position_mm) const {
  // Directly map Cartesian [x, y, z] to joint values
  std::vector<double> joints_mm = {
    position_mm[0],  // horizontal
    position_mm[1],  // insertion
    position_mm[2]   // vertical
  };
  // Clamp to joint limits
  std::vector<double> safe_joints = check_limits(joints_mm);
  // Return as array
  return { safe_joints[0], safe_joints[1], safe_joints[2] };
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
  return true;
}

// Get current robot joint values [mm]
std::vector<double> SmartTemplateNode::get_joints() {
  try {
    char buffer[1024];
    GSize bytes_returned;
    // Query encoder positions from Galil
    GCommand(galil_, "TP", buffer, sizeof(buffer), &bytes_returned);
    // Parse comma-separated counts
    std::vector<double> counts;
    char* token = strtok(buffer, ",");
    while (token != nullptr && counts.size() < joint_info_.names.size()) {
      counts.push_back(static_cast<double>(std::stoi(token)));
      token = strtok(nullptr, ",");
    }
    if (counts.size() != joint_info_.names.size()) {
      RCLCPP_WARN(this->get_logger(),
                  "Expected %zu joints, got %zu",
                  joint_info_.names.size(), counts.size());
      return std::vector<double>(joint_info_.names.size(), 0.0);
    }
    // Convert counts to mm
    std::vector<double> joint_values_mm(joint_info_.names.size(), 0.0);
    for (size_t i = 0; i < joint_info_.names.size(); ++i) {
      joint_values_mm[i] = counts[i] * joint_info_.count_to_mm[i];
    }
    return joint_values_mm;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to get joint values: %s", e.what());
    return std::vector<double>(joint_info_.names.size(), 0.0);
  }
}

// Get current joints error [mm]
std::vector<double> SmartTemplateNode::get_joints_err() {
  try {
    char buffer[1024];
    GSize bytes_returned;
    // Query encoder positions from Galil
    GCommand(galil_, "TE", buffer, sizeof(buffer), &bytes_returned);
    // Parse comma-separated counts
    std::vector<double> counts;
    char* token = strtok(buffer, ",");
    while (token != nullptr && counts.size() < joint_info_.names.size()) {
      counts.push_back(static_cast<double>(std::stoi(token)));
      token = strtok(nullptr, ",");
    }
    if (counts.size() != joint_info_.names.size()) {
      RCLCPP_WARN(this->get_logger(),
                  "Expected %zu joints, got %zu",
                  joint_info_.names.size(), counts.size());
      return std::vector<double>(joint_info_.names.size(), 0.0);
    }
    // Convert counts to mm
    std::vector<double> joint_err_mm(joint_info_.names.size(), 0.0);
    for (size_t i = 0; i < joint_info_.names.size(); ++i) {
      joint_err_mm[i] = counts[i] * joint_info_.count_to_mm[i];
    }
    std::ostringstream oss;
    oss << "Joint errors [mm]: ";
    for (double err : joint_err_mm) {
      oss << std::fixed << std::setprecision(3) << err << " ";
    }
    RCLCPP_DEBUG(this->get_logger(), "%s", oss.str().c_str());
    return joint_err_mm;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to get joint errors: %s", e.what());
    return std::vector<double>(joint_info_.names.size(), 0.0);
  }
}

// Get current robot position (end-effector)
std::array<double, 3> SmartTemplateNode::get_position() {
  std::vector<double> joints_mm = get_joints();
  if (joints_mm.empty()) {
    RCLCPP_WARN(this->get_logger(), "get_position(): No joint data available");
    return {0.0, 0.0, 0.0};
  }
  return fk_model(joints_mm);
}

// Get error (euclidean distance) between two 3D coordinates
double SmartTemplateNode::error_3d(const std::array<double, 3>& a, const std::array<double, 3>& b) const {
  double dx = a[0] - b[0];
  double dy = a[1] - b[1];
  double dz = a[2] - b[2];
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

// Abort motion
void SmartTemplateNode::abort_motion()
{
  try {
    char buffer[1024];
    GSize bytes_returned;
    GCommand(galil_, "SH", buffer, sizeof(buffer), &bytes_returned);
    RCLCPP_WARN(this->get_logger(), "ABORT sent to Galil");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Error aborting motion: %s", e.what());
  }
}

//Check joints limits
std::vector<double> SmartTemplateNode::check_limits(const std::vector<double>& joint_values) const {
  if (joint_values.size() != joint_info_.names.size()) {
    RCLCPP_ERROR(this->get_logger(), "check_limits: joint count mismatch (%zu != %zu)",
                 joint_values.size(), joint_info_.names.size());
    throw std::runtime_error("check_limits: vector size mismatch");
  }
  std::vector<double> capped_values;
  for (size_t i = 0; i < joint_values.size(); ++i) {
    double lower = joint_info_.limits_lower[i];
    double upper = joint_info_.limits_upper[i];
    double value = joint_values[i];
    if (value < lower || value > upper) {
      RCLCPP_WARN(this->get_logger(),
                  "%s value %.2f mm out of bounds [%.2f, %.2f] — clipping.",
                  joint_info_.names[i].c_str(), value, lower, upper);
    }
    capped_values.push_back(std::clamp(value, lower, upper));
  }
  return capped_values;
}

// Sends a movement command to all joints based on the goal [x, y, z] in mm.
void SmartTemplateNode::position_control(const std::array<double, 3> & goal)
{
  if (abort_) {
    RCLCPP_WARN(this->get_logger(), "Motion command ignored: ABORT active");
    return;
  }
  // Compute joint values from Cartesian target
  std::vector<double> joints_mm = ik_model(goal);
  // Enforce joint limits
  joints_mm = check_limits(joints_mm);
  // Convert mm to encoder counts and send to Galil
  for (size_t i = 0; i < joint_info_.names.size(); ++i) {
    int count_value = static_cast<int>(std::round(joints_mm[i] * joint_info_.mm_to_count[i]));
    std::string command = "PA" + joint_info_.channels[i] + "=" + std::to_string(count_value);
    try {
      char buffer[1024];
      GSize bytes_returned;
      GCommand(galil_, command.c_str(), buffer, sizeof(buffer), &bytes_returned);
      RCLCPP_INFO(this->get_logger(), "Sent to Galil: %s", command.c_str());
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(),
                   "Failed to send to channel %s: %s",
                   joint_info_.channels[i].c_str(), e.what());
    }
  }
}

//#### Listening callbacks ###################################################

// Publish robot joints and end-effector pose
void SmartTemplateNode::timer_stage_pose_callback() {
  // Get current joint values [mm]
  std::vector<double> joints_mm = get_joints();
  if (joints_mm.empty()) {
    RCLCPP_WARN(this->get_logger(), "Failed to read joint positions.");
    return;
  }
  // Forward kinematics to get stage position
  std::array<double, 3> position = fk_model(joints_mm);
  // Publish stage pose
  geometry_msgs::msg::PointStamped msg;
  msg.header.stamp = this->now();
  msg.header.frame_id = "stage";
  msg.point.x = position[0];
  msg.point.y = position[1];
  msg.point.z = position[2];
  publisher_stage_pose_->publish(msg);
  RCLCPP_DEBUG(this->get_logger(),
               "stage_pose [mm]: x=%.2f, y=%.2f, z=%.2f in %s frame",
               msg.point.x, msg.point.y, msg.point.z,
               msg.header.frame_id.c_str());
  // Publish joint states (converted to meters)
  sensor_msgs::msg::JointState joint_msg;
  joint_msg.header.stamp = this->now();
  joint_msg.name = joint_info_.names;
  joint_msg.position.resize(joints_mm.size());
  for (size_t i = 0; i < joints_mm.size(); ++i) {
    joint_msg.position[i] = joints_mm[i] * 0.001;  // mm to m
  }
  publisher_joint_states_->publish(joint_msg);
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
    std::array<double, 3> goal = {0.0, 0.0, 0.0};
    position_control(goal);
    response->response = "Command HOME sent";
  } else if (request->command == "RETRACT") {
    auto position = get_position();
    std::array<double, 3> goal = {position[0], 0.0, position[2]};
    position_control(goal);
    response->response = "Command RETRACT sent";
  } else if (request->command == "ABORT") {
    abort_ = true;
    abort_motion();
    response->response = "Command ABORT sent";
  } else if (request->command == "RESUME") {
      abort_ = false;
      response->response = "Command RESUME sent";
      RCLCPP_INFO(this->get_logger(), "System resumed. Motion re-enabled.");
    } else {
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
    std::array<double, 3> goal = {request->x, request->y, request->z};
    position_control(goal);
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
  std::array<double, 3> goal_position = {goal->x, goal->y, goal->z};
  RCLCPP_INFO(this->get_logger(), "Goal: x=%f, y=%f, z=%f, eps=%f", goal->x, goal->y, goal->z, goal->eps);
  // Send movement command
  position_control(goal_position);
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
    std::vector<double> err_joints = get_joints_err();
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
    auto current_position = get_position();
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
  auto current_position = get_position();
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
