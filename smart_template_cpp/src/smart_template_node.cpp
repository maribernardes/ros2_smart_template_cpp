#include "smart_template_cpp/smart_template_node.hpp"

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <gclib.h>
#include <gclib_errors.h>
#include <gclib_record.h>

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
  action_server_ = rclcpp_action::create_server<MoveAndObserve>(
    this,
    "/stage/move_and_observe",
    std::bind(&SmartTemplateNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&SmartTemplateNode::handle_cancel, this, std::placeholders::_1),
    std::bind(&SmartTemplateNode::handle_accepted, this, std::placeholders::_1));
  
  // Initialize Galil connection
  try {
    GOpen("192.168.0.99", &galil_);
    
    // Initial position is always set to (0,0,0)
    char buffer[1024];
    GSize bytes_returned;
    GCommand(galil_, "DPA=0", buffer, sizeof(buffer), &bytes_returned);
    GCommand(galil_, "DPB=0", buffer, sizeof(buffer), &bytes_returned);
    GCommand(galil_, "DPC=0", buffer, sizeof(buffer), &bytes_returned);
    GCommand(galil_, "PTA=1", buffer, sizeof(buffer), &bytes_returned);
    GCommand(galil_, "PTB=1", buffer, sizeof(buffer), &bytes_returned);
    GCommand(galil_, "PTC=1", buffer, sizeof(buffer), &bytes_returned);
    
    // Set joint names
    joint_names_ = {"horizontal_joint", "insertion_joint", "vertical_joint"};
    
    RCLCPP_INFO(this->get_logger(), "SmartTemplate ready");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize Galil connection: %s", e.what());
  }
}

SmartTemplateNode::~SmartTemplateNode()
{
  if (galil_) {
    GClose(galil_);
  }
}

void SmartTemplateNode::timer_stage_pose_callback()
{
  try {
    // Read guide position from robot motors
    auto position = get_position();
    
    // Construct and publish robot message
    auto msg = std::make_unique<geometry_msgs::msg::PointStamped>();
    msg->header.stamp = this->now();
    msg->header.frame_id = "stage";
    msg->point.x = position[0];
    msg->point.y = position[1];
    msg->point.z = position[2];
    publisher_stage_pose_->publish(std::move(msg));
    
    // Update and publish joint_state message
    auto joint_state_msg = std::make_unique<sensor_msgs::msg::JointState>();
    joint_state_msg->header.stamp = this->now();
    joint_state_msg->name = joint_names_;
    joint_state_msg->position = {0.001 * position[0], 0.001 * position[1], 0.001 * position[2]}; // Convert from mm to m
    publisher_joint_states_->publish(std::move(joint_state_msg));
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Error in timer callback: %s", e.what());
  }
}

std::vector<double> SmartTemplateNode::get_position()
{
  try {
    std::vector<double> position(3, 0.0);
    char buffer[1024];
    GSize bytes_returned;
    GCommand(galil_, "TP", buffer, sizeof(buffer), &bytes_returned);
    
    // Parse the response
    std::vector<double> values;
    char * token = strtok(buffer, ",");
    while (token != nullptr) {
      values.push_back(std::stod(token));
      token = strtok(nullptr, ",");
    }
    
    // WARNING: Galil channel B inverted, that is why the value is negative (to be verified)
    if (values.size() >= 3) {
      position[0] = values[0] * COUNT_2_MM_X; // CHANNEL A (horizontal)
      position[1] = values[2] * COUNT_2_MM_Y; // CHANNEL C (insertion)
      position[2] = values[1] * COUNT_2_MM_Z; // CHANNEL B (vertical)
    }
    
    return position;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Error getting position: %s", e.what());
    return {0.0, 0.0, 0.0};
  }
}

std::vector<double> SmartTemplateNode::tell_error()
{
  try {
    std::vector<double> errors(3, 0.0);
    char buffer[1024];
    GSize bytes_returned;
    GCommand(galil_, "TE", buffer, sizeof(buffer), &bytes_returned);
    
    // Parse the response
    std::vector<int> values;
    char * token = strtok(buffer, ",");
    while (token != nullptr) {
      values.push_back(std::stoi(token));
      token = strtok(nullptr, ",");
    }
    
    if (values.size() >= 3) {
      errors[0] = values[0]; // X error
      errors[1] = values[2]; // Y error
      errors[2] = values[1]; // Z error
    }
    
    RCLCPP_INFO(this->get_logger(), "errX=%f, errY=%f, errZ=%f", errors[0], errors[1], errors[2]);
    return errors;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Error getting position error: %s", e.what());
    return {0.0, 0.0, 0.0};
  }
}

void SmartTemplateNode::abort_motion()
{
  try {
    char buffer[1024];
    GSize bytes_returned;
    GCommand(galil_, "SH", buffer, sizeof(buffer), &bytes_returned);
    RCLCPP_INFO(this->get_logger(), "ABORT");
    abort_ = true;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Error aborting motion: %s", e.what());
  }
}

void SmartTemplateNode::send_movement(const std::vector<double> & goal)
{
  send_movement_in_counts(goal[0] * MM_2_COUNT_X, "A"); // X = CH_A
  send_movement_in_counts(goal[2] * MM_2_COUNT_Z, "B"); // Z = CH_B
  send_movement_in_counts(goal[1] * MM_2_COUNT_Y, "C"); // Y = CH_C
}

double SmartTemplateNode::check_limits(double x, const std::string & channel)
{
  if (channel == "C") {
    return x;
  }
  
  if (x > SAFE_LIMIT * MM_2_COUNT_X) {
    RCLCPP_INFO(this->get_logger(), "Limit reach at axis %s", channel.c_str());
    return SAFE_LIMIT * MM_2_COUNT_X;
  } else if (x < -SAFE_LIMIT * MM_2_COUNT_X) {
    RCLCPP_INFO(this->get_logger(), "Limit reach at axis %s", channel.c_str());
    return -SAFE_LIMIT * MM_2_COUNT_X;
  }
  
  return x;
}

bool SmartTemplateNode::send_movement_in_counts(double x, const std::string & channel)
{
  try {
    x = check_limits(x, channel);
    std::string command = "PA" + channel + "=" + std::to_string(static_cast<int>(x));
    char buffer[1024];
    GSize bytes_returned;
    GCommand(galil_, command.c_str(), buffer, sizeof(buffer), &bytes_returned);
    RCLCPP_INFO(this->get_logger(), "Sent to Galil %s", command.c_str());
    return true;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Error sending movement: %s", e.what());
    return false;
  }
}

double SmartTemplateNode::distance_positions(
  const std::vector<double> & goal, const std::vector<double> & position)
{
  return std::sqrt(
    std::pow(goal[0] - position[0], 2) +
    std::pow(goal[1] - position[1], 2) +
    std::pow(goal[2] - position[2], 2));
}

void SmartTemplateNode::command_callback(
  const std::shared_ptr<smart_template_interfaces::srv::Command::Request> request,
  std::shared_ptr<smart_template_interfaces::srv::Command::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Received command request: %s", request->command.c_str());
  
  if (request->command == "HOME") {
    std::vector<double> goal = {0.0, 0.0, 0.0};
    send_movement(goal);
    response->response = "Command HOME sent";
  } else if (request->command == "RETRACT") {
    auto position = get_position();
    std::vector<double> goal = {position[0], 0.0, position[2]};
    send_movement(goal);
    response->response = "Command RETRACT sent";
  } else if (request->command == "ABORT") {
    abort_motion();
    response->response = "Command ABORT sent";
  } else {
    response->response = "Unknown command: " + request->command;
  }
}

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
    
    std::vector<double> goal = {request->x, request->y, request->z};
    send_movement(goal);
    response->response = "Success: Robot moved to the specified position.";
  } catch (const std::exception & e) {
    response->response = "Error: " + std::string(e.what());
  }
}

void SmartTemplateNode::current_position_callback(
  const std::shared_ptr<smart_template_interfaces::srv::GetPoint::Request> /*request*/,
  std::shared_ptr<smart_template_interfaces::srv::GetPoint::Response> response)
{
  RCLCPP_DEBUG(this->get_logger(), "Received current position request");
  
  try {
    auto position = get_position();
    response->valid = true;
    response->x = position[0];
    response->y = position[1];
    response->z = position[2];
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Error getting position: %s", e.what());
    response->valid = false;
  }
}

rclcpp_action::GoalResponse SmartTemplateNode::handle_goal(
  const rclcpp_action::GoalUUID & /*uuid*/,
  std::shared_ptr<const MoveAndObserve::Goal> /*goal*/)
{
  RCLCPP_DEBUG(this->get_logger(), "Received goal request");
  // This server allows multiple goals in parallel
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse SmartTemplateNode::handle_cancel(
  const std::shared_ptr<GoalHandleMoveAndObserve> /*goal_handle*/)
{
  RCLCPP_INFO(this->get_logger(), "Received cancel request");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void SmartTemplateNode::handle_accepted(
  const std::shared_ptr<GoalHandleMoveAndObserve> goal_handle)
{
  // Create a new thread to execute the action
  std::thread{std::bind(&SmartTemplateNode::execute_goal, this, std::placeholders::_1), goal_handle}.detach();
}

void SmartTemplateNode::execute_goal(
  const std::shared_ptr<GoalHandleMoveAndObserve> goal_handle)
{
  RCLCPP_DEBUG(this->get_logger(), "Executing move_and_observe goal");
  
  auto result = std::make_shared<MoveAndObserve::Result>();
  auto feedback = std::make_shared<MoveAndObserve::Feedback>();
  
  // Check if goal is being canceled
  if (goal_handle->is_canceling()) {
    goal_handle->canceled(result);
    RCLCPP_INFO(this->get_logger(), "Goal canceled");
    return;
  }
  
  // Get goal parameters
  const auto goal = goal_handle->get_goal();
  std::vector<double> goal_position = {goal->x, goal->y, goal->z};
  RCLCPP_INFO(this->get_logger(), "Goal: x=%f, y=%f, z=%f, eps=%f", goal->x, goal->y, goal->z, goal->eps);
  
  // Send movement command
  send_movement(goal_position);
  
  // Start timeout timer
  auto start_time = this->now();
  const std::chrono::seconds timeout(static_cast<int>(TIMEOUT));
  result->error_code = 0;
  abort_ = false;
  
  // Feedback loop (while goal is not reached or timeout)
  while (rclcpp::ok()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // Check for cancelation
    if (goal_handle->is_canceling()) {
      abort_motion();
      goal_handle->canceled(result);
      RCLCPP_INFO(this->get_logger(), "Goal canceled during execution");
      return;
    }
    
    // Check current position error
    auto err = tell_error();
    
    // Check if aborted
    if (abort_) {
      goal_handle->abort(result);
      result->error_code = 2; // abort
      RCLCPP_INFO(this->get_logger(), "Goal aborted");
      abort_ = false;
      break;
    }
    
    // Check if reached target
    if (std::abs(err[0]) <= goal->eps * ERROR_GAIN &&
        std::abs(err[1]) <= goal->eps * ERROR_GAIN &&
        std::abs(err[2]) <= goal->eps * ERROR_GAIN) {
      goal_handle->succeed(result);
      result->error_code = 0;
      RCLCPP_INFO(this->get_logger(), "Goal reached successfully");
      break;
    }
    
    // Check for timeout
    if ((this->now() - start_time) > timeout) {
      goal_handle->abort(result);
      result->error_code = 1; // timeout
      RCLCPP_INFO(this->get_logger(), "Goal timed out");
      break;
    }
  }
  
  // Set final result values
  auto position = get_position();
  result->x = position[0];
  result->y = position[1];
  result->z = position[2];
  result->error = distance_positions(goal_position, position);
  result->time = (this->now() - start_time).seconds();
  
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