#ifndef SMART_TEMPLATE_CPP__SMART_TEMPLATE_NODE_HPP_
#define SMART_TEMPLATE_CPP__SMART_TEMPLATE_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "smart_template_cpp/srv/command.hpp"
#include "smart_template_cpp/srv/move.hpp"
#include "smart_template_cpp/srv/get_point.hpp"
#include "smart_template_cpp/action/move_and_observe.hpp"
#include "ros2_igtl_bridge/msg/transform.hpp"

// Forward declaration for gclib
typedef void* GCon;

namespace smart_template_cpp
{

class SmartTemplateNode : public rclcpp::Node
{
public:
  using MoveAndObserve = smart_template_cpp::action::MoveAndObserve;
  using GoalHandleMoveAndObserve = rclcpp_action::ServerGoalHandle<MoveAndObserve>;

  SmartTemplateNode();
  virtual ~SmartTemplateNode();

private:
  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr publisher_stage_pose_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_joint_states_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Service servers
  rclcpp::Service<smart_template_cpp::srv::Command>::SharedPtr command_server_;
  rclcpp::Service<smart_template_cpp::srv::Move>::SharedPtr move_server_;
  rclcpp::Service<smart_template_cpp::srv::GetPoint>::SharedPtr current_position_server_;

  // Action server
  rclcpp_action::Server<MoveAndObserve>::SharedPtr action_server_;

  // Galil communication
  GCon galil_;
  bool abort_;
  std::vector<std::string> joint_names_;
  
  // Constants for unit conversion
  const double MM_2_COUNT_X = 715.0;  // Horizontal
  const double COUNT_2_MM_X = 0.0014;
  const double MM_2_COUNT_Y = -2000.0/1.27;  // Depth
  const double COUNT_2_MM_Y = -0.0005*1.27;
  const double MM_2_COUNT_Z = 1430.0;  // Vertical
  const double COUNT_2_MM_Z = 0.0007;
  const double SAFE_LIMIT = 60.0;
  const double ERROR_GAIN = 500.0;
  const double TIMEOUT = 30.0;

  // Callback functions
  void timer_stage_pose_callback();
  
  // Service callbacks
  void command_callback(
    const std::shared_ptr<smart_template_cpp::srv::Command::Request> request,
    std::shared_ptr<smart_template_cpp::srv::Command::Response> response);

  void move_callback(
    const std::shared_ptr<smart_template_cpp::srv::Move::Request> request,
    std::shared_ptr<smart_template_cpp::srv::Move::Response> response);

  void current_position_callback(
    const std::shared_ptr<smart_template_cpp::srv::GetPoint::Request> request,
    std::shared_ptr<smart_template_cpp::srv::GetPoint::Response> response);

  // Action callbacks
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const MoveAndObserve::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleMoveAndObserve> goal_handle);

  void handle_accepted(
    const std::shared_ptr<GoalHandleMoveAndObserve> goal_handle);

  void execute_goal(
    const std::shared_ptr<GoalHandleMoveAndObserve> goal_handle);

  // Internal helper functions
  std::vector<double> get_position();
  std::vector<double> tell_error();
  void abort_motion();
  void send_movement(const std::vector<double> & goal);
  double check_limits(double x, const std::string & channel);
  bool send_movement_in_counts(double x, const std::string & channel);
  double distance_positions(const std::vector<double> & goal, const std::vector<double> & position);
};

} // namespace smart_template_cpp

#endif // SMART_TEMPLATE_CPP__SMART_TEMPLATE_NODE_HPP_