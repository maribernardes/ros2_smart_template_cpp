#ifndef SMART_TEMPLATE_CPP__SMART_TEMPLATE_NODE_HPP_
#define SMART_TEMPLATE_CPP__SMART_TEMPLATE_NODE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <array>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "smart_template_interfaces/srv/command.hpp"
#include "smart_template_interfaces/srv/move.hpp"
#include "smart_template_interfaces/srv/get_point.hpp"
#include "smart_template_interfaces/action/move_and_observe.hpp"


// Forward declaration for gclib
typedef void* GCon;

namespace smart_template_cpp
{

class SmartTemplateNode : public rclcpp::Node
{
public:
  using JointState = sensor_msgs::msg::JointState;
  using MoveAndObserve = smart_template_interfaces::action::MoveAndObserve;
  using GoalHandleMoveAndObserve = rclcpp_action::ServerGoalHandle<MoveAndObserve>;
  using Command = smart_template_interfaces::srv::Command;
  using Move = smart_template_interfaces::srv::Move;
  using GetPoint = smart_template_interfaces::srv::GetPoint;

  SmartTemplateNode();
  virtual ~SmartTemplateNode();

private:
  // Struct to hold joint metadata (mimicking Python version)
  struct JointInfo {
    std::vector<std::string> names;
    std::vector<std::string> channels;
    std::vector<double> limits_lower;
    std::vector<double> limits_upper;
    std::vector<double> mm_to_count;
    std::vector<double> count_to_mm;

    void add(const std::string& name, const std::string& channel,
             double lower, double upper, double mm_to_count_val) {
      names.push_back(name);
      channels.push_back(channel);
      limits_lower.push_back(lower);
      limits_upper.push_back(upper);
      mm_to_count.push_back(mm_to_count_val);
      count_to_mm.push_back(mm_to_count_val != 0.0 ? 1.0 / mm_to_count_val : 0.0);
    }

    int index(const std::string& joint_name) const {
      auto it = std::find(names.begin(), names.end(), joint_name);
      return (it != names.end()) ? std::distance(names.begin(), it) : -1;
    }
  };
  // JointInfo internal object
  JointInfo joint_info_;

  // FK/IK functions
  std::array<double, 3> fk_model(const std::vector<double>& joints_mm) const;
  std::vector<double> ik_model(const std::array<double, 3>& position_mm) const;

  // Helper to parse URDF into JointInfo
  bool parse_joint_info_from_urdf(const std::string& urdf_str);

  // Subscribers
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr desired_position_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr desired_command_sub_;

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr publisher_stage_pose_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_joint_states_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Service servers
  rclcpp::Service<smart_template_interfaces::srv::Command>::SharedPtr command_server_;
  rclcpp::Service<smart_template_interfaces::srv::Move>::SharedPtr move_server_;
  rclcpp::Service<smart_template_interfaces::srv::GetPoint>::SharedPtr current_position_server_;

  // Action server
  rclcpp_action::Server<MoveAndObserve>::SharedPtr action_server_;
  std::shared_ptr<GoalHandleMoveAndObserve> current_goal_handle_;
  std::mutex goal_mutex_;
  rclcpp::TimerBase::SharedPtr goal_timer_;
  rclcpp::CallbackGroup::SharedPtr action_callback_group_;

  // Galil communication
  GCon galil_;
  std::atomic<bool> abort_{false};
  
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


  // Subscriber callbacks
  void desired_position_callback(const geometry_msgs::msg::Point::SharedPtr msg);
  void desired_command_callback(const std_msgs::msg::String::SharedPtr msg);

  // Publisher callbacks
  void timer_stage_pose_callback();
  
  // Service callbacks
  void command_callback(
    const std::shared_ptr<smart_template_interfaces::srv::Command::Request> request,
    std::shared_ptr<smart_template_interfaces::srv::Command::Response> response);

  void move_callback(
    const std::shared_ptr<smart_template_interfaces::srv::Move::Request> request,
    std::shared_ptr<smart_template_interfaces::srv::Move::Response> response);

  void current_position_callback(
    const std::shared_ptr<smart_template_interfaces::srv::GetPoint::Request> request,
    std::shared_ptr<smart_template_interfaces::srv::GetPoint::Response> response);

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
  void initialize_galil();
  std::vector<double> get_joints();
  std::vector<double> get_joints_err();
  std::array<double, 3> get_position();
  double error_3d(const std::array<double, 3>& a,const std::array<double, 3>& b) const;
  void abort_motion();
  std::vector<double> check_limits(const std::vector<double>& joint_values) const;
  void position_control(const std::array<double, 3> & goal);
};

} // namespace smart_template_cpp

#endif // SMART_TEMPLATE_CPP__SMART_TEMPLATE_NODE_HPP_