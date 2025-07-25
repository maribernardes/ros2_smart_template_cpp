#ifndef SMART_TEMPLATE_CPP__SMART_TEMPLATE_NODE_HPP_
#define SMART_TEMPLATE_CPP__SMART_TEMPLATE_NODE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <array>
#include <unordered_map>

#include <Eigen/Core>
#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "smart_template_interfaces/srv/command.hpp"
#include "smart_template_interfaces/srv/move.hpp"
#include "smart_template_interfaces/srv/get_point.hpp"
#include "smart_template_interfaces/action/move_and_observe.hpp"


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
    std::unordered_map<std::string, int> name_to_index_;

    void add(const std::string& name, const std::string& channel,
             double lower, double upper, double mm_to_count_val) {
      names.push_back(name);
      channels.push_back(channel);
      limits_lower.push_back(lower);
      limits_upper.push_back(upper);
      mm_to_count.push_back(mm_to_count_val);
      count_to_mm.push_back(mm_to_count_val != 0.0 ? 1.0 / mm_to_count_val : 0.0);
    }
    void finalize() { // Call this after all add() calls to build fast index lookup
      name_to_index_.clear();
      for (size_t i = 0; i < names.size(); ++i) {
        name_to_index_[names[i]] = i;
      }
    }
    int index(const std::string& joint_name) const {
      auto it = name_to_index_.find(joint_name);
      return (it != name_to_index_.end()) ? static_cast<int>(it->second) : -1;
    }
    size_t size() const {
      return names.size();
    }
  };

  JointInfo joint_info_;                // Helper structure with joint names, limits, etc.
  Eigen::VectorXd joint_positions_;     // Joint state storage (matches JointInfo order)
  Eigen::VectorXd last_joint_command_;  // Joint position command storage (matches JointInfo order)

  // FK/IK functions
  Eigen::Vector3d compute_fk(const Eigen::VectorXd& joints_mm) const;
  Eigen::VectorXd compute_ik(const Eigen::Vector3d& position_mm) const;

  // Internal helper functions
  bool parse_joint_info_from_urdf(const std::string& urdf_str); // Parse URDF info for joints
  Eigen::VectorXd get_joints() const;                           // Getter for current joint positions
  Eigen::VectorXd get_joints_err() const;                       // Getter for joint error (current - last sent)
  Eigen::Vector3d get_position() const;                         // Get EE position from joint values
  double error_3d(const Eigen::Vector3d& a, const Eigen::Vector3d& b) const;  // Euclidean distance between two 3D points
  Eigen::VectorXd check_limits(const Eigen::VectorXd& joint_values) const;    // Check each joint limit
  void abort_motion();

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr desired_position_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr desired_command_sub_;

  // Publishers
  //std::vector<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> joint_command_pubs_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_command_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr stage_pose_pub_;
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
  const double TIMEOUT = 30.0;

  // Abort state
  std::atomic<bool> abort_{false};
  
  // Subscriber callbacks
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void desired_position_callback(const geometry_msgs::msg::Point::SharedPtr msg);
  void desired_command_callback(const std_msgs::msg::String::SharedPtr msg);

  // Publisher functions/callbacks
  void send_joint_command(const Eigen::VectorXd& q_cmd);
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
};

} // namespace smart_template_cpp

#endif // SMART_TEMPLATE_CPP__SMART_TEMPLATE_NODE_HPP_