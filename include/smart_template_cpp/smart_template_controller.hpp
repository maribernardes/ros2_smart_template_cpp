#ifndef SMART_TEMPLATE_CPP__SMART_TEMPLATE_CONTROLLER_HPP_
#define SMART_TEMPLATE_CPP__SMART_TEMPLATE_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"

namespace smart_template_cpp
{

class SmartTemplateController : public controller_interface::ControllerInterface
{
public:
  SmartTemplateController();

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
    
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_init() override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

protected:
  std::vector<std::string> joint_names_;
  std::string interface_name_;
  std::string robot_base_link_;
  std::string end_effector_link_;

  // Hardware interfaces
  std::vector<std::string> command_interface_types_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> joint_command_interfaces_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> joint_state_interfaces_;

  // Publishers
  using GuidePointPublisher = realtime_tools::RealtimePublisher<geometry_msgs::msg::PointStamped>;
  std::shared_ptr<GuidePointPublisher> guide_point_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr guide_point_pub_;

  // Constants for unit conversion
  const double MM_2_COUNT_X = 715.0;  // Horizontal
  const double COUNT_2_MM_X = 0.0014;
  const double MM_2_COUNT_Y = -2000.0/1.27;  // Depth
  const double COUNT_2_MM_Y = -0.0005*1.27;
  const double MM_2_COUNT_Z = 1430.0;  // Vertical
  const double COUNT_2_MM_Z = 0.0007;
};

} // namespace smart_template_cpp

#endif // SMART_TEMPLATE_CPP__SMART_TEMPLATE_CONTROLLER_HPP_