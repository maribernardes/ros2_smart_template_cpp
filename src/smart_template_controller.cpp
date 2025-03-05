#include "smart_template_cpp/smart_template_controller.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace smart_template_cpp
{

SmartTemplateController::SmartTemplateController()
: controller_interface::ControllerInterface()
{
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
SmartTemplateController::on_init()
{
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
SmartTemplateController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  
  for (const auto & joint_name : joint_names_) {
    config.names.push_back(joint_name + "/" + interface_name_);
  }
  
  return config;
}

controller_interface::InterfaceConfiguration
SmartTemplateController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  
  for (const auto & joint_name : joint_names_) {
    config.names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
    config.names.push_back(joint_name + "/" + hardware_interface::HW_IF_VELOCITY);
  }
  
  return config;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
SmartTemplateController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto node = get_node();
  
  // Get parameters
  std::vector<std::string> empty_vector;
  joint_names_ = node->declare_parameter<std::vector<std::string>>("joints", empty_vector);
  if (joint_names_.empty()) {
    RCLCPP_ERROR(node->get_logger(), "No joints specified.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }
  
  interface_name_ = node->declare_parameter<std::string>("interface_name", std::string(hardware_interface::HW_IF_VELOCITY));
  robot_base_link_ = node->declare_parameter<std::string>("robot_base_link", std::string("world"));
  end_effector_link_ = node->declare_parameter<std::string>("end_effector_link", std::string("needle_link"));
  
  // Create publishers
  guide_point_pub_ = node->create_publisher<geometry_msgs::msg::PointStamped>(
    "/stage/state/guide_pose", 10);
  guide_point_publisher_ = std::make_shared<GuidePointPublisher>(guide_point_pub_);
  
  RCLCPP_INFO(node->get_logger(), "Smart Template controller initialized with %zu joints", joint_names_.size());
  
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
SmartTemplateController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  joint_command_interfaces_.clear();
  joint_state_interfaces_.clear();
  
  // Get command interfaces
  for (const auto & joint_name : joint_names_) {
    const auto command_interface_name = joint_name + "/" + interface_name_;
    
    command_interface_types_.push_back(interface_name_);
    
    for (auto & command_interface : command_interfaces_) {
      if (command_interface.get_name() == command_interface_name) {
        joint_command_interfaces_.push_back(std::ref(command_interface));
        break;
      }
    }
  }
  
  // Get state interfaces
  for (const auto & joint_name : joint_names_) {
    for (auto & state_interface : state_interfaces_) {
      if (state_interface.get_name() == joint_name + "/" + hardware_interface::HW_IF_POSITION ||
          state_interface.get_name() == joint_name + "/" + hardware_interface::HW_IF_VELOCITY) {
        joint_state_interfaces_.push_back(std::ref(state_interface));
      }
    }
  }
  
  if (joint_command_interfaces_.size() != joint_names_.size()) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Expected %zu command interfaces, got %zu",
      joint_names_.size(), joint_command_interfaces_.size());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }
  
  if (joint_state_interfaces_.size() != joint_names_.size() * 2) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Expected %zu state interfaces, got %zu",
      joint_names_.size() * 2, joint_state_interfaces_.size());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }
  
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
SmartTemplateController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  joint_command_interfaces_.clear();
  joint_state_interfaces_.clear();
  
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

controller_interface::return_type
SmartTemplateController::update(const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  // Publish guide point (current position)
  if (guide_point_publisher_ && guide_point_publisher_->trylock()) {
    auto & msg = guide_point_publisher_->msg_;
    msg.header.stamp = time;
    msg.header.frame_id = "stage";
    
    // Get position from the state interfaces (horizontal, insertion, vertical)
    // In state_interfaces, the order is [horizontal_position, horizontal_velocity, insertion_position, ...]
    msg.point.x = joint_state_interfaces_[0].get().get_value() * 1000.0; // Convert m to mm
    msg.point.y = joint_state_interfaces_[2].get().get_value() * 1000.0;
    msg.point.z = joint_state_interfaces_[4].get().get_value() * 1000.0;
    
    guide_point_publisher_->unlockAndPublish();
  }
  
  return controller_interface::return_type::OK;
}

} // namespace smart_template_cpp

// Register controller to plugin
PLUGINLIB_EXPORT_CLASS(
  smart_template_cpp::SmartTemplateController,
  controller_interface::ControllerInterface)