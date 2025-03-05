cmake_minimum_required(VERSION 3.8)
project(smart_template_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(controller_manager REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(ros2_igtl_bridge REQUIRED)
find_package(galil_driver REQUIRED)
find_package(realtime_tools REQUIRED)

# Find message generation packages
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(action_msgs REQUIRED)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "interfaces/srv/Command.srv"
  "interfaces/srv/GetPoint.srv"
  "interfaces/srv/GetPose.srv"
  "interfaces/srv/Move.srv"
  "interfaces/action/MoveAndObserve.action"
  DEPENDENCIES builtin_interfaces action_msgs
)

include_directories(
  include
)

# Create Smart Template node
add_executable(smart_template_node 
  src/smart_template_node.cpp
)

rosidl_get_typesupport_target(cpp_typesupport_target_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(smart_template_node "${cpp_typesupport_target_node}")

ament_target_dependencies(smart_template_node
  rclcpp
  rclcpp_action
  geometry_msgs
  sensor_msgs
  std_msgs
  ros2_igtl_bridge
)

target_link_libraries(smart_template_node
  gclib
)

# Add controller
add_library(smart_template_controller SHARED
  src/smart_template_controller.cpp
)

target_include_directories(smart_template_controller PRIVATE
  include
)

ament_target_dependencies(smart_template_controller
  rclcpp
  rclcpp_components
  controller_interface
  hardware_interface
  pluginlib
  geometry_msgs
  sensor_msgs
  realtime_tools
)

# Export controller as plugin
pluginlib_export_plugin_description_file(controller_interface smart_template_controller_plugin.xml)

# Install Python GUI module
ament_python_install_package(src)

# For Python components
if(PYTHON_INSTALL_DIR)
  # Already set
else()
  set(PYTHON_INSTALL_DIR lib/python3/dist-packages)
endif()

# Install
install(TARGETS
  smart_template_node
  smart_template_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install the GUI launcher script with executable permissions
install(PROGRAMS
  gui/launch_gui.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  launch
  config
  description
  DESTINATION share/${PROJECT_NAME}
)

# Install plugin.xml
install(FILES
  plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  scripts/run_gui.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(smart_template_controller)
ament_export_dependencies(
  controller_interface
  hardware_interface
  rclcpp
  pluginlib
  rosidl_default_runtime
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(smart_template_controller "${cpp_typesupport_target}")

ament_package()