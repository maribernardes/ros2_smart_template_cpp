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


# Find smart_template packages
find_package(smart_template_description REQUIRED) # URDF
find_package(smart_template_interfaces REQUIRED)  # Messages
find_package(galil_driver REQUIRED)               # Controller

include_directories(
  include
)

# Create Smart Template node
add_executable(smart_template_node 
  src/smart_template_node.cpp
)

#rosidl_get_typesupport_target(cpp_typesupport_target_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
#target_link_libraries(smart_template_node "${cpp_typesupport_target_node}")

ament_target_dependencies(smart_template_node
  rclcpp
  rclcpp_action
  geometry_msgs
  sensor_msgs
  std_msgs
  smart_template_interfaces
  smart_template_description
)

target_link_libraries(smart_template_node
  gclib
  tinyxml2
)

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
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install Python executables with clean names
install(PROGRAMS
  src/virtual_template.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME virtual_template
)

install(PROGRAMS
  src/world_pose_listener.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME world_pose_listener
)

install(PROGRAMS
  src/smart_template_gui.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME smart_template_gui
)


install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

# Install plugin.xml
install(FILES
  plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)

ament_package()