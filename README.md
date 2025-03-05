# Smart Template C++ Package

This package provides a complete C++ implementation of the smart_template robot that integrates with the ROS 2 Control framework. It leverages the existing `galil_driver` hardware interface while containing all robot-specific details, descriptions, and interfaces within a single package.

## Package Structure

- `src/smart_template_node.cpp`: C++ implementation of the SmartTemplate node (equivalent to the Python version)
- `src/smart_template_controller.cpp`: ROS 2 Controller implementation for the Smart Template robot
- `include/smart_template_cpp/`: Header files for the C++ implementations
- `config/`: Controller configuration files
- `launch/`: Launch files for starting the system
- `description/`: URDF, meshes, and RViz configuration files
- `interfaces/`: Message, service, and action definitions

## Features

- Direct communication with the Galil hardware through the `galil_driver` hardware interface
- ROS 2 Control framework integration
- Service and action interfaces for controlling the robot:
  - `/stage/command`: Command service for predefined commands (HOME, RETRACT, ABORT)
  - `/stage/move`: Service for sending movement commands
  - `/stage/get_position`: Service for retrieving the current position
  - `/stage/move_and_observe`: Action server for movement with feedback

## Dependencies

- `galil_driver`: Hardware interface for the Galil controller 
- ROS 2 Control framework packages
- General ROS 2 dependencies (rclcpp, etc.)

The package is completely self-contained and includes:
- All URDF, mesh, and visualization files
- All message, service, and action definitions
- All required launch and configuration files

## Usage

To launch the Smart Template robot with ros2_control:

```bash
ros2 launch smart_template_cpp smart_template.launch.py
```

Available launch arguments:
- `name`: Robot name (default: 'smart_template')
- `needle_type`: Type of needle to use - options are 'default', 'biopsygun', 'smartneedle', or 'stylet' (default: 'default')
- `zframe_config`: Configuration for the zframe (default: 'default')
- `use_rviz`: Launch RViz visualization (default: 'true')
- `use_gui`: Launch the SmartTemplate GUI (default: 'false')
- `use_sim_time`: Use simulation clock (default: 'false')

To launch with GUI:
```bash
ros2 launch smart_template_cpp smart_template.launch.py use_gui:=true
```

Example with custom needle type:
```bash
ros2 launch smart_template_cpp smart_template.launch.py needle_type:=smartneedle
```

## GUI Usage

The SmartTemplate GUI provides a user-friendly interface to control the robot:

- **Joint Controls**: Set and monitor specific joint positions
- **Arrow Controls**: Move the robot incrementally in different directions
- **Command Buttons**: Send predefined commands (HOME, RETRACT)
- **Movement Controls**: Intuitive directional control buttons with configurable step sizes

This will start the required nodes:
- Controller Manager with the Smart Template controller
- Joint State Broadcaster
- Position Controller
- Smart Template Node (service and action interfaces)
- Robot State Publisher
- RViz (optional)

## Architecture

The Smart Template C++ implementation follows this architecture:

1. `galil_driver` provides the hardware interface implementation for the Galil controller
2. `smart_template_cpp` provides:
   - Robot-specific configuration
   - Controllers and high-level interfaces
   - Launch files for starting the system
3. `smart_template_description` provides the robot description (URDF)
4. `smart_template_interfaces` provides the message, service, and action definitions

This architecture allows for modifying the high-level control while reusing the hardware interface code in `galil_driver`.# ros2_smart_template_cpp
