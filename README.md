# SmartTemplate ROS 2

ROS 2 packages for the SmartTemplate, a three-degree-of-freedom Cartesian needle-guide robot. The repository contains the robot description, custom service and action interfaces, a C++ control node with an RQt GUI, and MoveIt 2 configuration. It supports both mock hardware and the Galil hardware interface.

The current `main` branch is intended for ROS 2 Jazzy.

## Repository contents

| Package | Purpose |
| --- | --- |
| `smart_template_cpp` | C++ control node, RQt GUI, launch file, controller configuration, and utility nodes |
| `smart_template_description` | Xacro/URDF model, meshes, Z-frame configurations, and RViz configuration |
| `smart_template_interfaces` | `Command` and `Move` services and the `MoveAndObserve` action |
| `smart_template_moveit_config` | MoveIt 2 planning, kinematics, controller, and RViz configuration |

## Requirements

- Ubuntu 24.04 with ROS 2 Jazzy
- ROS 2 Control and ROS 2 Controllers
- MoveIt 2
- Eigen3 and TinyXML2
- Python 3, RQt, and Qt bindings for the GUI
- Galil `gclib` (the C++ executable links to this library in both mock and hardware builds)
- `galil_driver` when `use_mock_hardware:=false`

Install the ROS packages available through APT:

```bash
sudo apt update
sudo apt install \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-moveit \
  ros-jazzy-rqt-gui \
  ros-jazzy-rqt-gui-py
```

Install `gclib` separately and ensure that the linker can find it before building. Real-hardware operation also requires a `galil_driver` installation that exports `galil_driver/GalilSystemHardwareInterface`.

## Build

```bash
source /opt/ros/jazzy/setup.bash

mkdir -p ~/smart_template_ws/src
cd ~/smart_template_ws/src
git clone https://github.com/maribernardes/ros2_smart_template_cpp.git

cd ~/smart_template_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Source `/opt/ros/jazzy/setup.bash` and the workspace `install/setup.bash` in every new terminal.

## Launch the robot

The default launch uses mock hardware, starts the RQt GUI, and does not start RViz:

```bash
ros2 launch smart_template_cpp robot.launch.py
```

Common examples:

```bash
# Mock hardware with the standard URDF RViz display and no GUI
ros2 launch smart_template_cpp robot.launch.py \
  use_mock_hardware:=true gui:=false launch_rviz:=true

# SmartNeedle model
ros2 launch smart_template_cpp robot.launch.py needle_type:=smartneedle

# Real Galil hardware
ros2 launch smart_template_cpp robot.launch.py use_mock_hardware:=false

# Calibration mode (joint limits are intentionally expanded)
ros2 launch smart_template_cpp robot.launch.py robot_mode:=calibration
```

### Launch arguments

| Argument | Default | Accepted values or purpose |
| --- | --- | --- |
| `robot_mode` | `default` | `default`, `calibration` |
| `needle_type` | `default` | `default`, `stylet`, `smartneedle`, `biopsygun` |
| `zframe_config` | `default` | `default`, `test`, `new`, `old` |
| `use_mock_hardware` | `true` | `true` for `mock_components/GenericSystem`; `false` for the Galil interface |
| `launch_rviz` | `false` | Start the standard robot-description RViz configuration |
| `gui` | `true` | Start the SmartTemplate RQt plugin |
| `description_package` | `smart_template_description` | Package containing the robot description |
| `description_file` | `smart_template.urdf.xacro` | Xacro file in the description package |
| `name` | `smart_template` | ROS 2 Control system name passed to Xacro |
| `controller_spawner_timeout` | `10` | Controller-manager timeout in seconds |

The launch file starts:

- `robot_state_publisher`
- a static `world` to `base_link` transform
- `world_pose_listener`, which accepts updated transforms on `/world_pose`
- `ros2_control_node`
- `smart_template_node`
- `joint_state_broadcaster`
- `joint_trajectory_controller`
- the SmartTemplate RQt GUI and standard RViz display when enabled

## MoveIt 2

Start the robot/control stack first, then launch MoveIt in a second sourced terminal. Use matching robot, needle, Z-frame, and hardware arguments in both commands.

```bash
# Terminal 1
ros2 launch smart_template_cpp robot.launch.py gui:=false

# Terminal 2
ros2 launch smart_template_moveit_config moveit.launch.py
```

`moveit.launch.py` starts `move_group` and, by default, the MoveIt RViz configuration. Set `launch_rviz:=false` to run without MoveIt RViz.

## ROS interfaces

Positions supplied through the SmartTemplate-specific command interfaces are expressed in millimetres. Standard ROS joint-state and controller command values are expressed in metres.

### Subscribed topics

| Topic | Type | Units/purpose |
| --- | --- | --- |
| `/joint_states` | `sensor_msgs/msg/JointState` | Joint positions in metres |
| `/desired_position` | `geometry_msgs/msg/Point` | Desired Cartesian position in millimetres |
| `/desired_command` | `std_msgs/msg/String` | `HOME`, `RETRACT`, `ABORT`, or `RESUME` |
| `/world_pose` | `geometry_msgs/msg/TransformStamped` | Updated `world` to `base_link` transform |

### Published topics

| Topic | Type | Units/purpose |
| --- | --- | --- |
| `/position_controller/commands` | `std_msgs/msg/Float64MultiArray` | Joint position commands in metres |

### Services

| Service | Type | Purpose |
| --- | --- | --- |
| `/stage/command` | `smart_template_interfaces/srv/Command` | Send `HOME`, `RETRACT`, `ABORT`, or `RESUME` |
| `/stage/move` | `smart_template_interfaces/srv/Move` | Send an `(x, y, z)` target and tolerance `eps`, in millimetres |

Example service calls:

```bash
ros2 service call /stage/command \
  smart_template_interfaces/srv/Command "{command: HOME}"

ros2 service call /stage/move \
  smart_template_interfaces/srv/Move "{x: 0.0, y: 20.0, z: 0.0, eps: 0.3}"
```

### Action

| Action | Type | Purpose |
| --- | --- | --- |
| `/stage/move_and_observe` | `smart_template_interfaces/action/MoveAndObserve` | Move to an `(x, y, z)` target while reporting position, error, and elapsed time |

Example action goal:

```bash
ros2 action send_goal --feedback \
  /stage/move_and_observe \
  smart_template_interfaces/action/MoveAndObserve \
  "{x: 0.0, y: 20.0, z: 0.0, eps: 0.3}"
```

## GUI

The RQt plugin displays current and desired joint positions in millimetres and provides:

- direct joint-position entry;
- incremental horizontal, vertical, and insertion motion;
- `HOME` and `RETRACT` commands; and
- `ABORT` and `RESUME` controls.

If the GUI was disabled at launch, start it after the robot nodes are available:

```bash
rqt --standalone smart_template_gui --force-discover
```

## Current limitations

- `robot.launch.py` activates `joint_trajectory_controller`, whereas `smart_template_node` currently publishes direct commands to `/position_controller/commands`. These must be aligned before the node's topic-, service-, action-, and GUI-generated motion commands can drive the controller launched by default.
- The installed `virtual_template` utility imports a `GetPoint` service that is not present in `smart_template_interfaces`; it cannot currently be started without restoring that interface or removing the dependency.
- `ABORT` sets the node's internal abort state, but the hardware-level Galil stop command is currently a placeholder in `smart_template_node.cpp`.
- The MoveIt launch file starts the planning components only; it does not start the ROS 2 Control stack.

## Safety

This software controls a needle-guide robot and is research software. Validate joint limits, coordinate frames, controller selection, and emergency-stop behaviour with mock hardware before connecting physical hardware. Do not rely on the current software `ABORT` command as a hardware emergency stop.
