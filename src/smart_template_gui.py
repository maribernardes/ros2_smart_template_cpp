#!/usr/bin/env python3

import threading
import uuid  # Import uuid to generate unique  

# Import URDF description
import xml.etree.ElementTree as ET
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.msg import ParameterType

# Import ROS 2 libraries
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock

# Import rqt Plugin base class
from rqt_gui_py.plugin import Plugin

# Import Qt libraries
from python_qt_binding.QtGui import QColor
from python_qt_binding.QtWidgets import (
    QWidget, QLabel, QSlider, QLineEdit, QPushButton, QVBoxLayout, QHBoxLayout, QSpacerItem, QFrame, QGridLayout, QTextEdit)
from python_qt_binding.QtCore import Qt, Signal, Slot, QTimer

# Import ROS 2 message types
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from functools import partial

class SmartTemplateGUIPlugin(Plugin):
    # Define a signal that carries joint names and positions
    update_joint_state_signal = Signal(dict)

    def __init__(self, context):
        super(SmartTemplateGUIPlugin, self).__init__(context)
        self.setObjectName('SmartTemplateGUIPlugin')

        # Create QWidget
        self._widget = QWidget()
        self._widget.setWindowTitle('SmartTemplate GUI')
        context.add_widget(self._widget)

        # Initialize ROS 2 node if not already initialized
        if not rclpy.ok():
            rclpy.init(args=None)
        unique_id = uuid.uuid4().hex[:8]
        self.node = rclpy.create_node(f'smart_template_gui_{unique_id}')
        
        # Fetch and parse robot description
        urdf_string = self.get_robot_description()
        try:
            self.node.get_logger().info(f"Successfully loaded robot_description: {len(urdf_string)} characters")
            self.joint_names, self.joint_limits = self.extract_robot_joints(urdf_string)
        except Exception as e:
            self.node.get_logger().error(f"Failed to retrieve robot_description: {e}")
            self.joint_names = ['horizontal_joint', 'insertion_joint', 'vertical_joint']
            self.joint_limits = {
                'horizontal_joint': {'min': -30.0, 'max': 30.0},
                'insertion_joint': {'min': 0.0, 'max': 100.0},
                'vertical_joint': {'min': 0.0, 'max': 50.0}
            }

        self.current_joint_values = {name: 0.0 for name in self.joint_names}
        self.desired_joint_values = {name: 0.0 for name in self.joint_names}

        # Robot status
        self.robot_idle = True

        # Start a QTimer to spin the node
        self.timer = QTimer()
        self.timer.timeout.connect(self.spin_once)
        self.timer.start(10)  # Call spin_once every 10 milliseconds

        # ROS subscribers
        self.joint_state_subscriber = self.node.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)
            
        # ROS publishers for position controller
        self.position_publisher = self.node.create_publisher(
            Float64MultiArray, '/position_controller/commands', 10)
            
        # Using ROS 2 Control position controller
        self.node.get_logger().info('Using ROS2 Control position_controller for smart_template')

        # Setup UI elements
        self.setup_ui()

        self.node.get_logger().info('Successfully connected to SmartTemplate')

    def get_robot_description(self):
        # Create a client for the 'get_parameters' service
        param_client = self.node.create_client(GetParameters, '/robot_state_publisher/get_parameters')

        # Wait for the service to become available
        if not param_client.wait_for_service(timeout_sec=5.0):
            self.node.get_logger().error("Service /robot_state_publisher/get_parameters not available.")
            return ""
        # Create a request to get the 'robot_description' parameter
        request = GetParameters.Request()
        request.names = ['robot_description']
        # Call the service
        future = param_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        # Handle the response
        response = future.result()
        if response is None:
            self.node.get_logger().error("Failed to fetch robot_description parameter.")
            return ""
        if len(response.values) == 0 or response.values[0].type != ParameterType.PARAMETER_STRING:
            self.node.get_logger().error("robot_description parameter is empty or not a string!")
            return ""
        # Extract the URDF string
        urdf_string = response.values[0].string_value
        self.node.get_logger().info(f"Robot Description retrieved: {len(urdf_string)} characters")
        return urdf_string

    def extract_robot_joints(self, urdf_string):
        # Parse URDF
        root = ET.fromstring(urdf_string)
        joint_names = []
        joint_limits = {}

        for joint in root.findall('joint'):
            joint_name = joint.get('name')
            limit = joint.find('limit')
            if limit is not None:
                lower = 1000*float(limit.get('lower', '0.0'))
                upper = 1000*float(limit.get('upper', '0.0'))
                joint_names.append(joint_name)
                joint_limits[joint_name] = {'min': lower, 'max': upper}

        self.node.get_logger().info(f"Joint Names: {joint_names}")
        self.node.get_logger().info(f"Joint Limits: {joint_limits}")
        return joint_names, joint_limits

    def spin_once(self):
        rclpy.spin_once(self.node, timeout_sec=0)

    def setup_ui(self):
        # Main layout to hold left and right panels
        panel_layout = QHBoxLayout()

        # Left panel: Joint controls
        joint_controls_layout = QVBoxLayout()

        # Dictionaries to hold widgets
        self.sliders = {}
        self.text_boxes = {}
        self.current_value_boxes = {}  # For 'Current [mm]' textboxes

        for joint in self.joint_names:
            # Remove '_joint' suffix and capitalize the label
            joint_label = QLabel(f'{joint.replace("_joint", "").capitalize()}')

            # Slider for current joint state
            slider = QSlider(Qt.Horizontal)
            limits = self.joint_limits[joint]
            slider.setMinimum(int(limits['min']))
            slider.setMaximum(int(limits['max']))
            slider.setEnabled(False)  # Non-editable
            slider.setValue(0)

            # Min and Max labels
            min_label = QLabel(f"{limits['min']}")
            max_label = QLabel(f"{limits['max']}")

            # Layout for slider and labels
            slider_layout = QHBoxLayout()
            slider_layout.addWidget(min_label)
            slider_layout.addWidget(slider)
            slider_layout.addWidget(max_label)

            # Text box for current joint value (non-editable)
            current_value_box = QLineEdit('0.0')
            current_value_box.setReadOnly(True)
            current_value_box.setStyleSheet("background: transparent; border: none; color: black;")
            current_value_label = QLabel('Current [mm]:')

            # Text box for desired joint value
            desired_value_box = QLineEdit('0.0')
            desired_value_label = QLabel('Desired [mm]:')

            # Layout for current and desired values
            value_layout = QHBoxLayout()
            value_layout.addWidget(current_value_label)
            value_layout.addWidget(current_value_box)
            value_layout.addWidget(desired_value_label)
            value_layout.addWidget(desired_value_box)

            # Layout for each joint
            joint_layout = QVBoxLayout()
            joint_layout.addWidget(joint_label)
            joint_layout.addLayout(slider_layout)
            joint_layout.addLayout(value_layout)

            # Add the joint layout to the main joint controls layout
            joint_controls_layout.addLayout(joint_layout)

            # Add a horizontal separator line after each joint section
            separator_line = QFrame()
            separator_line.setFrameShape(QFrame.HLine)
            separator_line.setFrameShadow(QFrame.Sunken)
            joint_controls_layout.addWidget(separator_line)

            # Save references
            self.sliders[joint] = slider
            self.text_boxes[joint] = desired_value_box
            self.current_value_boxes[joint] = current_value_box

        # Add a "Send" button at the bottom of the left panel
        send_button = QPushButton('Send')
        send_button.clicked.connect(self.handle_send_desired_joints_button)  # Connect button to the function
        joint_controls_layout.addWidget(send_button)

        # Add joint controls to main layout
        panel_layout.addLayout(joint_controls_layout)

        # Add spacer between left and right panels
        panel_layout.addSpacerItem(QSpacerItem(15, 0))  # Horizontal spacer to increase space

        # Vertical line separator
        line = QFrame()
        line.setFrameShape(QFrame.VLine)
        line.setFrameShadow(QFrame.Sunken)
        panel_layout.addWidget(line)
        panel_layout.addSpacerItem(QSpacerItem(15, 0))  # Horizontal spacer to increase space

        # Right panel: Button controls
        button_controls_layout = QVBoxLayout()

        # Top row: RETRACT and HOME buttons
        retract_home_layout = QHBoxLayout()
        retract_button = QPushButton('RETRACT')
        home_button = QPushButton('HOME')

        # Connect buttons to the service request function
        retract_button.clicked.connect(lambda: self.send_command('RETRACT'))
        home_button.clicked.connect(lambda: self.send_command('HOME'))

        retract_home_layout.addWidget(retract_button)
        retract_home_layout.addWidget(home_button)
        button_controls_layout.addLayout(retract_home_layout)

        # Middle row: Cross-patterned arrow buttons
        cross_pattern_layout = QVBoxLayout()

        # Create directional buttons
        up_button = QPushButton('↑')
        left_button = QPushButton('←')
        right_button = QPushButton('→')
        down_button = QPushButton('↓')

        # Create step size text boxes
        self.up_down_step_size = QLineEdit('1.0')  # Text box for up/down step size
        self.left_right_step_size = QLineEdit('1.0')  # Text box for left/right step size
        self.up_down_step_size.setFixedWidth(50)
        self.left_right_step_size.setFixedWidth(50)

        # Set fixed size for buttons
        button_size = 40
        up_button.setFixedSize(button_size, button_size)
        left_button.setFixedSize(button_size, button_size)
        right_button.setFixedSize(button_size, button_size)
        down_button.setFixedSize(button_size, button_size)

        # Connect buttons to the unified motion handling function
        up_button.clicked.connect(lambda: self.handle_step_motion_button('UP'))
        left_button.clicked.connect(lambda: self.handle_step_motion_button('LEFT'))
        right_button.clicked.connect(lambda: self.handle_step_motion_button('RIGHT'))
        down_button.clicked.connect(lambda: self.handle_step_motion_button('DOWN'))

        # Layout for cross pattern with empty center
        cross_layout = QGridLayout()
        cross_layout.setAlignment(Qt.AlignCenter)  # Center the grid

        # Position elements in the grid
        cross_layout.addWidget(up_button, 0, 1, alignment=Qt.AlignCenter)  # Up button
        cross_layout.addWidget(left_button, 1, 0, alignment=Qt.AlignCenter)  # Left button
        cross_layout.addWidget(right_button, 1, 2, alignment=Qt.AlignCenter)  # Right button
        cross_layout.addWidget(down_button, 2, 1, alignment=Qt.AlignCenter)  # Down button

        # Add step size text boxes near corresponding buttons
        cross_layout.addWidget(self.up_down_step_size, 0, 3, alignment=Qt.AlignLeft)  # Step size for up/down
        cross_layout.addWidget(self.left_right_step_size, 1, 3, alignment=Qt.AlignLeft)  # Step size for left/right

        # Add cross layout to main button controls layout
        cross_pattern_layout.addLayout(cross_layout)
        button_controls_layout.addLayout(cross_pattern_layout)

        # Bottom row: + and - buttons
        plus_minus_layout = QHBoxLayout()
        plus_button = QPushButton('+')
        minus_button = QPushButton('-')
        self.insertion_step_size = QLineEdit('5.0')  # Text box for insertion step size
        self.insertion_step_size.setFixedWidth(50)

        # Connect buttons to the unified motion handling function
        plus_button.clicked.connect(lambda: self.handle_step_motion_button('+'))
        minus_button.clicked.connect(lambda: self.handle_step_motion_button('-'))

        plus_minus_layout.addWidget(plus_button)
        plus_minus_layout.addWidget(minus_button)
        plus_minus_layout.addStretch()  # Spacer to push the text box to the right
        plus_minus_layout.addWidget(self.insertion_step_size)

        button_controls_layout.addLayout(plus_minus_layout)

        # Add button controls to main layout
        panel_layout.addLayout(button_controls_layout)

        main_layout = QVBoxLayout()
        messageBox_label = QLabel('Debugger')
        self.messageBox = QTextEdit()
        self.messageBox.setReadOnly(True)
        
        main_layout.addLayout(panel_layout)
        main_layout.addWidget(messageBox_label)
        main_layout.addWidget(self.messageBox)

        # Set layout to the widget
        self._widget.setLayout(main_layout)

    # Get timestamp
    def get_ros_timestamp(self):
        ros_time = self.node.get_clock().now()  # Get current ROS time
        return f"[{ros_time.to_msg().sec}.{ros_time.to_msg().nanosec:09d}]"

    # Callback to /joint_states publisher messages
    def joint_state_callback(self, msg):
        try:
            for name, position in zip(msg.name, msg.position):
                if name in self.joint_names:
                    # Convert position from meters to millimeters
                    position_mm = position * 1000.0
                    self.current_joint_values[name] = position_mm
                    # Update slider value
                    slider = self.sliders[name]
                    slider_value = int(position_mm)
                    # Ensure slider_value is within slider range
                    slider_min = slider.minimum()
                    slider_max = slider.maximum()
                    slider_value = max(min(slider_value, slider_max), slider_min)
                    slider.setValue(slider_value)
                    # Update the 'Current [mm]' textbox
                    current_value_box = self.current_value_boxes[name]
                    current_value_box.setText(f'{position_mm:.2f}')
        except Exception as e:
            self.node.get_logger().error(f'Error in joint_state_callback: {e}')

    # Handle desired joints button
    def handle_send_desired_joints_button(self):
        for joint in self.joint_names:
            textbox = self.text_boxes[joint]
            try:
                # Directly parse the value from the text box
                value = float(textbox.text())
                self.desired_joint_values[joint] = value
            except ValueError:
                self.node.get_logger().warn(f'Invalid input for {joint}')
                self.messageBox.append(f'{self.get_ros_timestamp()} <span style="color: red;">Warning:</span> Invalid input for {joint}. Please enter a numeric value.')
                return  # Exit early if any value is invalid
        # Send position command
        self.send_joint_position_command(self.desired_joint_values)

    # Handle incremental step buttons
    def handle_step_motion_button(self, direction):
        try:
            step_size = 0
            joint_key = ''
            if direction in ['UP', 'DOWN']:
                step_size = float(self.up_down_step_size.text())
                joint_key = 'vertical_joint'
                step_modifier = 1 if direction == 'UP' else -1
            elif direction in ['LEFT', 'RIGHT']:
                step_size = float(self.left_right_step_size.text())
                joint_key = 'horizontal_joint'
                step_modifier = 1 if direction == 'RIGHT' else -1
            elif direction in ['+', '-']:
                step_size = float(self.insertion_step_size.text())
                joint_key = 'insertion_joint'
                step_modifier = 1 if direction == '+' else -1
            self.desired_joint_values = self.current_joint_values.copy()  # Make a copy of current joints
            if joint_key:                                         # Increment desired step value in the selected joint
                self.desired_joint_values[joint_key] = self.current_joint_values[joint_key] + step_modifier * step_size
                self.send_joint_position_command(self.desired_joint_values)
        except ValueError:
            self.node.get_logger().warn('Invalid step size value')

    # Send command request to smart_template robot (using ROS2 Control)
    def send_command(self, cmd_string):
        self.robot_idle = False
        
        # Handle different commands
        if cmd_string == 'HOME':
            # Send all joints to home position (0)
            self.node.get_logger().info('Sending HOME command')
            self.messageBox.append(f'{self.get_ros_timestamp()} Sending HOME command')
            
            # Send zeros to all joint positions
            home_positions = [0.0, 0.0, 0.0]  # horizontal, insertion, vertical
            self.send_position_command(home_positions)
            
        elif cmd_string == 'RETRACT':
            # Keep horizontal and vertical position, but retract insertion to 0
            current_values = self.current_joint_values
            self.node.get_logger().info('Sending RETRACT command')
            self.messageBox.append(f'{self.get_ros_timestamp()} Sending RETRACT command')
            
            # Create command with current horizontal/vertical but 0 insertion
            retract_positions = [
                current_values.get('horizontal_joint', 0.0),
                0.0,  # insertion_joint set to 0
                current_values.get('vertical_joint', 0.0)
            ]
            self.send_position_command(retract_positions)
        
        self.robot_idle = True

    # Send position command directly to position_controller
    def send_position_command(self, positions):
        msg = Float64MultiArray()
        
        # Convert mm to meters for ROS2 Control
        positions_meters = [p / 1000.0 for p in positions]
        
        msg.data = positions_meters
        self.position_publisher.publish(msg)
        self.node.get_logger().info(f'Published position command: {positions_meters}')

    # Send position command for specified joints
    def send_joint_position_command(self, desired_joint_values):
        self.robot_idle = False
        corrected_values = {}
        
        # Validate and correct desired_joint_values against joint_limits
        for joint, value in desired_joint_values.items():
            limits = self.joint_limits.get(joint, {})
            if 'min' in limits and value < limits['min']:
                corrected_values[joint] = limits['min']
                self.node.get_logger().warn(f'Desired value for {joint} below limit. Setting to minimum: {limits["min"]}')
                self.messageBox.append(f'{self.get_ros_timestamp()} <span style="color: red;">Warning:</span> Desired value for {joint} is below the limit. Setting to minimum: {limits["min"]} mm')
            elif 'max' in limits and value > limits['max']:
                corrected_values[joint] = limits['max']
                self.node.get_logger().warn(f'Desired value for {joint} above limit. Setting to maximum: {limits["max"]}')
                self.messageBox.append(f'{self.get_ros_timestamp()} <span style="color: red;">Warning:</span> Desired value for {joint} is above the limit. Setting to maximum: {limits["max"]} mm')
            else:
                corrected_values[joint] = value
                
        # Get values in the right order (must match controller's joint order)
        x = corrected_values.get('horizontal_joint', 0.0)
        y = corrected_values.get('insertion_joint', 0.0)
        z = corrected_values.get('vertical_joint', 0.0)
        
        # Send to position controller directly
        self.send_position_command([x, y, z])
        self.node.get_logger().info(f'Sending position: x={x} mm, y={y} mm, z={z} mm')
        self.messageBox.append(f'{self.get_ros_timestamp()} Sending position: x={x} mm, y={y} mm, z={z} mm')
        
        # In this ROS control approach, we don't get feedback about completion
        # So we'll just mark robot as idle immediately
        self.robot_idle = True

    def shutdown_plugin(self):
        # Shutdown QTimer
        self.timer.stop()
        # Shutdown ROS node
        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()