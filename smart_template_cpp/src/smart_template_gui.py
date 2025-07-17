#!/usr/bin/env python3

import uuid  # Import uuid to generate unique  
import copy

# Import URDF description
import xml.etree.ElementTree as ET
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.msg import ParameterType

# Import ROS 2 libraries
import rclpy
from rclpy.action import ActionClient

# Import rqt Plugin base class
from rqt_gui_py.plugin import Plugin

# Import Qt libraries
from python_qt_binding.QtWidgets import (QWidget, QLabel, QSlider, QLineEdit, QPushButton, QVBoxLayout, QHBoxLayout, QSpacerItem, QFrame, QGridLayout, QTextEdit)
from python_qt_binding.QtCore import Qt, Signal, QTimer

# Import ROS 2 message and service types
from action_msgs.msg import GoalStatus
from sensor_msgs.msg import JointState
from smart_template_interfaces.action import MoveAndObserve
from smart_template_interfaces.srv import Command

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
        except:
            self.node.get_logger().error("Failed to retrieve robot_description.")

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

        # ROS Action client
        self.action_client = ActionClient(self.node, MoveAndObserve, '/stage/move_and_observe')
        self.node.get_logger().info('Waiting for action server /stage/move_and_observe...')
        self.action_client.wait_for_server()

        # ROS Service client
        self.service_client = self.node.create_client(Command, '/stage/command')
        self.node.get_logger().info('Waiting for service /stage/command...')
        self.service_client.wait_for_service()

        # Setup UI elements
        self.setup_ui()

        self.node.get_logger().info('Successfully connected to SmartTemplate')

    def get_robot_description(self):
        # Create a client for the 'get_parameters' service
        param_client = self.node.create_client(GetParameters, '/robot_state_publisher/get_parameters')

        # Wait for the service to become available
        if not param_client.wait_for_service(timeout_sec=5.0):
            self.node.get_logger().error("Service /robot_state_publisher/get_parameters not available.")
            return None
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
            return None
        if len(response.values) == 0 or response.values[0].type != ParameterType.PARAMETER_STRING:
            self.node.get_logger().error("robot_description parameter is empty or not a string!")
            return None
        # Extract the URDF string
        urdf_string = response.values[0].string_value
        self.node.get_logger().info(f"Robot Description retrieved: {len(urdf_string)} characters")
        return urdf_string

    def extract_robot_joints(self, urdf_string):
        # Parse URDF
        root = ET.fromstring(urdf_string)
        raw_channels = {}
        raw_limits = {}
        for joint_elem in root.findall('joint'):
            name = joint_elem.get('name')
            channel_elem = joint_elem.find('channel')
            if channel_elem is not None:
                channel = channel_elem.text.strip()
                raw_channels[name] = channel
            limit = joint_elem.find('limit')
            if limit is not None:
                lower = 1000*float(limit.get('lower', 'nan'))
                upper = 1000*float(limit.get('upper', 'nan'))
                raw_limits[name] = {'min': lower, 'max': upper}
        # Sort by channel order (A, B, C, ...)
        sorted_joint_items = sorted(raw_channels.items(), key=lambda item: item[1])
        joint_names = [joint for joint, _ in sorted_joint_items]
        joint_limits = {
            joint: raw_limits[joint]
            for joint in joint_names if joint in raw_limits
        }
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

            # Layout for each jointBefore
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
        retract_button.clicked.connect(lambda: self.send_service_request('RETRACT'))
        home_button.clicked.connect(lambda: self.send_service_request('HOME'))

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

        # ABORT and RESUME buttons below debugger
        abort_resume_layout = QHBoxLayout()
        #abort_resume_layout.setAlignment(Qt.AlignCenter)
        abort_button = QPushButton('ABORT')
        resume_button = QPushButton('RESUME')

        # Connect the buttons to respective service requests
        abort_button.clicked.connect(lambda: self.send_service_request('ABORT'))
        resume_button.clicked.connect(lambda: self.send_service_request('RESUME'))

        abort_resume_layout.addWidget(abort_button)
        abort_resume_layout.addWidget(resume_button)
        main_layout.addLayout(abort_resume_layout)

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
        # Send action request
        self.send_action_request(self.desired_joint_values)

    # Handle incremental step buttons
    def handle_step_motion_button(self, direction):
        try:
            step_size = 0
            joint_name = None
            if direction in ['UP', 'DOWN']:
                step_size = float(self.up_down_step_size.text())
                joint_name = 'vertical_joint'
                step_modifier = 1 if direction == 'UP' else -1
            elif direction in ['LEFT', 'RIGHT']:
                step_size = float(self.left_right_step_size.text())
                joint_name = 'horizontal_joint'
                step_modifier = 1 if direction == 'RIGHT' else -1
            elif direction in ['+', '-']:
                step_size = float(self.insertion_step_size.text())
                joint_name = 'insertion_joint'
                step_modifier = 1 if direction == '+' else -1
            self.desired_joint_values = copy.deepcopy(self.current_joint_values) # Put desired values equal to current joints
            formatted = {k: f"{v:.2f}" for k, v in self.current_joint_values.items()}
            self.node.get_logger().info(f"Current = {formatted}")
            if joint_name is not None:                                         # Increment desired step value in the selected joint
                self.desired_joint_values[joint_name] = self.current_joint_values[joint_name] + step_modifier * step_size
                formatted = {k: f"{v:.2f}" for k, v in self.desired_joint_values.items()}
                self.node.get_logger().info(f"Desired = {formatted}")
                self.send_action_request(self.desired_joint_values)
        except ValueError:
            self.node.get_logger().warn('Invalid step size value')

    # Send command request to smart_template robot 
    def send_service_request(self, cmd_string):
        self.robot_idle = False
        request = Command.Request()
        request.command = cmd_string
        future = self.service_client.call_async(request)
        future.add_done_callback(partial(self.get_response_callback))

    def get_response_callback(self, future):
        try:
            response = future.result()
            self.node.get_logger().info('Service request: %s' %(response.response)) 
        except Exception as e:
            self.node.get_logger().error('Service call failed: %r' %(e,))
        self.robot_idle = True

    # Send action request to smart_template robot
    def send_action_request(self, desired_joint_values):
        self.robot_idle = False
        corrected_values = {}
        # Validate and correct desired_joint_values against joint_limits
        for joint, value in desired_joint_values.items():
            limits = self.joint_limits.get(joint, {})
            self.node.get_logger().debug(f"{joint}: value={value}, min={limits['min']}, value < min: {value < limits['min']}")
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
        # Send corrected values
        goal_msg = MoveAndObserve.Goal()
        goal_msg.x = corrected_values.get('horizontal_joint', 0.0)
        goal_msg.y = corrected_values.get('insertion_joint', 0.0)
        goal_msg.z = corrected_values.get('vertical_joint', 0.0)
        goal_msg.eps = 0.25  # Set appropriate epsilon value
        self.node.get_logger().info(f'Sending goal: x={goal_msg.x} mm, y={goal_msg.y} mm, z={goal_msg.z} mm')
        # Send goal asynchronously
        self.send_goal_future = self.action_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.node.get_logger().info('Goal rejected :(')
                return
            self._get_result_future = goal_handle.get_result_async()
            self._get_result_future.add_done_callback(self.get_result_callback)
        except Exception as e:
            self.node.get_logger().error(f'Error in goal_response_callback: {e}')

    def get_result_callback(self, future):
        try:
            result = future.result().result
            status = future.result().status
            if status == GoalStatus.STATUS_SUCCEEDED:
                self.node.get_logger().info(f"Goal succeeded: position=({result.x:.2f}, {result.y:.2f}, {result.z:.2f}), error={result.error:.4f}, time={result.time:.2f}s")
            else:
                self.node.get_logger().warn(f"Goal failed with status {status}, error code: {result.error_code}")
            self.robot_idle = True  # Set robot status to IDLE
        except Exception as e:
            self.node.get_logger().error(f'Error in get_result_callback: {e}')

    def feedback_callback(self, feedback_msg):
        # Handle feedback if needed
        feedback = feedback_msg.feedback
        # For example, update GUI elements or log feedback
        pass

    def shutdown_plugin(self):
        # Shutdown QTimer
        self.timer.stop()
        # Shutdown ROS node
        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()