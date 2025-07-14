#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import Pose
from geometry_msgs.msg import TransformStamped

class WorldPoseListener(Node):
    def __init__(self):
        super().__init__('world_pose_listener')
        self.subscription = self.create_subscription(
            TransformStamped,
            '/world_pose',
            self.pose_callback,
            10
        )
        self.broadcaster = StaticTransformBroadcaster(self)

    def pose_callback(self, msg):
        self.get_logger().info(f"Received new world_pose: {msg}")

        # Add header to make static transform message
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.child_frame_id = 'base_link'

        # Broadcast the updated static transform
        self.broadcaster.sendTransform(msg)
        self.get_logger().info("Updated static transform world -> base_link published.")

def main():
    rclpy.init()
    node = WorldPoseListener()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
