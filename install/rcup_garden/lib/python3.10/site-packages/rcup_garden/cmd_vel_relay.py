#!/usr/bin/env python3
"""
Simple relay node to forward /cmd_vel to /diff_cont/cmd_vel
Allows teleop_twist_keyboard to work with the diff_cont controller without modification
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdVelRelay(Node):
    def __init__(self):
        super().__init__('cmd_vel_relay')
        
        # Subscribe to /cmd_vel (from teleop_twist_keyboard)
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Publish to /diff_cont/cmd_vel (what the diff_drive_controller expects by default)
        self.publisher = self.create_publisher(
            Twist,
            '/diff_cont/cmd_vel_unstamped',
            10
        )
        
        self.get_logger().info('cmd_vel_relay node started')
        self.get_logger().info('Relaying /cmd_vel -> /diff_cont/cmd_vel_unstamped')
    
    def cmd_vel_callback(self, msg):
        """Forward velocity commands from teleop to the controller"""
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    relay = CmdVelRelay()
    rclpy.spin(relay)
    relay.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
