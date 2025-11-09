#!/usr/bin/env python3
"""
Diagnostic script to trace cmd_vel message flow through the system
Helps identify where velocity commands are being dropped or not routed properly
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time


class CmdVelDiagnostic(Node):
    def __init__(self):
        super().__init__('cmd_vel_diagnostic')
        
        self.get_logger().info("=" * 80)
        self.get_logger().info("CMD_VEL FLOW DIAGNOSTIC")
        self.get_logger().info("=" * 80)
        
        # Subscribe to all possible cmd_vel topics
        self.subscribers = {}
        topics_to_monitor = [
            '/cmd_vel_nav',
            '/cmd_vel',
            '/diff_cont/cmd_vel',
            '/diff_cont/cmd_vel_unstamped',
            '/cmd_vel_teleop',
        ]
        
        for topic in topics_to_monitor:
            try:
                sub = self.create_subscription(
                    Twist,
                    topic,
                    lambda msg, t=topic: self.callback(msg, t),
                    10
                )
                self.subscribers[topic] = {'sub': sub, 'count': 0, 'last_time': time.time()}
                self.get_logger().info(f"✓ Subscribed to: {topic}")
            except Exception as e:
                self.get_logger().error(f"✗ Failed to subscribe to {topic}: {e}")
        
        # Check joint states
        self.joint_sub = self.create_subscription(
            rclpy.import_message_module('sensor_msgs.msg', 'JointState'),
            '/joint_states',
            self.joint_callback,
            10
        )
        self.get_logger().info("✓ Subscribed to: /joint_states")
        
        # Check odometry
        self.odom_sub = self.create_subscription(
            rclpy.import_message_module('nav_msgs.msg', 'Odometry'),
            '/diff_cont/odom',
            self.odom_callback,
            10
        )
        self.get_logger().info("✓ Subscribed to: /diff_cont/odom")
        
        self.wheel_velocity_log = {}
        self.odom_log = {'last_x': None, 'last_y': None}
        self.get_logger().info("=" * 80)
        self.get_logger().info("Waiting for messages... Set a goal in Nav2 to trigger motion")
        self.get_logger().info("=" * 80)
    
    def callback(self, msg, topic):
        """Track cmd_vel messages"""
        if topic not in self.subscribers:
            return
        
        self.subscribers[topic]['count'] += 1
        current_time = time.time()
        
        # Only log once per second to avoid spam
        if current_time - self.subscribers[topic]['last_time'] > 1.0:
            self.get_logger().info(
                f"[{topic}] #msgs: {self.subscribers[topic]['count']:4d} | "
                f"linear.x: {msg.linear.x:6.3f} m/s, angular.z: {msg.angular.z:6.3f} rad/s"
            )
            self.subscribers[topic]['last_time'] = current_time
    
    def joint_callback(self, msg):
        """Monitor wheel joint velocities"""
        try:
            # Look for wheel joint velocities
            for i, name in enumerate(msg.name):
                if 'wheel' in name.lower() or 'motor' in name.lower():
                    velocity = msg.velocity[i] if i < len(msg.velocity) else 0.0
                    
                    if name not in self.wheel_velocity_log:
                        self.wheel_velocity_log[name] = {'count': 0, 'last_time': time.time()}
                    
                    self.wheel_velocity_log[name]['count'] += 1
                    current_time = time.time()
                    
                    # Log once per second
                    if current_time - self.wheel_velocity_log[name]['last_time'] > 1.0:
                        status = "✓ MOVING" if abs(velocity) > 0.01 else "✗ STUCK"
                        self.get_logger().warn(
                            f"[JOINT] {name:40s} velocity: {velocity:8.4f} rad/s {status}"
                        )
                        self.wheel_velocity_log[name]['last_time'] = current_time
        except Exception as e:
            pass  # Silently skip if index out of range
    
    def odom_callback(self, msg):
        """Monitor odometry to see if robot is actually moving"""
        try:
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            
            if self.odom_log['last_x'] is None:
                self.odom_log['last_x'] = x
                self.odom_log['last_y'] = y
                return
            
            dx = x - self.odom_log['last_x']
            dy = y - self.odom_log['last_y']
            distance = (dx**2 + dy**2)**0.5
            
            # Log if significant movement
            if distance > 0.01:
                self.get_logger().info(
                    f"[ODOMETRY] Position: ({x:6.3f}, {y:6.3f}) | "
                    f"Delta: ({dx:6.3f}, {dy:6.3f}) | Distance: {distance:6.4f} m ✓ MOVING"
                )
            else:
                self.get_logger().warn(
                    f"[ODOMETRY] Position: ({x:6.3f}, {y:6.3f}) | "
                    f"No movement detected ✗ STUCK"
                )
            
            self.odom_log['last_x'] = x
            self.odom_log['last_y'] = y
        except Exception as e:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelDiagnostic()
    
    print("\n" + "=" * 80)
    print("DIAGNOSTIC RUNNING - Set a goal pose in RViz")
    print("Watch for messages below to trace where commands go")
    print("=" * 80 + "\n")
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n" + "=" * 80)
        print("DIAGNOSTIC COMPLETE")
        print("=" * 80)
        node.get_logger().info(f"Total messages received per topic:")
        for topic, data in node.subscribers.items():
            node.get_logger().info(f"  {topic}: {data['count']} messages")
        
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
