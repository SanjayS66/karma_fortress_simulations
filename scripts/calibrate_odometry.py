#!/usr/bin/env python3
"""
Odometry Calibration Script
This script helps you calibrate wheel_separation and wheel_radius multipliers
by comparing commanded motion with actual motion in Gazebo.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import time

class OdometryCalibrator(Node):
    def __init__(self):
        super().__init__('odometry_calibrator')
        
        # QoS profile for odometry - use RELIABLE to match controller publisher
        odom_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.cmd_pub = self.create_publisher(Twist, '/diff_cont/cmd_vel_unstamped', 10)
        self.odom_sub = self.create_subscription(Odometry, '/diff_cont/odom', self.odom_callback, odom_qos)
        self.gazebo_odom_sub = self.create_subscription(Odometry, '/odom', self.gazebo_odom_callback, odom_qos)
        
        self.controller_odom = None
        self.gazebo_odom = None
        
        self.get_logger().info("Odometry Calibrator Node Started")
        self.get_logger().info("This will help you calibrate wheel parameters")
        
    def odom_callback(self, msg):
        self.controller_odom = msg
        
    def gazebo_odom_callback(self, msg):
        self.gazebo_odom = msg
        
    def stop_robot(self):
        """Send zero velocity command"""
        twist = Twist()
        self.cmd_pub.publish(twist)
        time.sleep(0.5)
        
    def drive_forward(self, distance=1.0, speed=0.2):
        """Drive forward for a specific distance"""
        self.get_logger().info(f"Driving forward {distance}m at {speed}m/s...")
        
        # Record starting position
        self.stop_robot()
        time.sleep(1.0)
        
        if not self.controller_odom or not self.gazebo_odom:
            self.get_logger().error("No odometry data received!")
            return
            
        start_ctrl = self.controller_odom.pose.pose.position
        start_gaz = self.gazebo_odom.pose.pose.position
        
        # Drive forward
        twist = Twist()
        twist.linear.x = speed
        duration = distance / speed
        
        start_time = time.time()
        while (time.time() - start_time) < duration:
            self.cmd_pub.publish(twist)
            time.sleep(0.1)
            
        self.stop_robot()
        time.sleep(1.0)
        
        # Calculate distances
        end_ctrl = self.controller_odom.pose.pose.position
        end_gaz = self.gazebo_odom.pose.pose.position
        
        ctrl_dist = math.sqrt((end_ctrl.x - start_ctrl.x)**2 + (end_ctrl.y - start_ctrl.y)**2)
        gaz_dist = math.sqrt((end_gaz.x - start_gaz.x)**2 + (end_gaz.y - start_gaz.y)**2)
        
        self.get_logger().info(f"\n=== LINEAR MOTION TEST ===")
        self.get_logger().info(f"Controller odometry: {ctrl_dist:.3f}m")
        self.get_logger().info(f"Gazebo odometry:    {gaz_dist:.3f}m")
        self.get_logger().info(f"Difference:         {abs(ctrl_dist - gaz_dist):.3f}m")
        
        if abs(ctrl_dist - gaz_dist) > 0.01:
            ratio = gaz_dist / ctrl_dist if ctrl_dist > 0 else 1.0
            self.get_logger().warn(f"RADIUS MULTIPLIER NEEDED: {ratio:.4f}")
            self.get_logger().warn(f"Update: left_wheel_radius_multiplier: {ratio:.4f}")
            self.get_logger().warn(f"Update: right_wheel_radius_multiplier: {ratio:.4f}")
        else:
            self.get_logger().info("✓ Wheel radius is correctly calibrated!")
            
    def rotate_in_place(self, angle_deg=360, speed=0.5):
        """Rotate in place for a specific angle"""
        angle_rad = math.radians(angle_deg)
        self.get_logger().info(f"Rotating {angle_deg}° at {speed}rad/s...")
        
        # Record starting orientation
        self.stop_robot()
        time.sleep(1.0)
        
        if not self.controller_odom or not self.gazebo_odom:
            self.get_logger().error("No odometry data received!")
            return
            
        def get_yaw(quaternion):
            """Extract yaw from quaternion"""
            siny_cosp = 2 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
            cosy_cosp = 1 - 2 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
            return math.atan2(siny_cosp, cosy_cosp)
        
        start_ctrl_yaw = get_yaw(self.controller_odom.pose.pose.orientation)
        start_gaz_yaw = get_yaw(self.gazebo_odom.pose.pose.orientation)
        
        # Rotate
        twist = Twist()
        twist.angular.z = speed if angle_rad > 0 else -speed
        duration = abs(angle_rad / speed)
        
        start_time = time.time()
        while (time.time() - start_time) < duration:
            self.cmd_pub.publish(twist)
            time.sleep(0.1)
            
        self.stop_robot()
        time.sleep(1.0)
        
        # Calculate angles
        end_ctrl_yaw = get_yaw(self.controller_odom.pose.pose.orientation)
        end_gaz_yaw = get_yaw(self.gazebo_odom.pose.pose.orientation)
        
        ctrl_angle = math.degrees(end_ctrl_yaw - start_ctrl_yaw)
        gaz_angle = math.degrees(end_gaz_yaw - start_gaz_yaw)
        
        # Normalize angles to [-180, 180]
        ctrl_angle = (ctrl_angle + 180) % 360 - 180
        gaz_angle = (gaz_angle + 180) % 360 - 180
        
        self.get_logger().info(f"\n=== ROTATION TEST ===")
        self.get_logger().info(f"Controller odometry: {ctrl_angle:.1f}°")
        self.get_logger().info(f"Gazebo odometry:    {gaz_angle:.1f}°")
        self.get_logger().info(f"Difference:         {abs(ctrl_angle - gaz_angle):.1f}°")
        
        if abs(ctrl_angle - gaz_angle) > 2.0:
            ratio = gaz_angle / ctrl_angle if ctrl_angle != 0 else 1.0
            self.get_logger().warn(f"SEPARATION MULTIPLIER NEEDED: {ratio:.4f}")
            self.get_logger().warn(f"Update: wheel_separation_multiplier: {ratio:.4f}")
        else:
            self.get_logger().info("✓ Wheel separation is correctly calibrated!")


def main(args=None):
    rclpy.init(args=args)
    calibrator = OdometryCalibrator()
    
    print("\n" + "="*60)
    print("ODOMETRY CALIBRATION TOOL")
    print("="*60)
    print("\nWaiting for odometry data...")
    
    # Spin to receive odometry - increase wait time and check more frequently
    received_data = False
    for i in range(100):  # Try for 10 seconds (100 * 0.1s)
        rclpy.spin_once(calibrator, timeout_sec=0.1)
        if calibrator.controller_odom and calibrator.gazebo_odom:
            received_data = True
            print(f"✓ Odometry data received!")
            break
        if i % 10 == 0 and i > 0:
            print(f"  Still waiting... ({i/10:.0f}s)")
    
    if not received_data:
        print("\n❌ ERROR: Could not receive odometry data!")
        print("   Make sure the simulation is running:")
        print("   ros2 launch rcup_garden slam.launch.py")
        calibrator.destroy_node()
        rclpy.shutdown()
        return
        
    print("\nStarting calibration tests...")
    print("\n1. LINEAR MOTION TEST (tests wheel_radius)")
    print("   Robot will drive forward 1 meter")
    calibrator.drive_forward(distance=1.0, speed=0.2)
    
    time.sleep(2)
    
    print("\n2. ROTATION TEST (tests wheel_separation)")
    print("   Robot will rotate 360 degrees")
    calibrator.rotate_in_place(angle_deg=360, speed=0.5)
    
    print("\n" + "="*60)
    print("CALIBRATION COMPLETE")
    print("="*60)
    print("\nUpdate the multipliers in diff_drive_controllers.yaml")
    print("Then rebuild and restart the simulation.")
    print("="*60 + "\n")
    
    calibrator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
