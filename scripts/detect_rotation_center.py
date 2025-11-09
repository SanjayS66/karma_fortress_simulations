#!/usr/bin/env python3
"""
Detect the actual rotation center of the robot in Gazebo by analyzing
how different parts of the robot move during pure rotation.

This script:
1. Records poses of base_footprint, base_final-v3, and wheel positions
2. Rotates robot in place and measures how each joint/link moves
3. Calculates the instantaneous center of rotation (ICR)
4. Reports where the robot is actually rotating about
5. Suggests corrections to base_footprint transform
"""

import rclpy
from rclpy.node import Node
import math
import time
from geometry_msgs.msg import Twist
from tf2_ros import TransformListener, Buffer
from tf2_geometry_msgs import do_transform_point
from geometry_msgs.msg import PointStamped, Point
import numpy as np


class RotationCenterDetector(Node):
    def __init__(self):
        super().__init__('rotation_center_detector')
        
        self.cmd_pub = self.create_publisher(Twist, '/diff_cont/cmd_vel', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Collect poses during rotation
        self.positions_base_fp = []      # base_footprint positions in odom
        self.positions_base_body = []    # base_final-v3 positions in odom
        self.positions_wheel_1 = []      # wheel1 positions in odom
        self.timestamps = []
        
        self.get_logger().info("Ready to detect rotation center...")
    
    def get_pose_in_frame(self, source_frame, target_frame="odom"):
        """Get the position of source_frame in target_frame"""
        try:
            trans = self.tf_buffer.lookup_transform(
                target_frame, source_frame, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0)
            )
            pos = trans.transform.translation
            return np.array([pos.x, pos.y])
        except Exception as e:
            return None
    
    def rotate_and_record(self, duration_sec=10.0, angular_vel=1.0):
        """
        Rotate robot in place and record positions of key frames.
        
        Args:
            duration_sec: How long to rotate (seconds)
            angular_vel: Angular velocity (rad/s)
        """
        self.get_logger().info(f"\n{'='*60}")
        self.get_logger().info(f"Starting rotation for {duration_sec}s at {angular_vel:.2f} rad/s")
        self.get_logger().info(f"{'='*60}\n")
        
        # Stop and wait for stable state
        self.stop_robot()
        time.sleep(1.0)
        
        # Start recording
        twist = Twist()
        twist.angular.z = angular_vel
        start_time = time.time()
        
        while (time.time() - start_time) < duration_sec:
            self.cmd_pub.publish(twist)
            
            # Record positions
            pos_fp = self.get_pose_in_frame("base_footprint", "odom")
            pos_body = self.get_pose_in_frame("base_final-v3", "odom")
            pos_w1 = self.get_pose_in_frame("wheel1", "odom")
            
            if pos_fp is not None and pos_body is not None:
                self.positions_base_fp.append(pos_fp)
                self.positions_base_body.append(pos_body)
                if pos_w1 is not None:
                    self.positions_wheel_1.append(pos_w1)
                self.timestamps.append(time.time() - start_time)
            
            time.sleep(0.05)  # 20 Hz recording
        
        self.stop_robot()
        time.sleep(0.5)
        
        self.get_logger().info(f"Recorded {len(self.positions_base_fp)} position samples")
    
    def stop_robot(self):
        """Stop the robot"""
        twist = Twist()
        self.cmd_pub.publish(twist)
    
    def calculate_rotation_center(self, positions):
        """
        Calculate the rotation center given a list of 2D positions.
        
        Uses least-squares circle fitting to find the center of circular motion.
        
        Args:
            positions: List of [x, y] positions during rotation
            
        Returns:
            (center_x, center_y, radius)
        """
        if len(positions) < 3:
            return None
        
        positions = np.array(positions)
        x = positions[:, 0]
        y = positions[:, 1]
        
        # Algebraic circle fit (least squares)
        # Fit circle: (x - a)^2 + (y - b)^2 = r^2
        A = np.column_stack([x, y, np.ones(len(x))])
        b = x**2 + y**2
        
        try:
            params, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
            cx = params[0] / 2
            cy = params[1] / 2
            r = np.sqrt(params[2] + cx**2 + cy**2)
            
            return cx, cy, r
        except:
            return None
    
    def analyze_rotation(self):
        """Analyze the rotation and calculate ICR"""
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("ROTATION CENTER ANALYSIS")
        self.get_logger().info("="*60)
        
        if len(self.positions_base_fp) < 3:
            self.get_logger().error("Not enough data points collected!")
            return
        
        # Calculate rotation center for each frame
        icr_fp = self.calculate_rotation_center(self.positions_base_fp)
        icr_body = self.calculate_rotation_center(self.positions_base_body)
        icr_w1 = self.calculate_rotation_center(self.positions_wheel_1) if self.positions_wheel_1 else None
        
        self.get_logger().info("\n1. INSTANTANEOUS CENTER OF ROTATION (ICR) per frame:")
        if icr_fp:
            self.get_logger().info(f"   base_footprint ICR: ({icr_fp[0]:.4f}, {icr_fp[1]:.4f}) m, radius: {icr_fp[2]:.4f} m")
        if icr_body:
            self.get_logger().info(f"   base_final-v3 ICR:  ({icr_body[0]:.4f}, {icr_body[1]:.4f}) m, radius: {icr_body[2]:.4f} m")
        if icr_w1:
            self.get_logger().info(f"   wheel1 ICR:         ({icr_w1[0]:.4f}, {icr_w1[1]:.4f}) m, radius: {icr_w1[2]:.4f} m")
        
        # For a rigid body, all frames should have the SAME ICR
        # If they don't, that's your problem!
        if icr_fp and icr_body:
            dist = math.sqrt((icr_fp[0] - icr_body[0])**2 + (icr_fp[1] - icr_body[1])**2)
            self.get_logger().info(f"\n   Distance between base_fp ICR and base_body ICR: {dist:.4f} m")
            
            if dist > 0.01:
                self.get_logger().warn("⚠ PROBLEM DETECTED: Different frames have different rotation centers!")
                self.get_logger().warn("   This means the transform between frames is incorrect or changing.")
        
        # Analyze the center relative to robot geometry
        self.get_logger().info("\n2. GEOMETRIC IMPLICATIONS:")
        
        # Get first base_footprint position to establish robot frame
        first_pos_fp = self.positions_base_fp[0]
        self.get_logger().info(f"   Initial base_footprint position: ({first_pos_fp[0]:.4f}, {first_pos_fp[1]:.4f}) m")
        
        if icr_fp:
            # For diff drive, ICR should be perpendicular to the wheelbase
            # If all wheels have equal speed, ICR should be at infinity (straight line motion)
            # If angular motion, ICR should be along the line perpendicular to wheelbase
            
            # Calculate offset of ICR from initial base_footprint position
            offset_x = icr_fp[0] - first_pos_fp[0]
            offset_y = icr_fp[1] - first_pos_fp[1]
            
            self.get_logger().info(f"   ICR offset from initial base_fp: ({offset_x:.4f}, {offset_y:.4f}) m")
            
            # For pure rotation (both wheels equal speed), ICR should be at base_fp center
            # If not, the base_footprint is not at the actual rotation center
            if abs(offset_x) > 0.01 or abs(offset_y) > 0.01:
                self.get_logger().warn(f"\n⚠ ROTATION CENTER NOT AT BASE_FOOTPRINT!")
                self.get_logger().warn(f"   base_footprint should be shifted by ({-offset_x:.4f}, {-offset_y:.4f}) m")
                self.get_logger().warn(f"   to align with actual rotation center.")
        
        # Get current base_to_robot transform
        self.get_logger().info("\n3. CURRENT TRANSFORMS:")
        try:
            trans = self.tf_buffer.lookup_transform("base_footprint", "base_final-v3", rclpy.time.Time())
            xyz = trans.transform.translation
            self.get_logger().info(f"   base_to_robot (base_fp -> base_body):")
            self.get_logger().info(f"      xyz: ({xyz.x:.4f}, {xyz.y:.4f}, {xyz.z:.4f})")
            
            # Check if this matches the URDF
            urdf_xyz = (-0.36, 0.0, -0.15)
            self.get_logger().info(f"   URDF specifies: xyz = {urdf_xyz}")
            
            if abs(xyz.x - urdf_xyz[0]) > 0.001:
                self.get_logger().warn(f"   ⚠ X offset mismatch! URDF={urdf_xyz[0]}, TF={xyz.x}")
        except Exception as e:
            self.get_logger().error(f"   Could not lookup transform: {e}")
        
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("RECOMMENDATIONS:")
        self.get_logger().info("="*60)
        
        if icr_fp and (abs(icr_fp[0] - first_pos_fp[0]) > 0.01 or abs(icr_fp[1] - first_pos_fp[1]) > 0.01):
            self.get_logger().info("\nTo fix the rotation center alignment:")
            self.get_logger().info(f"1. Update base_to_robot joint origin in URDF:")
            self.get_logger().info(f"   Change xyz from (-0.36, 0.0, -0.15) to")
            self.get_logger().info(f"   ({-icr_fp[0] + first_pos_fp[0]:.4f}, {-icr_fp[1] + first_pos_fp[1]:.4f}, <z unchanged>)")
            self.get_logger().info(f"\n2. Or: Set base_footprint at the actual rotation center point")
            self.get_logger().info(f"   in odom frame: ({icr_fp[0]:.4f}, {icr_fp[1]:.4f})")
        
        print("\n")


def main(args=None):
    rclpy.init(args=args)
    detector = RotationCenterDetector()
    
    # Wait for TF to be available (specifically the 'odom' frame)
    detector.get_logger().info("Waiting up to 15s for TF and controller to initialize...")
    waited = 0.0
    timeout = 15.0
    poll = 0.5
    odom_available = False
    while waited < timeout:
        try:
            # Try a direct lookup with timeout
            trans = detector.tf_buffer.lookup_transform('odom', 'base_footprint', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.5))
            odom_available = True
            detector.get_logger().info("✓ TF ready!")
            break
        except Exception as e:
            pass
        time.sleep(poll)
        waited += poll

    if not odom_available:
        detector.get_logger().error("TF frame 'odom' not found after 15s. Gazebo/controllers may not be running.")
        detector.get_logger().info("STEPS:")
        detector.get_logger().info("1. Launch Gazebo in Terminal 1: ros2 launch rcup_garden gazebo_diff.launch.py")
        detector.get_logger().info("2. Wait ~10-15s for robot to spawn and controllers to load")
        detector.get_logger().info("3. Run this script in Terminal 2: python3 src/rcup_garden/scripts/detect_rotation_center.py")
        detector.destroy_node()
        rclpy.shutdown()
        return
    
    # Run rotation test
    detector.rotate_and_record(duration_sec=8.0, angular_vel=1.0)
    
    # Check if we got data
    if len(detector.positions_base_fp) < 3:
        detector.get_logger().error("Not enough rotation data collected. Controller may not have published odometry.")
        detector.get_logger().info("Check: ros2 topic echo /odom")
        detector.destroy_node()
        rclpy.shutdown()
        return
    
    # Analyze
    detector.analyze_rotation()
    
    detector.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()
