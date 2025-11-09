#!/usr/bin/env python3
"""
Detect the actual rotation center by comparing odometry from the controller
with the actual base_footprint motion in Gazebo during pure rotation.

SIMPLER APPROACH: Subscribe to odometry directly instead of TF lookups.
"""

import rclpy
from rclpy.node import Node
import math
import time
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np


class RotationCenterDetector(Node):
    def __init__(self):
        super().__init__('rotation_center_detector_v2')
        
        self.cmd_pub = self.create_publisher(Twist, '/diff_cont/cmd_vel', 10)
        
        # Subscribe to controller odometry (not Gazebo /odom, which is the ground truth)
        # We use controller odometry to detect rotation center based on what the controller calculates
        self.odom_sub = self.create_subscription(Odometry, '/diff_cont/odom', self.odom_callback, 10)
        
        # Collect odometry data during rotation
        self.odom_positions = []  # (x, y) in odom frame
        self.timestamps = []
        self.latest_odom = None
        
        self.get_logger().info("Rotation Center Detector v2 ready...")
    
    def odom_callback(self, msg):
        """Called when odometry is received"""
        self.latest_odom = msg
    
    def rotate_and_record(self, duration_sec=10.0, angular_vel=1.0):
        """
        Rotate robot in place and record odometry data.
        """
        self.get_logger().info(f"\n{'='*60}")
        self.get_logger().info(f"Starting rotation for {duration_sec}s at {angular_vel:.2f} rad/s")
        self.get_logger().info(f"{'='*60}\n")
        
        # Stop and wait
        self.stop_robot()
        time.sleep(2.0)
        
        # Clear data
        self.odom_positions = []
        self.timestamps = []
        
        # Start rotation - publish command repeatedly at higher frequency
        twist = Twist()
        twist.angular.z = angular_vel
        start_time = time.time()
        record_count = 0
        
        while (time.time() - start_time) < duration_sec:
            # Publish command at high frequency to ensure it's received
            self.cmd_pub.publish(twist)
            
            # Spin once to process messages
            rclpy.spin_once(self, timeout_sec=0.01)
            
            # Record if we have odometry
            if self.latest_odom:
                pos = self.latest_odom.pose.pose.position
                self.odom_positions.append(np.array([pos.x, pos.y]))
                self.timestamps.append(time.time() - start_time)
                record_count += 1
            
            time.sleep(0.02)  # 50 Hz command publishing
        
        self.stop_robot()
        time.sleep(1.0)
        
        self.get_logger().info(f"Recorded {record_count} odometry samples")
        if record_count < 10:
            self.get_logger().warn(f"WARNING: Only {record_count} samples recorded. Controller may not be responding.")
            self.get_logger().warn(f"Try manually sending: ros2 topic pub /diff_cont/cmd_vel geometry_msgs/Twist '{{angular: {{z: 1.0}}}}'")
            self.get_logger().warn(f"Check if wheels move in Gazebo. If not, controller is not working.")
    
    def stop_robot(self):
        """Stop the robot"""
        twist = Twist()
        self.cmd_pub.publish(twist)
        time.sleep(0.1)
    
    def calculate_rotation_center(self, positions):
        """
        Calculate rotation center using least-squares circle fitting.
        
        Returns:
            (center_x, center_y, radius) or None if fit failed
        """
        if len(positions) < 3:
            return None
        
        positions = np.array(positions)
        x = positions[:, 0]
        y = positions[:, 1]
        
        # Algebraic circle fit
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
        """Analyze rotation data"""
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("ROTATION CENTER ANALYSIS")
        self.get_logger().info("="*60)
        
        if len(self.odom_positions) < 3:
            self.get_logger().error("Not enough samples! Got only " + str(len(self.odom_positions)))
            return
        
        # Calculate ICR (instantaneous center of rotation)
        icr = self.calculate_rotation_center(self.odom_positions)
        
        if not icr:
            self.get_logger().error("Failed to calculate rotation center!")
            return
        
        cx, cy, r = icr
        
        self.get_logger().info(f"\n1. ODOMETRY ANALYSIS:")
        self.get_logger().info(f"   Rotation center (ICR): ({cx:.4f}, {cy:.4f}) m")
        self.get_logger().info(f"   Radius of motion:     {r:.4f} m")
        
        # Initial position
        first_pos = self.odom_positions[0]
        self.get_logger().info(f"\n2. STARTING POSITION:")
        self.get_logger().info(f"   First odometry:  ({first_pos[0]:.4f}, {first_pos[1]:.4f}) m")
        
        # For pure rotation, the rotation center should be at the starting position
        # (or very close to it, since the base_footprint should stay roughly in place while rotating)
        offset_x = cx - first_pos[0]
        offset_y = cy - first_pos[1]
        offset_dist = math.sqrt(offset_x**2 + offset_y**2)
        
        self.get_logger().info(f"\n3. ROTATION CENTER OFFSET FROM START:")
        self.get_logger().info(f"   Δ position: ({offset_x:.4f}, {offset_y:.4f}) m")
        self.get_logger().info(f"   Distance:   {offset_dist:.4f} m")
        
        # Diagnostics
        self.get_logger().info(f"\n4. DIAGNOSTICS:")
        if offset_dist < 0.02:
            self.get_logger().info("   ✓ EXCELLENT: base_footprint at rotation center!")
            self.get_logger().info("   No adjustment needed to URDF.")
        elif offset_dist < 0.1:
            self.get_logger().info(f"   ⚠ GOOD: Small offset detected ({offset_dist:.3f} m)")
            self.get_logger().info(f"   May want to adjust base_to_robot xyz by approximately:")
            self.get_logger().info(f"   Δx: {-offset_x:.4f} m, Δy: {-offset_y:.4f} m")
        else:
            self.get_logger().warn(f"   ✗ PROBLEM: Large offset ({offset_dist:.3f} m)!")
            self.get_logger().warn(f"   base_footprint is NOT at rotation center.")
            self.get_logger().info(f"\n   RECOMMENDED FIX:")
            self.get_logger().info(f"   In urdf/ArmPlate.urdf.xacro around line 310, update:")
            self.get_logger().info(f"   Current:  <origin xyz=\"-0.36 0.0 -0.15\" rpy=\"...\"/>")
            self.get_logger().info(f"   New:      <origin xyz=\"{-0.36-offset_x:.4f} {-offset_y:.4f} -0.15\" rpy=\"...\"/>")
            self.get_logger().info(f"\n   Then rebuild: colcon build --symlink-install")
            self.get_logger().info(f"   And retest.")
        
        # Show trajectory for debugging
        self.get_logger().info(f"\n5. TRAJECTORY STATS:")
        positions_arr = np.array(self.odom_positions)
        self.get_logger().info(f"   First pose:  ({positions_arr[0, 0]:.4f}, {positions_arr[0, 1]:.4f})")
        self.get_logger().info(f"   Last pose:   ({positions_arr[-1, 0]:.4f}, {positions_arr[-1, 1]:.4f})")
        min_x, max_x = positions_arr[:, 0].min(), positions_arr[:, 0].max()
        min_y, max_y = positions_arr[:, 1].min(), positions_arr[:, 1].max()
        self.get_logger().info(f"   X range:     [{min_x:.4f}, {max_x:.4f}] (span: {max_x-min_x:.4f})")
        self.get_logger().info(f"   Y range:     [{min_y:.4f}, {max_y:.4f}] (span: {max_y-min_y:.4f})")
        
        print("\n")


def main(args=None):
    rclpy.init(args=args)
    detector = RotationCenterDetector()
    
    detector.get_logger().info("Waiting for first odometry message (max 10s)...")
    
    # Spin for a bit to let subscriptions connect
    start_wait = time.time()
    while detector.latest_odom is None and (time.time() - start_wait) < 10.0:
        rclpy.spin_once(detector, timeout_sec=0.1)
        time.sleep(0.1)
    
    if detector.latest_odom is None:
        detector.get_logger().error("No odometry received after 10s! Make sure:")
        detector.get_logger().error("  1. Gazebo is running: ros2 launch rcup_garden gazebo_diff.launch.py")
        detector.get_logger().error("  2. Wait ~15s for robot spawn and controller loading")
        detector.get_logger().error("  3. Verify: ros2 topic list | grep odom")
        detector.destroy_node()
        rclpy.shutdown()
        return
    
    detector.get_logger().info("✓ Odometry connected!")
    
    # Run rotation test
    detector.rotate_and_record(duration_sec=10.0, angular_vel=1.0)
    
    # Analyze
    detector.analyze_rotation()
    
    detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
