#!/usr/bin/env python3
"""
Simple test to verify diff_cont controller is working
"""
import rclpy
from geometry_msgs.msg import Twist
import time

rclpy.init()
node = rclpy.create_node('controller_test')

pub = node.create_publisher(Twist, '/diff_cont/cmd_vel', 10)

print("Testing controller...")
print("Sending rotation command (angular.z = 1.0 rad/s) for 3 seconds...")
print("")
print("Check Gazebo:")
print("  - Do the wheels spin?")
print("  - Does the robot rotate?")
print("")

twist = Twist()
twist.angular.z = 1.0

start = time.time()
while (time.time() - start) < 3.0:
    pub.publish(twist)
    print(f"  Publishing command... {time.time()-start:.1f}s", end='\r')
    time.sleep(0.1)

# Stop
twist.angular.z = 0.0
pub.publish(twist)
print("\nDone. Robot should stop now.")

node.destroy_node()
rclpy.shutdown()
