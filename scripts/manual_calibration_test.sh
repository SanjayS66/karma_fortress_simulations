#!/bin/bash

# Manual Odometry Calibration Test
# This script helps you manually calibrate wheel parameters

echo "=========================================="
echo "MANUAL ODOMETRY CALIBRATION"
echo "=========================================="
echo ""
echo "This script will guide you through calibration."
echo "Watch both RViz and Gazebo windows!"
echo ""

# Source ROS2
cd ~/ros2_workspaces/rcup_migration
source install/setup.bash

echo "TEST 1: ROTATION TEST (wheel_separation calibration)"
echo "----------------------------------------------"
echo "The robot will rotate 360° clockwise"
echo ""
echo "Before starting:"
echo "1. Note the robot's orientation in RViz"
echo "2. Note the robot's orientation in Gazebo"
echo ""
read -p "Press ENTER to start rotation test..."

echo "Rotating for ~12.6 seconds..."
# Publish continuously at 10Hz for proper controller response
ros2 topic pub --rate 10 /diff_cont/cmd_vel_unstamped geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.5}}" &
PUB_PID=$!
sleep 12.6

echo "Stopping..."
kill $PUB_PID 2>/dev/null
ros2 topic pub --rate 10 /diff_cont/cmd_vel_unstamped geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" &
PUB_PID=$!
sleep 1
kill $PUB_PID 2>/dev/null
sleep 2

echo ""
echo "QUESTION: Did the robot complete 360° in BOTH RViz and Gazebo?"
echo ""
echo "A) RViz rotated MORE than Gazebo (over-rotated)"
echo "   → wheel_separation is TOO LARGE"
echo "   → Try: wheel_separation_multiplier: 0.90"
echo ""
echo "B) RViz rotated LESS than Gazebo (under-rotated)"
echo "   → wheel_separation is TOO SMALL"  
echo "   → Try: wheel_separation_multiplier: 1.10"
echo ""
echo "C) Both rotated the same amount"
echo "   → wheel_separation is CORRECT!"
echo "   → Keep: wheel_separation_multiplier: 1.0"
echo ""
read -p "Press ENTER to continue to linear test..."

echo ""
echo "TEST 2: LINEAR MOTION TEST (wheel_radius calibration)"
echo "----------------------------------------------"
echo "The robot will drive forward for 5 seconds at 0.2 m/s (should be ~1 meter)"
echo ""
read -p "Press ENTER to start linear test..."

echo "Driving forward for 5 seconds..."
# Publish continuously at 10Hz for proper controller response
ros2 topic pub --rate 10 /diff_cont/cmd_vel_unstamped geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}" &
PUB_PID=$!
sleep 5

echo "Stopping..."
kill $PUB_PID 2>/dev/null
ros2 topic pub --rate 10 /diff_cont/cmd_vel_unstamped geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" &
PUB_PID=$!
sleep 1
kill $PUB_PID 2>/dev/null
sleep 2

echo ""
echo "QUESTION: Did the robot move the SAME distance in RViz and Gazebo?"
echo ""
echo "A) RViz moved MORE than Gazebo"
echo "   → wheel_radius is TOO LARGE"
echo "   → Try: left_wheel_radius_multiplier: 0.95"
echo "   → Try: right_wheel_radius_multiplier: 0.95"
echo ""
echo "B) RViz moved LESS than Gazebo"
echo "   → wheel_radius is TOO SMALL"
echo "   → Try: left_wheel_radius_multiplier: 1.05"
echo "   → Try: right_wheel_radius_multiplier: 1.05"
echo ""
echo "C) Both moved the same distance"
echo "   → wheel_radius is CORRECT!"
echo "   → Keep multipliers at: 1.0"
echo ""

echo "=========================================="
echo "CALIBRATION COMPLETE"
echo "=========================================="
echo ""
echo "Update the multipliers in:"
echo "  config/diff_drive_controllers.yaml"
echo ""
echo "Then rebuild and restart:"
echo "  colcon build --packages-select rcup_garden"
echo "  ros2 launch rcup_garden slam.launch.py"
echo ""
echo "Repeat this test until RViz matches Gazebo perfectly!"
echo "=========================================="
