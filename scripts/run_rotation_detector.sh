#!/bin/bash
# Script to detect and fix rotation center misalignment

echo "=========================================="
echo "ROTATION CENTER DETECTION DIAGNOSTIC"
echo "=========================================="
echo ""
echo "This script will:"
echo "1. Launch Gazebo with the robot"
echo "2. Run rotation diagnostic"
echo "3. Report actual rotation center"
echo "4. Suggest corrections"
echo ""

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Check prerequisites
echo "Checking prerequisites..."
if ! command -v ros2 &> /dev/null; then
    echo -e "${RED}ERROR: ROS2 not found${NC}"
    exit 1
fi

echo -e "${GREEN}✓ ROS2 found${NC}"
echo ""

# Ask user for input
read -p "Start Gazebo simulation? (y/n) " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    echo "Launching Gazebo (in background, you can close the window when done)..."
    cd /home/sanjay/ros2_workspaces/rcup_migration/
    ros2 launch rcup_garden gazebo_diff.launch.py &
    GAZEBO_PID=$!
    
    echo "Waiting 10 seconds for Gazebo to fully start..."
    sleep 10
    
    echo -e "\n${YELLOW}Starting rotation center detection...${NC}\n"
    
    # Run the detection script
    python3 /home/sanjay/ros2_workspaces/rcup_migration/src/rcup_garden/scripts/detect_rotation_center.py
    
    # Kill Gazebo
    echo -e "\n${YELLOW}Cleaning up... closing Gazebo${NC}\n"
    kill $GAZEBO_PID 2>/dev/null || true
else
    echo "Skipped. You can run the detection script manually with:"
    echo "python3 /home/sanjay/ros2_workspaces/rcup_migration/src/rcup_garden/scripts/detect_rotation_center.py"
fi

echo -e "\n${GREEN}Done!${NC}"
