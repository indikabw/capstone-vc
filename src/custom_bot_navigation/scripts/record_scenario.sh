#!/bin/bash
# record_scenario.sh
# This script runs the navigation test and records the destination camera to a rosbag.

BAG_NAME="scenario_run_$(date +%Y%m%d_%H%M%S)"
TOPIC="${TOPIC:-/camera/image_raw}"

echo "Starting rosbag recording for topic $TOPIC..."
ros2 bag record -o $BAG_NAME --topics $TOPIC &
BAG_PID=$!

# Wait a moment for recording to start
sleep 2

echo "Starting navigation scenario (verify_nav.py)..."
PYTHONUNBUFFERED=1 ros2 run custom_bot_navigation verify_nav.py

echo "Navigation script finished. Stopping rosbag recording..."
pkill -INT -f "rosbag2"
wait $BAG_PID

echo "Recording saved to $BAG_NAME"
echo "To convert to video, run: python3 convert_bag_to_video.py $BAG_NAME output.mp4"
