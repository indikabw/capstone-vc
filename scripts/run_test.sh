#!/bin/bash
set -e

if [ -f ".env" ]; then
    echo "Sourcing .env file..."
    source .env
    export $(grep -v '^#' .env | xargs)
fi

# Make sure old processes are dead
pkill -9 -f '[r]os2|[g]z|[r]uby|[b]ehave|[c]olcon|[c]omponent|[p]ython3.*custom_bot|robot_state_publisher|test_nav_and_pick' || true
sleep 2

source /opt/ros/lyrical/setup.bash
source ~/moveit_ws/install/setup.bash
source ~/capstone-vc/install/setup.bash

export LIBGL_ALWAYS_SOFTWARE=1
export GALLIUM_DRIVER=llvmpipe
export DISPLAY=:0

# 1. Start Gazebo Sim Server in background
echo "Starting simulation..."
nohup ros2 launch custom_bot_gazebo sim.launch.py headless:=true > /tmp/sim.log 2>&1 &
SIM_PID=$!

# Wait for simulation to be active
echo "Waiting for simulation clock..."
until ros2 topic echo /clock --once > /dev/null 2>&1; do
    sleep 1
done
echo "Simulation clock active."

# 2. Start Navigation in background
echo "Starting navigation..."
nohup ros2 launch custom_bot_navigation navigation.launch.py > /tmp/nav.log 2>&1 &
NAV_PID=$!

# Wait for Nav2
echo "Waiting for navigation action server..."
until ros2 action list | grep -q "/navigate_to_pose"; do
    sleep 1
done
echo "Navigation action server online. Sleeping 15s for lifecycle activation..."
sleep 15

# 3. Start controllers
echo "Spawning arm and gripper controllers..."
nohup ros2 launch custom_bot_moveit_config demo.launch.py > /tmp/controllers.log 2>&1 &
sleep 5

# 4. Start Reasoning node in background
echo "Starting reasoning node..."
nohup ros2 run custom_bot_reasoning reasoning_node > /tmp/reasoning.log 2>&1 &
REASONING_PID=$!

# Wait for MoveIt2 move_action to come online
until ros2 action list | grep -q "/move_action"; do sleep 1; done
echo "MoveIt2 action server online."
echo "Waiting for reasoning action server..."
until ros2 action list | grep -q "/reasoning_task"; do
    sleep 1
done
echo "Reasoning action server online."

# 5. Start recording topic
echo "Starting rosbag record..."
BAG_NAME="red_cube_run"
rm -rf $BAG_NAME
ros2 bag record -o $BAG_NAME --topics /camera/image_raw /destination_camera/image_raw > /tmp/bag.log 2>&1 &
BAG_PID=$!
sleep 2

# 6. Send action goal
echo "Sending navigation command to reasoning node..."
python3 scripts/test_nav_and_pick.py

echo "Command finished! Stopping recording..."
sleep 2

# 7. Stop recording
kill -TERM $BAG_PID || true
sleep 3

# 8. Kill Gazebo, Nav, MoveIt, and Reasoning
echo "Cleaning up processes..."
pkill -9 -f '[r]os2|[g]z|[r]uby|[b]ehave|[c]olcon|[c]omponent|[p]ython3.*custom_bot|robot_state_publisher|test_nav_and_pick' || true

# 9. Convert bag to video
echo "Converting bag to video..."
python3 scripts/convert_bag_to_video.py $BAG_NAME red_cube_run.mp4 /camera/image_raw
python3 scripts/convert_bag_to_video.py $BAG_NAME red_cube_run_overhead.mp4 /destination_camera/image_raw

echo "Done! Video saved to red_cube_run.mp4"
