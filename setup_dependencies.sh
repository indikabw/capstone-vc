#!/bin/bash
# Script to install system dependencies for ROS 2 Lyrical source packages on the VM.
set -e

echo "============================================="
echo "Initializing and updating rosdep..."
echo "============================================="
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    echo "Running 'sudo rosdep init'..."
    sudo rosdep init
else
    echo "rosdep already initialized."
fi

echo "Running 'rosdep update'..."
rosdep update

echo "============================================="
echo "Installing dependencies for workspace..."
echo "============================================="
# Install libompl-dev natively since the ROS Lyrical binary package is missing
sudo apt-get install -y libompl-dev

cd "$(dirname "$0")"
rosdep install --from-paths src --ignore-src -r -y --rosdistro lyrical --skip-keys "ompl"

echo "============================================="
echo "Dependencies setup complete!"
echo "============================================="
