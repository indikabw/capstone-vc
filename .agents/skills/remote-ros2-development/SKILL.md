---
name: remote-ros2-development
description: Use this skill whenever the user wants to compile, build, launch, or test ROS 2 packages in this workspace. It handles the split local-Mac/remote-VM development environment.
---

# Remote ROS 2 Development Workflow

This project uses a split-environment architecture. The source code is edited locally on macOS, but it is compiled and executed inside a remote Ubuntu VM where ROS 2 Lyrical is natively installed.

Whenever the user asks you to build, test, or launch ROS 2 code, **you MUST follow this strict workflow**:

## 1. Do Not Use Local ROS 2 Commands
Do not attempt to run `colcon`, `ros2`, or `rosdep` natively in the local macOS terminal. They are not available locally.

## 2. Sync Code to the VM Before Building
Before executing any remote build commands, you must always push the local modifications to the remote VM to prevent running stale code.
*   **Remote IP:** `172.16.187.128`
*   **Remote User:** `indikabw`
*   **Remote Workspace Path:** `~/capstone-vc/`

**Sync Command**:
Run the following `rsync` command from the local workspace root:
`rsync -avz --exclude='.git' ./ indikabw@172.16.187.128:~/capstone-vc/`

## 3. Remote Execution via SSH
All ROS 2 commands must be wrapped in an SSH call targeting the VM. Because the VM requires environment variables for ROS 2 and GUI forwarding, use the following template for all SSH executions:

`ssh indikabw@172.16.187.128 "export DISPLAY=:0 && source /opt/ros/lyrical/setup.bash && source ~/capstone-vc/install/setup.bash && cd ~/capstone-vc && <YOUR_COMMAND_HERE>"`

*(Replace `<YOUR_COMMAND_HERE>` with the actual `colcon build` or `ros2 launch` command).*

## 4. Launching GUI Applications (Gazebo, RViz)
If you are launching a node that requires a graphical user interface (e.g., Gazebo via `sim.launch.py`, or RViz via Navigation 2):
1. Ensure `export DISPLAY=:0` is included in your SSH payload (as shown in the template above). This ensures the window appears on the VM's primary monitor.
2. If launching a blocking, long-running process (like a `ros2 launch` file), launch it in the background using `nohup ... > /dev/null 2>&1 &` so that your SSH connection doesn't hang indefinitely waiting for the node to exit.
