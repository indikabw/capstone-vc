---
name: remote-ros2-development
description: Use this skill whenever the user wants to compile, build, launch, or test ROS 2 packages in this workspace. It handles the split local-Mac/remote-VM development environment.
---

# Remote ROS 2 Development Workflow

This project uses a split-environment architecture. The source code is edited locally on macOS, but it is compiled and executed inside a remote Ubuntu VM where ROS 2 Lyrical is natively installed.

Whenever the user asks you to build, test, or launch ROS 2 code, **you MUST follow this strict workflow**:

## 1. Pre-Flight Cleanup (CRITICAL)
Before starting any new ROS 2 build, launch, or test execution, you MUST always perform the following cleanup tasks to prevent resource starvation or port collisions:
*   **Kill Local Stale Tasks**: Call the `manage_task` tool with action `list` to inspect active background tasks in the Antigravity chat window. If you see active SSH launch commands or behave tests, terminate them using `manage_task` with action `kill` for each task ID.
*   **Clean VM Zombie Processes**: Run the following two SSH commands sequentially to stop the ROS 2 daemon and terminate any orphaned Gazebo, ROS 2, or python processes running in the VM (using character brackets in the regex to prevent the `pkill` process from matching and killing its own SSH shell session):
    1. Stop the ROS 2 daemon process cleanly:
       `ssh -o ConnectTimeout=5 indikabw@172.16.187.128 "source /opt/ros/lyrical/setup.bash && ros2 daemon stop || true"`
    2. Forcefully kill active processes:
       `ssh -o ConnectTimeout=5 indikabw@172.16.187.128 "pkill -9 -f '[r]os2|[g]z|[r]uby|[b]ehave|[c]olcon' || true"`

## 2. Do Not Use Local ROS 2 Commands
Do not attempt to run `colcon`, `ros2`, or `rosdep` natively in the local macOS terminal. They are not available locally.

## 3. Sync Code to the VM Before Building
Before executing any remote build commands, you must always push the local modifications to the remote VM to prevent running stale code.
*   **Remote IP:** `172.16.187.128`
*   **Remote User:** `indikabw`
*   **Remote Workspace Path:** `~/capstone-vc/`

**Sync Command**:
Run the following `rsync` command from the local workspace root:
`rsync -avz --exclude='.git' ./ indikabw@172.16.187.128:~/capstone-vc/`

## 4. Remote Execution via SSH
All ROS 2 commands must be wrapped in an SSH call targeting the VM. Because the VM requires environment variables for ROS 2 and GUI forwarding, use the following template for all SSH executions:

`ssh indikabw@172.16.187.128 "export DISPLAY=:0 && source /opt/ros/lyrical/setup.bash && source ~/capstone-vc/install/setup.bash && cd ~/capstone-vc && <YOUR_COMMAND_HERE>"`

*(Replace `<YOUR_COMMAND_HERE>` with the actual `colcon build` or `ros2 launch` command).*

## 5. Launching GUI Applications (Gazebo, RViz)
If you are launching a node that requires a graphical user interface (e.g., Gazebo via `sim.launch.py`, or RViz via Navigation 2):
1. Ensure `export DISPLAY=:0` is included in your SSH payload (as shown in the template above). This ensures the window appears on the VM's primary monitor.
2. If launching a blocking, long-running process (like a `ros2 launch` file), launch it in the background using `nohup ... > /dev/null 2>&1 &` so that your SSH connection doesn't hang indefinitely waiting for the node to exit.

## 6. Awaiting Asynchronous Tasks & Liveness Timers (CRITICAL)
When triggering builds, tests, or launches that run in the background (asynchronous tasks in the Antigravity chat):
1. **Set Sync Timeout**: If a command is expected to complete within 10 seconds, set `WaitMsBeforeAsync` to a value up to `10000` to run it synchronously and check its output immediately.
2. **Yield Prompt Control**: For longer commands, do NOT say "I will wait 35s and report back" without ending your turn. Output your summary, and stop calling tools (yield control). The system will automatically wake you up when the command outputs or completes.
3. **Use Liveness/Cleanup Timers**: If there's a risk the command might hang (e.g. Gazebo launches, BDD test suites), always schedule a one-shot liveness timer using the `schedule` tool. Set the `TimerCondition` to the target task ID so that you are awakened if it fails to finish or report back in a timely manner.
