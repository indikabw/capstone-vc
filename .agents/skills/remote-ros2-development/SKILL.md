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
       `ssh -o ConnectTimeout=5 indikabw@172.16.187.128 "pkill -9 -f '[r]os2|[g]z|[r]uby|[b]ehave|[c]olcon|[c]omponent|[p]ython3.*custom_bot|robot_state_publisher' || true"`

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

**CRITICAL WARNING:** NEVER use the `--delete` flag with `rsync`. The VM workspace often contains third-party ROS 2 source packages (e.g. `navigation2` because it's not yet released on `lyrical`) that do not exist locally on the Mac. Using `--delete` will obliterate them and cause catastrophic build failures.

## 4. Remote Execution via SSH
All ROS 2 commands must be wrapped in an SSH call targeting the VM. Because the VM requires environment variables for ROS 2 and GUI forwarding, use the following template for all SSH executions:

`ssh indikabw@172.16.187.128 "export DISPLAY=:0 && source /opt/ros/lyrical/setup.bash && source ~/capstone-vc/install/setup.bash && cd ~/capstone-vc && <YOUR_COMMAND_HERE>"`

*(Replace `<YOUR_COMMAND_HERE>` with the actual `colcon build` or `ros2 launch` command).*

**COMPILATION WARNING (GCC 15 on Lyrical):** 
When compiling third-party code like Navigation2, GCC 15 emits strict warnings that ROS 2 linters treat as errors (via `-Werror`). You MUST always append `--cmake-args -DAMENT_CMAKE_CXX_WARNINGS_AS_ERRORS=OFF` to your `colcon build` commands. If packages manually hardcode `-Werror` inside their CMake macros, you must strip them out using `sed` before compiling.

## 5. Launching GUI Applications (Gazebo, RViz)
If you are launching a node that requires a graphical user interface (e.g., Gazebo via `sim.launch.py`, or RViz via Navigation 2):
1. Ensure `export DISPLAY=:0` is included in your SSH payload (as shown in the template above). This ensures the window appears on the VM's primary monitor.
2. **Gazebo Headless Rendering (CRITICAL)**: Because the VM lacks hardware GPU acceleration, Gazebo will crash with an OpenGL error (`OpenGL 3.3 is not supported`) unless you force software rendering. Always prepend these variables to your Gazebo launch commands: `export LIBGL_ALWAYS_SOFTWARE=1 && export GALLIUM_DRIVER=llvmpipe && <command>`
3. If launching a blocking, long-running process (like a `ros2 launch` file), launch it in the background using `nohup ... > /dev/null 2>&1 &` so that your SSH connection doesn't hang indefinitely waiting for the node to exit.

## 6. Awaiting Asynchronous Tasks & Liveness Timers (CRITICAL)
When triggering builds, tests, or launches that run in the background (asynchronous tasks in the Antigravity chat):
1. **Set Sync Timeout**: If a command is expected to complete within 10 seconds, set `WaitMsBeforeAsync` to a value up to `10000` to run it synchronously and check its output immediately.
2. **Yield Prompt Control**: For longer commands, do NOT say "I will wait 35s and report back" without ending your turn. Output your summary, and stop calling tools (yield control). The system will automatically wake you up when the command outputs or completes.
3. **Use Liveness/Cleanup Timers**: If there's a risk the command might hang (e.g. Gazebo launches, BDD test suites), always schedule a one-shot liveness timer using the `schedule` tool. Set the `TimerCondition` to the target task ID so that you are awakened if it fails to finish or report back in a timely manner.

## 7. Pre-Flight Topic Verification (CRITICAL)
Whenever you are about to record a rosbag, test an agent's vision pipeline, or verify a node that relies on a specific topic (e.g., `/camera/image_raw`), you **must** verify that the topic exists and is actively publishing data.
Blindly running scripts without verifying topic output is a common pitfall that leads to wasted simulation runs.

1. **Check if the topic exists:**
   `ssh indikabw@172.16.187.128 "source /opt/ros/lyrical/setup.bash && ros2 topic info <TOPIC_NAME>"`
2. **Verify data is actually flowing:**
   `ssh indikabw@172.16.187.128 "source /opt/ros/lyrical/setup.bash && ros2 topic echo <TOPIC_NAME> --once > /dev/null"`
If `ros2 topic echo` hangs, the topic is not publishing data (e.g., Gazebo bridge misconfigured). You must fix the root cause before proceeding.
