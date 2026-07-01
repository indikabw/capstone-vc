# Custom Agent Rules for Capstone-VC Workspace

These rules govern the development, execution, and environment lifecycle for this workspace. All agents MUST strictly adhere to them.

## Stale Session and Task Management (CRITICAL)

To prevent resource starvation, lock-ups, and orphaned processes on the development VM and in the Antigravity chat interface, you must maintain clean process lifecycles.

### 1. Local Task Cleanup (Antigravity Chat Window)
*   **Check Active Tasks**: Before launching any long-running command (like `ssh`, `ros2 launch`, `behave` tests, or background builders), you MUST query running tasks using the `manage_task` tool with action `list`.
*   **Terminate Stale/Duplicate Tasks**: If there are active tasks executing SSH commands or ROS2 workflows, use `manage_task` with action `kill` and their respective Task IDs to terminate them. Do not let duplicate or defunct sessions accumulate in the user interface.

### 2. VM Process Cleanup
*   **Kill Zombie Processes**: Before launching new simulation, control, or navigation runs on the VM, execute these two remote SSH commands sequentially:
    1. Stop the ROS 2 daemon process cleanly (requires sourcing the setup first):
       `ssh -o ConnectTimeout=5 indikabw@172.16.187.128 "source /opt/ros/lyrical/setup.bash && ros2 daemon stop || true"`
    2. Forcefully kill active processes (using character brackets in regex to prevent the `pkill` process from matching and killing its own SSH shell session):
       `ssh -o ConnectTimeout=5 indikabw@172.16.187.128 "pkill -9 -f '[r]os2|[g]z|[r]uby|[b]ehave|[c]olcon|[c]omponent' || true"`
*   **Clean Discovery Graph**: Always stop the ROS 2 daemon process on the VM as part of the cleanup to ensure the DDS node discovery graph is completely reset. This avoids naming collisions and stale topic listings on subsequent launches.

## Handling Long-Running Commands & Waiting

To avoid blocking the developer's chat session or leaving dangling asynchronous tasks:

### 1. Wait Synchronously for Short Tasks
*   For commands expected to finish within 10 seconds, configure `WaitMsBeforeAsync` up to `10000` (10 seconds) in the `run_command` tool call. This forces synchronous execution and returns command outputs immediately.

### 2. Yield Turn for Asynchronous Tasks
*   For longer processes (e.g., compile runs, tests, Gazebo startups), launch the task asynchronously and immediately yield your turn. 
*   **Do NOT** make claims like "I will wait X seconds and check back" without ending your turn. The Antigravity environment will wake you up automatically as soon as the background task outputs or completes.

### 3. Use Liveness Timers
*   If there is a possibility that a command or test suite may hang indefinitely (e.g., `behave` tests getting stuck or Gazebo failing to launch), use the `schedule` tool to set a one-shot liveness timer (e.g., 3-5 minutes).
*   Set the `TimerCondition` to the specific task ID so that if the background task completes successfully, the timer is automatically cancelled, ensuring you do not generate spam.
