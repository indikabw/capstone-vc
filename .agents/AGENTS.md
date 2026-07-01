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
*   **Verify Cleanup**: If SSH commands time out, it means the VM is overwhelmed by zombie processes. DO NOT PROCEED until the VM is rebooted or processes are successfully verified dead.
*   **Clean Discovery Graph**: Always stop the ROS 2 daemon process on the VM as part of the cleanup to ensure the DDS node discovery graph is completely reset. This avoids naming collisions and stale topic listings on subsequent launches.

## ROS 2 / Gazebo Anti-Patterns

### 1. "Jump Back in Time" / "Static Cache is Empty" Errors
*   If Nav2 (AMCL) crashes with `tf2::LookupException: Static cache is empty` and logs complain about a "jump back in time", **this is almost always caused by multiple orphaned Gazebo instances publishing conflicting timestamps to `/clock`.**
*   **NEVER apply clock filters** or try to manipulate the `/clock` topic to fix this. It is a symptom of failing to kill old processes. Ensure all old processes are dead before restarting.

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

## Remote Syncing and Compilation (CRITICAL)

### 1. Never Use `rsync --delete` Blindly
*   The VM workspace often contains third-party ROS 2 source packages (like `navigation2`) or binary artifacts in `src/` that do not exist in the local Mac workspace. 
*   When syncing code from the local Mac to the VM, **NEVER** use the `--delete` flag with `rsync` unless explicitly told to. Doing so will obliterate the remote-only source code and cause catastrophic build failures.
*   Stick to: `rsync -avz --exclude='.git' ./ indikabw@172.16.187.128:~/capstone-vc/`

### 2. Disabling `-Werror` on Lyrical (Ubuntu 26.04 / GCC 15)
*   The Lyrical distribution uses GCC 15, which introduces many strict warnings (e.g., `deprecated-declarations`, `free-nonheap-object`).
*   Many ROS 2 packages enforce `-Werror` (warnings as errors) via `ament_lint_auto`. When building unreleased packages from source, this guarantees a build failure.
*   **Always globally disable warnings as errors** when building third-party code: pass `--cmake-args -DAMENT_CMAKE_CXX_WARNINGS_AS_ERRORS=OFF` to your `colcon build` command.
*   If a package explicitly hardcodes `-Werror` in a `CMakeLists.txt` or a `nav2_package.cmake` macro, you must use a tool (like `sed`) to physically strip it from those files before compiling.
