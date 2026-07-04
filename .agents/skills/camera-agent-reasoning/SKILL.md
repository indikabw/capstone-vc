---
name: camera-agent-reasoning
description: Provides guidelines for integrating Google ADK 2.0 within a ROS2 Python node to handle reasoning and Vision-Language tasks.
---

# Camera & Agent Reasoning Skill (Google ADK 2.0)

This skill outlines the pattern for executing the LLM reasoning loop inside a ROS2 Lyrical Luth environment using Google ADK 2.0.

## 1. Discrete Trigger Execution
To prevent latency bottlenecks and ROS2 callback starvation, the agent reasoning loop MUST be triggered discretely, not continuously on every camera frame.
*   **Interface**: Use a ROS2 Action Server (e.g., `ReasoningTask.action`).
*   **Goal Content**: The human-language command (e.g., "pick up the box on the right").
*   **Feedback**: Current stage (e.g., "sampling image", "reasoning", "executing move").
*   **Result**: Success/Failure flag and final summary.

## 2. Managing ROS2 Spin vs ADK Blocking Calls
LLM calls via ADK 2.0 are blocking operations. If they run on the main `rclpy` executor thread, they will block all other ROS callbacks (like `image_raw` subscribers).
*   **Implementation Pattern**: 
    *   Use a `MultiThreadedExecutor`.
    *   Define a Mutex/ReentrantLock for the latest camera image.
    *   In the Action Server callback, copy the latest image under the lock.
    *   Execute the ADK 2.0 reasoning call in a dedicated background thread or use the async ADK API while yielding control to `rclpy`.
    
## 3. ADK Workflow
1.  **Image Prep**: Encode the ROS2 `sensor_msgs/msg/Image` (via cv_bridge) into a format expected by the ADK vision models.
2.  **Prompt Engineering**: Construct a system prompt that defines the agent's available actions (e.g., `move_to(x,y)`, `pick(x,y,z)`).
3.  **Action Dispatch**: Parse the ADK output. If the agent decides to move, formulate a `NavigateToPose` goal. If it decides to manipulate, formulate a MoveIt2 Cartesian goal. Send these goals via standard ROS2 Action Clients from within the reasoning node.

## 4. Spatial Reasoning and Inverse Kinematics (IK) Pitfalls
When reasoning about physical interaction with the environment, ADK agents must correctly model spatial offsets and hardware kinematics:
*   **Actuation Point Offset**: Do not calculate IK trajectories solely for the wrist joint (e.g., `link5`). The physical grasp occurs at the center of the gripper jaws, which often protrudes significantly forward of the wrist (e.g., `r + 0.02` for an object at `r`). Failing to account for gripper depth results in the jaws closing on empty air behind the object.
*   **Hardware-Specific Joint Limits**: Gripper joint values are rarely intuitive. For example, on the OpenManipulator-X, a positive joint limit (e.g., `0.02`) opens the gripper fully, while a negative joint limit (e.g., `-0.011`) closes it. If a generic "close gripper" value is set to a positive number, the gripper will simply remain open during the grasp action. Always verify physical joint limits (`lower` and `upper` in URDF) and their directional meaning.
