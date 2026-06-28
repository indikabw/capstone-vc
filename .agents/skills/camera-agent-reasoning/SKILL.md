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
