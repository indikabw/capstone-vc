# Project Context: ROS2 Lyrical Luth + Gazebo Jetty Agentic Navigation

This project involves a TurtleBot4 equipped with an OpenManipulator-X arm navigating in a Gazebo Jetty simulated world. The robot uses a Google ADK 2.0 reasoning node to process human language commands and camera feeds, transforming them into simulated world tasks.

## 1. Architecture Stack
*   **ROS2 Release**: Lyrical Luth
*   **Gazebo Version**: Jetty
*   **Robot Platform**: TurtleBot4 paired with OpenManipulator-X
    *   **Integration Method**: Unified XACRO/URDF combining both robots to allow for a single MoveIt2 SRDF configuration.
*   **Navigation & SLAM**: Navigation2 (`nav2`) and Slam Toolbox.
*   **Arm Control**: MoveIt2 (Inverse Kinematics and Trajectory Planning).
*   **Reasoning Node**: Google ADK 2.0 Agent running within a ROS2 Python node.

## 2. Agentic Reasoning Workflow (ADK 2.0)
*   **Trigger Mechanism**: Discrete triggers via ROS2 Action Servers or Services (e.g., sending a text command "Pick up the red block"). Do **NOT** use continuous topic streams for the reasoning loop to avoid latency bottlenecks.
*   **Execution Flow**:
    1.  Receive command via Action Server.
    2.  Sample the latest image from the ROS2 camera topic (`/camera/image_raw`).
    3.  Pass the command and image to the ADK 2.0 LLM for reasoning.
    4.  Extract actionable output (e.g., Cartesian coordinates or Nav2 goals).
    5.  Dispatch goals to MoveIt2 or Nav2 Action Servers.
    6.  Return success/failure to the caller.

## 3. Behavior-Driven Development (BDD) with `behave`
*   **Scope**: Software integration level (Mocking Gazebo, Nav2, and MoveIt2).
*   **Directory Structure**:
    ```text
    features/
    ├── steps/
    │   └── reasoning_steps.py
    ├── environment.py  (MUST handle rclpy.init() and rclpy.shutdown())
    └── vision_agent.feature
    ```
*   **Execution**: Use `behave` to run the `.feature` files.

## 4. Coding Conventions
*   **ROS2 Python**: Adhere to PEP-8.
*   **Async/Blocking**: Because ADK 2.0 LLM calls are blocking, the reasoning node **must** utilize MultiThreadedExecutors or run the LLM calls in separate threads so that ROS2 callbacks (like the camera subscriber) are not starved.
*   **Dependencies**: Rely on `ros_gz_sim`, `ros_gz_bridge`, and `ros_gz_image` for Gazebo Jetty integration. Avoid deprecated `ros_ign` packages.

## Instructions for the AI Assistant
Before writing code or debugging, **always** refer to the specific `.agents/skills` directories (`ros2-navigation-controller`, `camera-agent-reasoning`, `bdd-gherkin-specs`) for exact implementations of the Navigation, Reasoning, and BDD test setups.
