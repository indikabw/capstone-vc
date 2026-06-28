# System Architecture Specification

## Overview
This document specifies the software architecture for the ROS2 Lyrical Luth robotic agent. The system integrates a TurtleBot4 mobile base and an OpenManipulator-X robotic arm operating within the AWS RoboMaker Small House simulated in Gazebo Jetty. The supervisory intelligence is powered by a Google ADK 2.0 reasoning node.

---

## 1. Physical & Kinematic Representation (Unified URDF)
To enable full-body collision avoidance and coordinated planning, the robot's physical representation must be unified into a single kinematic chain for MoveIt2.

*   **Custom Description Package (`custom_bot_description`)**:
    *   A single `robot.urdf.xacro` entry point.
    *   Includes the TurtleBot4 base macro.
    *   Includes the OpenManipulator-X arm macro.
    *   Defines a static joint rigidly attaching the base of the manipulator to the TurtleBot4's upper mounting plate.
*   **Semantic Robot Description Format (SRDF)**:
    *   A custom `custom_bot_moveit_config` package generated via MoveIt Setup Assistant targeting the unified URDF.
    *   Defines planning groups for the arm (`manipulator`) and the gripper (`gripper`).

---

## 2. Simulation Environment (Gazebo Jetty)
The simulation is hosted in Gazebo (Jetty release), specifically utilizing the AWS RoboMaker Small House World.

*   **Simulation Engine**: `ros_gz_sim`
*   **ROS/Gazebo Bridge (`ros_gz_bridge`)**:
    *   Bridges `/tf`, `/odom`, `/cmd_vel`, `/scan`, and `/joint_states` bidirectionally between ROS2 and Gazebo Transport.
*   **Vision Transport (`ros_gz_image`)**:
    *   Bridges the camera data to `/camera/image_raw` using specialized high-performance image transport instead of the generic bridge to ensure framerate stability.

---

## 3. Navigation & Localization (Nav2)
The robot relies on the ROS2 Navigation Framework (Nav2) for moving around the household.

*   **Map**: A static Occupancy Grid map of the AWS RoboMaker Small House World generated offline.
*   **Localization**: `nav2_amcl` (Adaptive Monte Carlo Localization) is used to localize the robot against the static map.
*   **Planner/Controller**: Standard Nav2 plugins (e.g., SmacPlanner, DWB Controller) configured with a custom footprint that encompasses both the TurtleBot4 base and the swept volume of the OpenManipulator-X arm.

---

## 4. Agentic AI Layer (Google ADK 2.0 Reasoning Node)
This custom Python ROS2 node acts as the supervisor, translating high-level human commands into low-level robotic actions.

### 4.1 Interface
*   **Action Server**: The node exposes an Action Server (e.g., `ReasoningTask.action`).
    *   **Goal**: The natural language instruction (e.g., "Find the kitchen", "Pick up the red book").
    *   **Result**: Success/Failure status and a semantic summary of the execution.

### 4.2 Concurrency & Real-Time Safety
Google ADK 2.0 LLM calls are inherently blocking.
*   **Executor**: The node spins using a `rclpy.executors.MultiThreadedExecutor`.
*   **Image Sampling**: A ROS2 subscriber continuously listens to `/camera/image_raw`. It stores the latest frame under a `threading.Lock()` (Mutex).
*   **Execution**: When a reasoning goal is received, the node clones the latest image under the mutex lock and offloads the blocking ADK LLM call to a background thread. This ensures the ROS2 node continues to process TF, `/odom`, and `/camera/image_raw` callbacks.

### 4.3 Semantic Exploration & Spatial Mapping
The agent builds and maintains a spatial semantic memory, mapping abstract concepts to 2D polygon boundaries on the static map.
*   **Exploration**: If commanded to "Find the kitchen", and the kitchen is not in its memory, the agent orchestrates a search pattern (dispatching Nav2 goals) to explore the environment.
*   **Spatial Reasoning**: Once the ADK Vision model recognizes visual cues of a kitchen (stove, fridge, sink), the agent pivots to scan the surrounding area. By combining depth/visual estimates with the 2D occupancy grid map and its AMCL pose, the reasoning node deduces the physical boundaries of the room.
*   **Memory Update**: The node saves the `{ "kitchen": Polygon(point1, point2, ...) }` mapping into its internal spatial dictionary. Future queries for the kitchen can sample target poses within this bounded polygon.

### 4.4 Action Dispatching
The reasoning node acts as a client to the lower-level subsystems:
1.  **Navigation**: Formulates a `NavigateToPose` action goal and sends it to the Nav2 Action Server.
2.  **Manipulation**: Computes Cartesian target coordinates and sends them to the MoveIt2 Action Server to control the OpenManipulator-X.

### 4.5 Chain-of-Thought (CoT) Agent Loop
To handle complex instructions (e.g., "bring the red mug on the coffee table to the kitchen"), the ADK 2.0 node executes a continuous Chain-of-Thought reasoning loop rather than a single pass.
*   **Context Management**: To prevent the LLM context window from overflowing during long tasks, the agent relies on state summarization. The context strictly maintains the **current environmental state** and the **plan for remaining subtasks**. Detailed logs of already completed subtasks are discarded.
*   **Task Decomposition & Intent Inference**: The agent breaks the high-level goal into sequential subtasks. It infers implicit requirements, such as reasoning that "bring" implies finding a stable flat surface (like a countertop) within the "kitchen" polygon to place the object.
*   **Execution & Visual Verification**: The agent executes the current subtask (dispatching Nav2/MoveIt2 goals). Upon subtask completion, it samples the camera to visually verify success (e.g., verifying the mug is securely in the gripper). 
*   **Error Handling**: In this initial architecture, if a subtask fails (e.g., a MoveIt2 planning failure or dropping an object), the agent will immediately abort the CoT loop and return a failure state to the user. Future iterations may introduce visual recovery loops.
*   **Completion Criteria**: The task is only marked as "Done" when the agent's visual/spatial state evaluation matches the criteria of the final subtask in its plan (e.g., visually confirming the mug is resting on the kitchen counter).
