# Phased Implementation Plan: ROS2 Agentic Navigation & Reasoning

This document outlines a phased implementation approach for the TurtleBot4 + OpenManipulator-X agentic robot in Gazebo Jetty, utilizing ROS2 Lyrical Luth and Google ADK 2.0.

## User Review Required

> [!IMPORTANT]
> Please review this phased approach. It balances setting up the foundational ROS2/Gazebo elements first, before incrementally adding complexity via the ADK 2.0 reasoning layer and MoveIt2 manipulation capabilities. Once you approve, we will use this plan to guide our development process.

## Open Questions

> [!NOTE]
> All initial questions resolved:
> - URDF descriptions for TurtleBot4 and OpenManipulator-X will be downloaded.
> - BDD testing framework (`behave`) will be established before implementing the reasoning node.
> - SLAM vs AMCL: After reviewing previous logs, we will use **AMCL with a static map**. While the workspace skills mentioned SLAM, using AMCL provides a much more stable foundation for the agent to perform semantic exploration without dealing with mapping drift.

## Proposed Phased Approach

---

### Phase 1: Foundational Simulation & Navigation Layer

**Goal:** Establish the simulated Gazebo Jetty world and basic autonomous navigation (without the agentic layer).

- **Phase 1.1: Unified URDF & Gazebo Jetty Integration**
  - Download and source the official TurtleBot4 and OpenManipulator-X URDF/XACRO descriptions.
  - Create a custom description package unifying the TurtleBot4 and OpenManipulator-X XACROs into a single robot URDF.
  - Set up `ros_gz_sim` to launch the Gazebo Jetty world.
  - Configure `ros_gz_bridge` and `ros_gz_image` for essential topics (`/tf`, `/odom`, `/cmd_vel`, `/scan`, `/camera/image_raw`, and joint states).

- **Phase 1.2: SLAM and Navigation (Nav2)**
  - Configure **AMCL** and Nav2 to use a pre-generated static map of the AWS RoboMaker Small House World. This provides a stable environment for semantic exploration.
  - Set up Nav2 with adjusted footprint parameters reflecting the added OpenManipulator-X arm.
  - Verify that the robot can receive a standard `NavigateToPose` goal and successfully plan/execute a collision-free path to a target.

### Phase 2: Basic Agentic Reasoning (Navigation)

**Goal:** Integrate the Google ADK 2.0 Agent to handle simple spatial and visual navigation commands.

- **Phase 2.1: BDD Mocking Framework Setup**
  - Implement the `behave` Gherkin specifications and the Pytest wrapper.
  - Create the `environment.py` and mocks for Nav2 and MoveIt2 action servers to allow testing the reasoning loop in isolation.

- **Phase 2.2: Reasoning Node & Action Server**
  - Develop the core ROS2 Python node utilizing a `MultiThreadedExecutor` to prevent ADK 2.0 from blocking ROS callbacks.
  - Expose a ROS2 Action Server to receive human language commands.
  - Implement the image sampling logic (fetching the latest frame from `/camera/image_raw` using cv_bridge and thread-safe locks).

- **Phase 2.3: Simple Agentic Tasks**
  - Prompt engineer the ADK 2.0 model to handle basic commands like "find and navigate to the kitchen" or "move close to the red mug".
  - Parse the LLM's structured output into a `NavigateToPose` goal and dispatch it to the Nav2 Action Server.

### Phase 3: Complex Agentic Reasoning (Manipulation)

**Goal:** Extend the agent's capabilities to perform multi-step tasks involving both navigation and manipulation.

- **Phase 3.1: MoveIt2 Integration**
  - Use MoveIt Setup Assistant to generate the `custom_bot_moveit_config` based on the unified URDF.
  - Validate Inverse Kinematics and Trajectory Planning for the OpenManipulator-X within ROS2/Gazebo.

- **Phase 3.2: Multi-Step Reasoning & Dispatch**
  - Expand the ADK 2.0 system prompt to include manipulation capabilities (`pick`, `place`, `move_arm_to`).
  - Update the reasoning node to parse manipulation actions and dispatch them to the MoveIt2 Action Server.
  - Handle complex tasks (e.g., "Move the mug on the coffee table to the kitchen") by orchestrating a sequence: Navigate -> Detect -> Pick -> Navigate -> Place.

## Verification Plan

### Automated Tests
- `colcon test --packages-select [package_name]` to run the BDD `behave` integration tests for the reasoning node. The mock Nav2 and MoveIt2 servers will assert that the correct Cartesian/Pose goals were dispatched by the agent.

### Manual Verification
- **Phase 1:** Launch Gazebo, manually set a 2D Nav Goal in RViz, and observe the robot navigating.
- **Phase 2:** Launch Gazebo and the Reasoning Node. Send a command via ROS2 CLI (e.g., `ros2 action send_goal /reasoning_task ... "Navigate to the red cube"`) and observe behavior.
- **Phase 3:** Send a complex manipulation command via the Action CLI and observe the complete pick-and-place operation in Gazebo.
