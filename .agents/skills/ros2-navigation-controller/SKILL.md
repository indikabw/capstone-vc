---
name: ros2-navigation-controller
description: Provides guidelines for configuring Gazebo Jetty, unifying the TurtleBot4 and OpenManipulator-X URDFs, setting up MoveIt2 SRDFs, and using Nav2 in ROS2 Lyrical Luth.
---

# ROS2 Navigation & Robot Integration Skill

This skill defines the technical approaches for integrating the TurtleBot4, OpenManipulator-X, Nav2, and MoveIt2 in Gazebo Jetty.

## 1. Unified URDF and MoveIt2 Setup
Because MoveIt2 requires a static Semantic Robot Description Format (SRDF) to understand the full kinematic chain and perform collision avoidance, you must create a single `xacro` file that includes both the TurtleBot4 and OpenManipulator-X macros.

**Implementation Steps**:
1.  Create a custom description package (e.g., `custom_bot_description`).
2.  In the main `robot.urdf.xacro`, include the TurtleBot4 base xacro and the OpenManipulator-X xacro.
3.  Define a static joint linking the base of the manipulator to the appropriate mount plate on the TurtleBot4.
4.  Use the MoveIt Setup Assistant to generate a MoveIt2 configuration package (`custom_bot_moveit_config`) targeting this new unified URDF.

## 2. Gazebo Jetty Integration
*   Use `ros_gz_sim` for launching the simulation environment.
*   Use `ros_gz_bridge` to bridge `/tf`, `/odom`, `/cmd_vel`, `/scan`, and joint states between ROS2 and Gazebo Transport.
*   Use `ros_gz_image` specifically for the camera feeds to ensure performant image transport.

## 3. Nav2 and Slam Toolbox
*   Configure the Slam Toolbox in `async` mode for online mapping.
*   Configure Nav2 parameters to account for the altered footprint and center of mass introduced by the OpenManipulator-X.
*   The reasoning agent will interface with Nav2 via the `NavigateToPose` action server.
