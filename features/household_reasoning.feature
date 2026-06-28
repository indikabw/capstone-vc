Feature: Household Agentic Reasoning and Navigation

  Background:
    Given the ROS2 Lyrical Luth reasoning action server is online
    And the unified robot is localized in the AWS RoboMaker Small House using AMCL
    And the camera topic "/camera/image_raw" is active
    And the Nav2 and MoveIt2 Action Servers are mocked for testing

  Scenario: Semantic Exploration and Spatial Boundary Mapping
    Given the agent's semantic dictionary is empty
    When the user commands "Find the kitchen"
    Then the agent should dispatch a series of Nav2 `NavigateToPose` goals to explore the map
    And when the camera detects a "stove", "sink", and "fridge"
    Then the agent should pivot to scan the surrounding area to deduce room boundaries using the 2D map
    And the agent should save the computed 2D polygon boundary as "kitchen" in its semantic dictionary
    And the action server should return success with summary "Kitchen boundaries located and saved."

  Scenario: Fetch and Deliver Object Using Semantic Memory
    Given the agent's semantic dictionary contains "coffee_table" and "bookshelf" spatial polygons
    When the user commands "Pick the book from the coffee table and place it next to the other books in the bookshelf"
    Then the agent should sample a target coordinate within the "coffee_table" polygon
    And the agent should dispatch a Nav2 `NavigateToPose` goal to the sampled coordinate
    And when the Nav2 goal succeeds
    And the camera detects a "book" at Cartesian coordinate [0.6, 0.0, 0.4] relative to the camera
    Then the agent should dispatch a MoveIt2 Cartesian goal to [0.6, 0.0, 0.4] to pick the "book"
    And when the MoveIt2 pick goal succeeds
    Then the agent should sample a target coordinate within the "bookshelf" polygon
    And the agent should dispatch a Nav2 `NavigateToPose` goal to the sampled coordinate
    And when the Nav2 goal succeeds
    Then the agent should dispatch a MoveIt2 Cartesian goal to place the "book"
    And the action server should return success with summary "Book successfully moved to bookshelf."

  Scenario: Chain-of-Thought Task Decomposition and Verification
    Given the agent's semantic dictionary contains "coffee_table" and "kitchen" spatial polygons
    When the user commands "Bring the red mug on the coffee table to the kitchen"
    Then the agent should decompose the task into a plan with sequential subtasks
    And the agent should infer that "bring to kitchen" requires finding a stable flat surface within the "kitchen" polygon
    And the agent executes the subtask to navigate to the "coffee_table"
    And when the navigation succeeds, the agent samples the camera to visually verify its location
    And the agent executes the subtask to pick up the "red mug"
    And when the grasp completes, the agent samples the camera to verify the mug is in the gripper
    And the agent updates its internal context, summarizing completed actions and retaining the plan for remaining subtasks
    And the agent executes the subtask to navigate to the "kitchen"
    And when navigation succeeds, the agent scans for a countertop and executes the placement subtask
    And the agent samples the camera to verify the mug is resting on the countertop
    Then the agent evaluates the visual state against the original goal and marks the task as "Done"
    And the action server should return success with summary "Red mug placed on kitchen counter."
