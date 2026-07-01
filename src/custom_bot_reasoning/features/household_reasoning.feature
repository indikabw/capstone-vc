Feature: Household Agentic Reasoning and Navigation

  Background:
    Given the Mock Universe node is initialized and the Nav2/MoveIt2 action servers are mocked
    And the ROS2 Lyrical Luth reasoning action server is online
    And the reasoning node is publishing to "/heartbeat" at 10Hz without blocking
    And the unified robot is localized in the AWS RoboMaker Small House using AMCL

  Scenario Outline: Semantic Exploration and Spatial Boundary Mapping
    Given the agent's semantic dictionary is empty
    When the user commands "Find the <target_room>"
    Then the agent should dispatch Nav2 NavigateToPose goals to explore map frontiers
    And when the agent reaches a frontier and the mock camera publishes the image "<detection_image>"
    Then the agent should deduce room boundaries and save the polygon as "<target_room>" in its semantic dictionary
    And the action server should return success with summary "<success_summary>"

    Examples:
      | target_room | detection_image                  | success_summary                          |
      | kitchen     | images/kitchen_appliances.jpg    | Kitchen boundaries located and saved.    |
      | living_room | images/sofa_and_tv.jpg           | Living room boundaries located and saved.|

  Scenario Outline: Fetch and Deliver Object Using Semantic Memory
    Given the agent's semantic dictionary contains "<pickup_zone>" and "<delivery_zone>" spatial polygons
    When the user commands "Pick the <item> from the <pickup_zone> and place it in the <delivery_zone>"
    Then the agent should dispatch a Nav2 goal to a valid pose within the "<pickup_zone>" polygon
    And when the Nav2 goal succeeds and the mock camera publishes the image "<pickup_image>"
    Then the agent should estimate a valid 6D grasp pose from the object geometry
    And the agent should dispatch a MoveIt2 pick goal using the estimated pose
    And when the MoveIt2 pick goal succeeds
    Then the agent should visually verify the grasp using the mock camera
    And the agent should dispatch a Nav2 goal to a valid pose within the "<delivery_zone>" polygon
    And when the Nav2 goal succeeds and the mock camera publishes the image "<delivery_surface_image>"
    Then the agent should estimate a valid 6D drop pose to place the "<item>"
    And the agent should dispatch a MoveIt2 place goal using the estimated drop pose
    And the action server should return success with summary "<success_summary>"

    Examples:
      | pickup_zone  | delivery_zone | item      | pickup_image              | delivery_surface_image       | success_summary                                |
      | coffee_table | bookshelf     | book      | images/book_on_table.jpg  | images/empty_bookshelf.jpg   | Book successfully moved to bookshelf.          |
      | kitchen_dock | dining_table  | red plate | images/plate_on_dock.jpg  | images/empty_dining_table.jpg| Red plate successfully moved to dining_table.  |

  Scenario Outline: Chain-of-Thought Task Decomposition and Verification
    Given the agent's semantic dictionary contains "<pickup_zone>" and "<delivery_zone>" spatial polygons
    When the user commands "Bring the <item> on the <pickup_zone> to the <delivery_zone>"
    Then the agent should decompose the task into a plan with sequential subtasks
    And the agent should dispatch a Nav2 goal to the "<pickup_zone>"
    And when the Nav2 goal succeeds and the mock camera publishes the image "<pickup_image>"
    Then the agent executes the subtask to estimate a grip pose and dispatch a MoveIt2 pick goal for the "<item>"
    And when the MoveIt2 pick goal succeeds and the mock camera publishes the image "<gripper_image>" to verify the grasp
    Then the agent updates its internal context and dispatches a Nav2 goal to the "<delivery_zone>"
    And when the Nav2 goal succeeds and the mock camera publishes the image "<delivery_surface_image>"
    Then the agent executes the subtask to estimate a 6D drop pose and dispatch a MoveIt2 place goal
    And when the MoveIt2 place goal succeeds and the mock camera publishes the image "<post_place_image>" to verify the placement
    Then the agent marks the task as "Done"
    And the action server should return success with summary "<success_summary>"

    Examples:
      | pickup_zone  | delivery_zone | item    | pickup_image             | gripper_image               | delivery_surface_image   | post_place_image            | success_summary                     |
      | coffee_table | kitchen       | red mug | images/mug_on_table.jpg  | images/mug_in_gripper.jpg   | images/kitchen_counter.jpg| images/mug_on_counter.jpg   | Red mug placed on kitchen counter.  |
