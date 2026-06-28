---
name: bdd-gherkin-specs
description: Guidelines for writing Gherkin BDD specifications and implementing behave integration tests with ROS2 mock components.
---

# BDD Gherkin Testing Skill (`behave`)

This skill outlines how to perform Behavior-Driven Development for the ROS2 Lyrical Luth reasoning node using `behave`.

## 1. Scope
The BDD tests are written at the **software integration level**. We do not spin up Gazebo Jetty during these tests to ensure they are fast and reliable.
Instead, we test the core ADK reasoning node by mocking the environment:
*   Publish static test images to the mocked camera topic.
*   Mock the Nav2 and MoveIt2 Action Servers to automatically return success upon receiving the correct goals.

## 2. Directory Structure
```text
your_ros_package/
├── features/
│   ├── reasoning_pipeline.feature
│   ├── environment.py
│   └── steps/
│       └── step_definitions.py
```

## 3. Managing the ROS2 Lifecycle (`environment.py`)
`behave` does not know about ROS2. You must initialize `rclpy` and spin nodes manually in the `behave` hooks.

```python
# features/environment.py
import rclpy
import threading

def before_all(context):
    rclpy.init()
    # Initialize your mocked action servers and publishers here
    # Example: context.mock_nav2_server = MockNav2Server()
    # context.executor = rclpy.executors.MultiThreadedExecutor()
    # context.executor.add_node(context.mock_nav2_server)
    # context.spin_thread = threading.Thread(target=context.executor.spin)
    # context.spin_thread.start()

def after_all(context):
    # Shutdown nodes and executor cleanly
    # context.executor.shutdown()
    rclpy.shutdown()
    # context.spin_thread.join()
```

## 4. Writing Steps
In `step_definitions.py`, interact with the ROS2 graph:
*   **Given**: Set up the state (publish a static image to `/camera/image_raw`).
*   **When**: Send the discrete text goal to the ADK reasoning node's Action Server.
*   **Then**: Assert that the mocked Nav2 or MoveIt2 Action Server received the correct Cartesian coordinates from the reasoning node.

## 5. Gherkin Specification Best Practices & Standards
To write high-quality Gherkin specifications that are both user-readable and agent-executable:

*   **Declarative vs. Imperative Style**: Write scenarios focused on *behaviors* and *outcomes* rather than low-level implementation details.
    *   *Avoid (Imperative)*: "When I publish a Twist message with linear.x = 0.5 to /cmd_vel..."
    *   *Prefer (Declarative)*: "When the robot is commanded to navigate to the 'red cube'..."
*   **Reusable Regex Parameters**: Parameterize step definitions to promote step reuse across scenarios.
    *   Example: Use `"the camera detects a {color} cube"` instead of writing separate steps for "red cube", "blue cube", etc.
*   **Use Backgrounds for Preconditions**: If all scenarios require the same setup (e.g., initializing the action servers), define them in a `Background:` block.
*   **Scenario Outlines for Data-Driven Cases**: Use `Scenario Outline` with `Examples` to run the same behavioral test with different inputs.

### Example Gherkin Template
```gherkin
Feature: Robot Agentic Reasoning and Navigation

  Background:
    Given the ROS2 reasoning action server is online
    And the camera topic "/camera/image_raw" is active

  Scenario Outline: Navigate and pick up objects using agentic reasoning
    Given the camera detects a <object_color> <object_type> at coordinate <approx_coord>
    When the user commands "Pick up the <object_color> <object_type>"
    Then the robot should navigate to <approx_coord>
    And the manipulator should pick up the <object_color> <object_type>

    Examples:
      | object_color | object_type | approx_coord   |
      | red          | cube        | [1.2, 0.5, 0.1]|
      | blue         | cylinder    | [-0.8, 1.1, 0.2]|
```

## 6. Testing Execution & Mock Separation

### Colcon Integration
To ensure the `behave` tests run as part of the standard ROS2 `colcon test` framework, you must create a `pytest` wrapper.
Create a file at `test/test_behave.py` in your package:
```python
import subprocess
import os
from ament_index_python.packages import get_package_share_directory

def test_behave_features():
    # Find the features directory (ensure it's installed via CMakeLists/setup.py)
    pkg_dir = get_package_share_directory('your_ros_package')
    features_dir = os.path.join(pkg_dir, 'features')
    
    # Run behave
    result = subprocess.run(['behave', features_dir], capture_output=True, text=True)
    print(result.stdout)
    assert result.returncode == 0, f"Behave tests failed:\n{result.stderr}"
```
Make sure to declare `<test_depend>behave</test_depend>` and `<test_depend>pytest</test_depend>` in `package.xml`.

### Mock Separation
Mocking `rclpy.action.ActionServer` requires boilerplate. Do **not** place this boilerplate directly in `environment.py` or `step_definitions.py`. 
Create a dedicated mocks module:
```text
features/
├── mocks/
│   ├── __init__.py
│   ├── mock_nav2_server.py
│   └── mock_moveit_server.py
```
Import and instantiate these cleanly inside `environment.py`'s `before_all` hook.
