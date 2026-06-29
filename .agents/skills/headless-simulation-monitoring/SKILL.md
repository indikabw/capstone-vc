---
name: headless-simulation-monitoring
description: Technical guidelines, recipes, and troubleshooting steps for running Gazebo Jetty simulations headlessly and monitoring robot kinematics, joint velocities, and clock synchronization.
---

# Headless Simulation & Monitoring Skill

This skill defines the technical approaches and diagnostic steps required to execute Gazebo Jetty simulations headlessly, configure software rendering, synchronize simulation clocks, and programmatically verify joint actuation.

---

## 1. Running Gazebo Jetty Headlessly
When running simulations in headless virtual machines, you must bypass hardware OpenGL calls and force software rendering using Mesa's `llvmpipe` backend.

**Execution Command**:
```bash
export LIBGL_ALWAYS_SOFTWARE=1
export GALLIUM_DRIVER=llvmpipe
ros2 launch custom_bot_gazebo sim.launch.py headless:=true
```

---

## 2. Gravity and Physics Engine Checks
A common pitfall is spawning a robot in a custom world that lacks gravity or physics stepping properties, which results in the robot floating in mid-air and the physics solver failing to actuate wheel joints.

### Verifying Robot Position and Pose
You can query the current 3D coordinates of a model directly from Gazebo Transport topics without a GUI:
```bash
# Echo the pose info of the model to check if it has dropped to the ground (Z close to 0)
gz topic -e -t /world/single_room/pose/info -n 1 | grep -A 10 'name: "custom_bot"'
```

### Necessary World XML Elements
Ensure the `.world` SDF file contains a `<gravity>` vector and a configured `<physics>` step size:
```xml
<gravity>0 0 -9.8</gravity>
<physics name='default_physics' default='0' type='ode'>
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1</real_time_factor>
  <real_time_update_rate>1000</real_time_update_rate>
</physics>
```

---

## 3. Clock and Time Synchronization
If `use_sim_time: True` is not set consistently, nodes will default to system time, causing the controller manager to discard velocity commands as stale.

Ensure the `use_sim_time` parameter is set to `True` for:
1. **Controller Manager**: Set under the root elements in `control.yaml`.
2. **Bridge Nodes**: Set as parameters for `ros_gz_bridge` and `image_bridge` in the launch script.
3. **Control Spawners**: Passed to spawner nodes during execution.

---

## 4. Topic Interface & Type Matching
The standard Create 3 safety stack (`motion_control` node) enforces strict type rules:
* **`/cmd_vel`**: Subscribed to by `motion_control` as `geometry_msgs/msg/TwistStamped`.
* **`/cmd_vel_unstamped`**: Subscribed to by `motion_control` as `geometry_msgs/msg/Twist`.

**Crucial Rule**: Standard teleop or manual tools sending unstamped `Twist` messages (without a header) must publish to **`/cmd_vel_unstamped`** to avoid a silent ROS 2 type mismatch.

---

## 5. Movement Verification Script Template
Use the following Python template node to verify movement. It runs entirely on the simulation clock and monitors joint velocities:

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
import sys

class MovementVerifier(Node):
    def __init__(self):
        super().__init__('movement_verifier')
        self.pub = self.create_publisher(Twist, '/cmd_vel_unstamped', 10)
        self.sub = self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.sim_start_time = None
        self.msg_count = 0

    def timer_callback(self):
        current_sim_time = self.get_clock().now().nanoseconds / 1e9
        if current_sim_time == 0.0:
            return  # Wait for clock to start
            
        if self.sim_start_time is None:
            self.sim_start_time = current_sim_time
            print(f"Simulation clock started at: {self.sim_start_time:.3f}")

        msg = Twist()
        msg.linear.x = 0.5
        self.pub.publish(msg)
        self.msg_count += 1

        if current_sim_time - self.sim_start_time > 8.0:
            print(f"Finished verification. Published {self.msg_count} messages.")
            sys.exit(0)

    def joint_callback(self, msg):
        for i, name in enumerate(msg.name):
            if name == 'left_wheel_joint':
                print(f"SimTime: {msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d} | Left Wheel Pos: {msg.position[i]:.6f} | Vel: {msg.velocity[i]:.6f}")

def main():
    rclpy.init()
    node = MovementVerifier()
    node.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```
