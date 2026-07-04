import re

with open("/Users/indikawijayasinghe/GIT/capstone-vc/src/custom_bot_reasoning/custom_bot_reasoning/reasoning_node.py", "r") as f:
    code = f.read()

# Add imports
if "from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint" not in code:
    code = code.replace("from geometry_msgs.msg import TransformStamped", "from geometry_msgs.msg import TransformStamped\nfrom trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint\nfrom builtin_interfaces.msg import Duration")

# Add publisher
if "self.gripper_pub =" not in code:
    code = code.replace("self._moveit_client = ActionClient(self, MoveGroup, 'move_action')", "self._moveit_client = ActionClient(self, MoveGroup, 'move_action')\n        self.gripper_pub = self.create_publisher(JointTrajectory, '/gripper_controller/joint_trajectory', 10)")

# Add control_gripper method
gripper_method = """
    def control_gripper(self, positions):
        msg = JointTrajectory()
        msg.joint_names = ['omx_gripper_left_joint']
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(sec=1, nanosec=0)
        msg.points.append(point)
        self.gripper_pub.publish(msg)
        time.sleep(1.0)
"""
if "def control_gripper" not in code:
    code = code.replace("    def execute_moveit_joints(self", gripper_method + "\n    def execute_moveit_joints(self")

# Replace execute_moveit_joints for gripper
code = code.replace("self.execute_moveit_joints('gripper', gripper_joints, [0.019])\n        time.sleep(1.0)", "self.control_gripper([0.019])")
code = code.replace("self.execute_moveit_joints('gripper', gripper_joints, [-0.010])\n        time.sleep(1.0)", "self.control_gripper([-0.010])")

with open("/Users/indikawijayasinghe/GIT/capstone-vc/src/custom_bot_reasoning/custom_bot_reasoning/reasoning_node.py", "w") as f:
    f.write(code)

