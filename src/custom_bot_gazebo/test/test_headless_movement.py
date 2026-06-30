import os
import unittest
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

import launch
import launch_testing
import launch_testing.actions
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_test_description():
    pkg_share = get_package_share_directory('custom_bot_gazebo')
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'sim.launch.py')),
        launch_arguments={'headless': 'true'}.items()
    )

    # Force software rendering for headless CI environments
    os.environ['LIBGL_ALWAYS_SOFTWARE'] = '1'
    os.environ['GALLIUM_DRIVER'] = 'llvmpipe'

    return launch.LaunchDescription([
        sim_launch,
        launch_testing.actions.ReadyToTest(),
    ])

class TestHeadlessMovement(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_movement(self):
        node = rclpy.create_node('movement_verifier')
        node.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        
        pub = node.create_publisher(Twist, '/cmd_vel_unstamped', 10)
        
        movement_detected = False
        
        def joint_callback(msg):
            nonlocal movement_detected
            for i, name in enumerate(msg.name):
                # Ensure the wheel joints actuate
                if name == 'left_wheel_joint':
                    if abs(msg.velocity[i]) > 0.05:
                        movement_detected = True

        sub = node.create_subscription(JointState, '/joint_states', joint_callback, 10)
        
        # Wait up to 15 seconds real time for the sim to boot and the robot to move
        import time
        start_time = time.time()
        
        while rclpy.ok() and time.time() - start_time < 15.0 and not movement_detected:
            msg = Twist()
            msg.linear.x = 0.5
            pub.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.1)
            
        node.destroy_node()
        
        self.assertTrue(movement_detected, "Robot joints did not actuate in headless simulation.")
