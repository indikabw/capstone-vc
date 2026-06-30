import os
import unittest
import rclpy
from rclpy.node import Node
import time

import launch
import launch_testing
import launch_testing.actions
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_test_description():
    pkg_share = get_package_share_directory('custom_bot_navigation')
    map_yaml_file = os.path.join(pkg_share, 'maps', 'aws_small_house.yaml')

    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'navigation.launch.py')),
        launch_arguments={'map': map_yaml_file, 'use_sim_time': 'false'}.items()
    )

    return launch.LaunchDescription([
        nav_launch,
        launch_testing.actions.ReadyToTest(),
    ])

class TestNav2Lifecycle(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_nav2_active(self):
        node = rclpy.create_node('nav2_lifecycle_tester')
        
        start_time = time.time()
        nav2_nodes_found = False
        
        # Wait up to 20 seconds for the nav2 stack to come up
        while rclpy.ok() and time.time() - start_time < 20.0:
            node_names = node.get_node_names()
            if 'amcl' in node_names and 'planner_server' in node_names and 'controller_server' in node_names:
                nav2_nodes_found = True
                break
            rclpy.spin_once(node, timeout_sec=0.5)
            
        node.destroy_node()
        self.assertTrue(nav2_nodes_found, "Nav2 nodes (amcl, planner_server, controller_server) did not start within 20 seconds.")
