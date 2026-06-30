import os
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_test_description():
    pkg_share = get_package_share_directory('custom_bot_description')
    xacro_file = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')

    doc = xacro.process_file(xacro_file)
    robot_description = {'robot_description': doc.toxml()}

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description]
    )

    return launch.LaunchDescription([
        robot_state_publisher_node,
        launch_testing.actions.ReadyToTest(),
    ]), {'rsp_node': robot_state_publisher_node}


class TestURDFLaunch(unittest.TestCase):
    def test_rsp_node_running(self, proc_info, rsp_node):
        """Test that the robot_state_publisher process has started and hasn't crashed."""
        proc_info.assertWaitForStartup(process=rsp_node, timeout=10)


@launch_testing.post_shutdown_test()
class TestRSPShutdown(unittest.TestCase):
    def test_exit_code(self, proc_info, rsp_node):
        """Test that the node exited normally on shutdown."""
        launch_testing.asserts.assertExitCodes(proc_info, process=rsp_node, allowable_exit_codes=[0, -2, -15])
