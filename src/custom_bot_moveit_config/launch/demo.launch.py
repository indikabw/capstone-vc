import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    pkg_custom_bot_moveit_config = get_package_share_directory('custom_bot_moveit_config')
    
    # Spawn the arm_controller
    spawn_arm_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'arm_controller', os.path.join(get_package_share_directory('custom_bot_description'), 'config', 'ros2_controllers.yaml')],
        output='screen'
    )
    
    # Spawn the gripper_controller
    spawn_gripper_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'gripper_controller', os.path.join(get_package_share_directory('custom_bot_description'), 'config', 'ros2_controllers.yaml')],
        output='screen'
    )

    # Spawn joint_state_broadcaster if not already done by something else
    spawn_jsb = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen'
    )

    # MoveIt MoveGroup node
    # Since we are not using the full setup assistant, we might just load move_group with minimal params
    # We will need the URDF and SRDF on the parameter server, which is usually handled by MoveGroup
    # For now, we assume the user will launch MoveIt after sim.
    # Note: Full MoveGroup configuration requires loading kinematics, limits, and controllers into the node's parameters.
    # Here we outline the basic MoveGroup node execution.
    
    return LaunchDescription([
        spawn_jsb,
        spawn_arm_controller,
        spawn_gripper_controller
    ])
