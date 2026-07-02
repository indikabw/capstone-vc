import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    pkg_custom_bot_description = get_package_share_directory('custom_bot_description')
    urdf_path = os.path.join(pkg_custom_bot_description, 'urdf', 'robot.urdf.xacro')

    moveit_config = (
        MoveItConfigsBuilder("custom_bot", package_name="custom_bot_moveit_config")
        .robot_description(file_path=urdf_path, mappings={"gazebo": "ignition"})
        .robot_description_semantic(file_path="config/custom_bot.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
        .to_moveit_configs()
    )

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            {'use_sim_time': True},
            {'publish_robot_description_semantic': True}
        ]
    )

    return LaunchDescription([move_group_node])
