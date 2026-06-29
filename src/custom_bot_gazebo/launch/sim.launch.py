import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_custom_bot_gazebo = get_package_share_directory('custom_bot_gazebo')
    pkg_custom_bot_description = get_package_share_directory('custom_bot_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Path to the world file inside the package
    world_path = os.path.join(pkg_custom_bot_gazebo, 'worlds', 'small_house.world')

    # Configure GZ_SIM_RESOURCE_PATH dynamically so Gazebo can find local meshes/models
    models_path = os.path.join(pkg_custom_bot_gazebo, 'models')
    
    # Append custom models directory to Gazebo Resource Path
    if 'GZ_SIM_RESOURCE_PATH' in os.environ:
        os.environ['GZ_SIM_RESOURCE_PATH'] += ':' + models_path
    else:
        os.environ['GZ_SIM_RESOURCE_PATH'] = models_path

    # Also add the worlds path so Gazebo can locate the world file
    os.environ['GZ_SIM_RESOURCE_PATH'] += ':' + os.path.join(pkg_custom_bot_gazebo, 'worlds')

    # Add all ROS2 workspaces to GZ_SIM_RESOURCE_PATH so package:// and model:// resolve correctly
    ament_prefix_path = os.environ.get('AMENT_PREFIX_PATH', '')
    for prefix in ament_prefix_path.split(':'):
        if prefix:
            os.environ['GZ_SIM_RESOURCE_PATH'] += ':' + os.path.join(prefix, 'share')

    # 1. Start Gazebo Sim (Jetty)
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r -v 4 {world_path}'}.items()
    )

    # 2. Parse XACRO and run Robot State Publisher
    xacro_file = os.path.join(pkg_custom_bot_description, 'urdf', 'robot.urdf.xacro')
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_raw,
            'use_sim_time': True
        }]
    )

    # 3. Spawn Robot in Gazebo Sim
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'custom_bot',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.2'
        ],
        output='screen'
    )

    # 4. Bridge ROS topics to Gazebo
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model'
        ],
        output='screen'
    )

    # 5. High-performance image bridge
    image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/camera/image_raw'],
        output='screen'
    )

    return LaunchDescription([
        gz_sim,
        robot_state_publisher,
        spawn_robot,
        bridge,
        image_bridge
    ])
