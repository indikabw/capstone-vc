import os
import xacro
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    pkg_custom_bot_gazebo = get_package_share_directory('custom_bot_gazebo')
    pkg_custom_bot_description = get_package_share_directory('custom_bot_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='single_room.world',
        description='World file to load'
    )

    world = LaunchConfiguration('world')
    world_path = PathJoinSubstitution([pkg_custom_bot_gazebo, 'worlds', world])

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

    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='true',
        description='Whether to run the Gazebo GUI client'
    )

    headless = LaunchConfiguration('headless')

    # 1. Start Gazebo Sim (Jetty) Server explicitly
    gz_sim_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': [
            '-r -s -v 4 ',
            world_path
        ]}.items()
    )

    # Start Gazebo Sim GUI (Client) explicitly if headless is false
    gz_sim_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-g -v 4 ']}.items(),
        condition=launch.conditions.UnlessCondition(headless)
    )

    # 2. Parse XACRO and run Robot State Publisher
    xacro_file = os.path.join(pkg_custom_bot_description, 'urdf', 'robot.urdf.xacro')
    robot_description_raw = xacro.process_file(xacro_file, mappings={'gazebo': 'ignition'}).toxml()

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
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # 4. Bridge ROS topics to Gazebo (except scan)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/model/custom_bot/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model'
        ],
        remappings=[
            ('/cmd_vel', '/cmd_vel_unstamped'),
            ('/model/custom_bot/tf', '/tf')
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )



    # 5a. Bridge only the fixed overhead destination camera (needed for recording).
    # The robot's RGBD camera is NOT bridged here to avoid the extra llvmpipe render cost.
    # Re-enable destination_camera_bridge to record the overhead view.
    destination_camera_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/destination_camera/image_raw'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # 5b. (Optional) Robot RGBD camera bridge — disabled for headless nav tests to save CPU.
    # Uncomment to enable /camera/image_raw for vision agent nodes.
    # robot_camera_bridge = Node(
    #     package='ros_gz_image',
    #     executable='image_bridge',
    #     arguments=['/world/single_room/model/custom_bot/link/oakd_rgb_camera_frame/sensor/rgbd_camera/image'],
    #     remappings=[
    #         ('/world/single_room/model/custom_bot/link/oakd_rgb_camera_frame/sensor/rgbd_camera/image', '/camera/image_raw')
    #     ],
    #     parameters=[{'use_sim_time': True}],
    #     output='screen'
    # )

    twist_converter_node = Node(
        package='custom_bot_navigation',
        executable='twist_stamped_to_twist.py',
        name='twist_stamped_to_twist',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # 7. Dynamic RPLidar TF Bridge (bypasses static cache loss)
    dynamic_rplidar_tf_node = Node(
        package='custom_bot_gazebo',
        executable='dynamic_rplidar_tf_bridge.py',
        name='dynamic_rplidar_tf_bridge',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # 8. TF Static Republisher (republishes all static TFs as dynamic)
    tf_static_republisher_node = Node(
        package='custom_bot_gazebo',
        executable='tf_static_republisher.py',
        name='tf_static_republisher',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        world_arg,
        headless_arg,
        gz_sim_server,
        gz_sim_gui,
        robot_state_publisher,
        spawn_robot,
        bridge,
        destination_camera_bridge,
        twist_converter_node,
        dynamic_rplidar_tf_node,
        tf_static_republisher_node
    ])
