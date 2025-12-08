from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import TimerAction
from launch_ros.actions import SetParameter

import os

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    world_name = LaunchConfiguration('world')

    pkg_project_bringup = get_package_share_directory('sambirion_bringup')
    pkg_project_gazebo = get_package_share_directory('sambirion_gazebo')
    pkg_project_description = get_package_share_directory('sambirion_description')

    sdf_file = os.path.join(pkg_project_description, 'models', 'sambirion', 'model.sdf')
    urdf_file = os.path.join(pkg_project_description, 'models', 'sambirion', 'model.urdf')
    world_path = PathJoinSubstitution([pkg_project_gazebo, 'worlds', world_name])

    # --- Launch Arguments ---
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='True',
        description='Use simulation (Gazebo) clock')
    declare_world = DeclareLaunchArgument(
        'world', default_value='world.sdf',
        description='World file to load')

    # --- Gazebo ---
    gzserver_cmd = ExecuteProcess(
        cmd=['gzserver', '--verbose',
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so',
             world_path],
        output='screen'
    )
    gzclient_cmd = ExecuteProcess(
        cmd=['gzclient'], output='screen'
    )

    # --- Robot State Publisher ---
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': open(urdf_file).read()
        }]
    )

    # --- Spawn Robot ---
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-file', sdf_file,
            '-entity', 'sambirion',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.0'
        ],
        output='screen'
    )

    # --- Map Server (Lifecycle Node) ---
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': '/root/sambirion/src/sambirion_navigation/maps/map.yaml',
            'use_sim_time': True
        }]
    )

    # --- AMCL (Lifecycle Node) ---
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[{
            'use_map_topic': True,
            'odom_frame_id': 'odom',
            'base_frame_id': 'base_link',
            'global_frame_id': 'map',
            'scan_topic': 'front_scan',
            'use_sim_time': True
        }]
    )

    # --- Lifecycle Manager ---
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['map_server', 'amcl']
        }]
    )

    # --- RViz and Joint State Publisher ---
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    jsp = Node(
        package='sambirion_application',
        executable='joint_state_publisher_node',
        name='joint_state_publisher_node'
    )

     # --- Timer Node to Publish Initial Pose ---
    initial_pose_publisher = Node(
        package='sambirion_application',  # or any custom package you have
        executable='initial_pose_pub',    # new Python script we will create
        name='initial_pose_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'initial_x': 0.0,
            'initial_y': 0.0,
            'initial_yaw': 0.0
        }]
    )

    # Wrap it in a short TimerAction to give map_server and AMCL time to activate
    initial_pose_timer = TimerAction(
        period=5.0,  # 2 seconds delay
        actions=[initial_pose_publisher]
    )
    set_sim_time = SetParameter(name='use_sim_time', value=True)

    goal_pub = Node(
        package='sambirion_application',
        executable='goal_publisher.py',
        name='goal_publisher'
    )
    return LaunchDescription([
        set_sim_time,
        declare_use_sim_time,
        declare_world,
        gzserver_cmd,
        gzclient_cmd,
        initial_pose_timer,
        robot_state_publisher,
        spawn_entity,
        map_server,
        amcl,
        lifecycle_manager,
        rviz,
        jsp,
        goal_pub,
    ])
