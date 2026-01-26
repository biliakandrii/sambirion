from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import TimerAction
from launch_ros.actions import SetParameter

import os

def generate_launch_description():
    """Headless launch file - no GUI (gzclient/rviz)"""
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    world_name = LaunchConfiguration('world')

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

    # --- Gazebo Server ONLY (no gzclient) ---
    gzserver_cmd = ExecuteProcess(
        cmd=['gzserver', '--verbose',
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so',
             world_path],
        output='screen'
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

    # --- Joint State Publisher ---
    jsp = Node(
        package='sambirion_application',
        executable='joint_state_publisher_node',
        name='joint_state_publisher_node'
    )

    # --- Initial Pose Publisher (with delay) ---
    initial_pose_publisher = Node(
        package='sambirion_application',
        executable='initial_pose_pub',
        name='initial_pose_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'initial_x': 0.0,
            'initial_y': 0.0,
            'initial_yaw': 0.0
        }]
    )

    initial_pose_timer = TimerAction(
        period=5.0,
        actions=[initial_pose_publisher]
    )

    # trajectory_plotter = Node(
    #     package='sambirion_application',
    #     executable='trajectory_plotter.py',
    #     name='trajectory_plotter',
    #     output='screen',
    #     parameters=[{
    #         'use_sim_time': True,
    #         'topic_name': '/odom',
    #         'max_points': 1000,
    #         'map_size': 10.0,
    #         'output_dir': '/root/sambirion/plots/trajectory/'
    #     }]
    # )
    
    set_sim_time = SetParameter(name='use_sim_time', value=True)

    return LaunchDescription([
        set_sim_time,
        declare_use_sim_time,
        declare_world,
        gzserver_cmd,  
        robot_state_publisher,
        spawn_entity,
        jsp,
        initial_pose_timer,
        # trajectory_plotter,
    ])