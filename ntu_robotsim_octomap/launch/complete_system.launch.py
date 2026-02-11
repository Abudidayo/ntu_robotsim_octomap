import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # Setup Package Directories
    pkg_octo_map = get_package_share_directory('ntu_robotsim_octomap')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    pkg_odom_tf = get_package_share_directory('odom_to_tf_ros2')
    pkg_octomap_server = get_package_share_directory('octomap_server2')

    # 1. Launch the Maze Environment (Gazebo)
    launch_maze = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_octo_map, 'launch', 'maze.launch.py'))
    )

    # 2. Launch the Robot and Bridge
    launch_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_octo_map, 'launch', 'single_robot_sim.launch.py'))
    )

    # 3. Launch TF Publishers (Odom to TF and Static Transform)
    launch_odom_tf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_odom_tf, 'launch', 'odom_to_tf.launch.py'))
    )

    # 4. Launch OctoMap Server with the filtered point cloud from the camera
    launch_octomap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_octomap_server, 'launch', 'octomap_filtered.launch.py'))
    )

    # 5. Launch RViz with a pre-configured view of the octomap and robot state
    rviz_config_path = os.path.join(pkg_octo_map, 'config', 'config.rviz')

    launch_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    # 6. Launch Nav2 after a delay to ensure the octomap server and TFs are up and running

    # Nav2 params file path
    nav2_params_path = os.path.join(pkg_octo_map, 'config', 'nav2_octomap_params.yaml')

    launch_nav2 = TimerAction(period=10.0, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')),
                launch_arguments={
                    'params_file': nav2_params_path,
                    'use_sim_time': 'true',
                    'use_rviz': 'false'
                }.items()
            )
        ]
    )

    # Static transform from map to odom frame, since the robot is not moving in the world, we can set the transform to be static
    static_map_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )

    return LaunchDescription([
        launch_maze,
        launch_robot,
        launch_odom_tf,
        launch_octomap,
        launch_rviz,
        launch_nav2,
        static_map_tf
    ])