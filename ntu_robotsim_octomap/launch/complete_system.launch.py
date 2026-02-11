import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    # Package Directories
    ntu_pkg_dir = get_package_share_directory('ntu_robotsim_octomap')
    odom_tf_pkg_dir = get_package_share_directory('odom_to_tf_ros2')
    nav2_pkg_dir = get_package_share_directory('nav2_bringup')

    # Launch Arguments
    use_rviz = LaunchConfiguration('rviz')
    use_teleop = LaunchConfiguration('teleop')

    # 1. Launch the Maze Environment (Gazebo)
    maze_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ntu_pkg_dir, 'launch', 'maze.launch.py')
        )
    )

    # 2. Launch the Robot and Bridge
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ntu_pkg_dir, 'launch', 'single_robot_sim.launch.py')
        )
    )

    # 3. Launch TF Publishers (Odom to TF and Static Transform)
    odom_tf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(odom_tf_pkg_dir, 'launch', 'odom_to_tf.launch.py')
        )
    )

    # 4. Launch OctoMap Server
    octomap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ntu_pkg_dir, 'launch', 'octomap_filtered.launch.py')
        )
    )

    # 5. Launch Nav2
    nav2_params_file = os.path.join(ntu_pkg_dir, 'config', 'nav2_octomap_params.yaml')
    
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_pkg_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'params_file': nav2_params_file,
            'use_sim_time': 'true'
        }.items()
    )

    # 6. Launch RViz
    rviz_cmd = ExecuteProcess(
        condition=IfCondition(use_rviz),
        cmd=['ros2', 'run', 'rviz2', 'rviz2'],
        output='screen'
    )

    # 7. Launch Teleop (Note: captures keyboard in the terminal where launch is run)
    teleop_cmd = ExecuteProcess(
        condition=IfCondition(use_teleop),
        cmd=['ros2', 'run', 'teleop_twist_keyboard', 'teleop_twist_keyboard'],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true', description='Open RViz'),
        DeclareLaunchArgument('teleop', default_value='true', description='Open Teleop'),
        maze_launch,
        robot_launch,
        odom_tf_launch,
        octomap_launch,
        nav2_launch,
        rviz_cmd,
        teleop_cmd
    ])