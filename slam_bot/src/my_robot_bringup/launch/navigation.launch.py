from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ── Launch arguments ──────────────────────────────────────────────────────
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    slam_params = PathJoinSubstitution([
        FindPackageShare('my_robot_bringup'),
        'config',
        'slam_toolbox_params.yaml',
    ])

    nav2_params = PathJoinSubstitution([
        FindPackageShare('my_robot_bringup'),
        'config',
        'nav2_params.yaml',
    ])

    nav2_bringup_dir = FindPackageShare('nav2_bringup')

    # ── SLAM Toolbox (async online mapping) ───────────────────────────────────
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[slam_params, {'use_sim_time': use_sim_time}],
        output='screen',
    )

    # ── Nav2 bringup (navigation stack, no AMCL – SLAM provides the map→odom TF)
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            nav2_bringup_dir, '/launch/navigation_launch.py'
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params,
            # SLAM already publishes the map, so we skip map_server/AMCL
            'slam': 'False',
            'use_composition': 'False',
        }.items(),
    )

    # ── RViz2 (pre-configured for SLAM + Nav2) ────────────────────────────────
    rviz_config = PathJoinSubstitution([
        nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz'
    ])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true',
        ),
        slam_toolbox_node,
        nav2_launch,
        rviz_node,
    ])