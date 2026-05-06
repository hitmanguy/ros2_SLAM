from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch.substitutions import PathJoinSubstitution, FileContent
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    urdf = FileContent(
        PathJoinSubstitution([
            FindPackageShare('my_robot_description'),
            'urdf',
            'my_robot.urdf.xml'
        ])
    )

    world = PathJoinSubstitution([
        FindPackageShare('my_robot_description'),
        'worlds',
        'empty.sdf'
    ])

    ekf_config = PathJoinSubstitution([
        FindPackageShare('my_robot_bringup'),
        "config",
        "ekf.yaml"
    ])

    slam_params = PathJoinSubstitution([
        FindPackageShare('my_robot_bringup'),
        'config',
        'slam_toolbox_params.yaml'
    ])

    nav2_params = PathJoinSubstitution([
        FindPackageShare('my_robot_bringup'),
        'config',
        'nav2_params.yaml'
    ])

    rviz_config = PathJoinSubstitution([
        FindPackageShare('nav2_bringup'),
        'rviz',
        'nav2_default_view.rviz'
    ])

    return LaunchDescription([

        # 1. Start Gazebo
        ExecuteProcess(
            cmd=['ign', 'gazebo', world, '-r'],
            output='screen'
        ),

        # 2. Publish robot URDF transforms to ROS 2
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': ParameterValue(urdf, value_type=str)}],
            output='screen'
        ),

        # Robot Localization EKF Node
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            output="screen",
            parameters=[ekf_config, {"use_sim_time": True}],
        ),

        # 3. Spawn robot in Gazebo (delayed to let Gazebo finish loading)
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='ros_gz_sim',
                    executable='create',
                    arguments=[
                        '-name', 'my_robot',
                        '-topic', 'robot_description'
                    ],
                    output='screen'
                )
            ]
        ),

        # 4. ROS <-> Gazebo topic bridge
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
                '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
                '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
                '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
                '/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
                '/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
                '/model/my_robot/pose@geometry_msgs/msg/PoseStamped[ignition.msgs.Pose',
                '/camera/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image',
                '/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo'
            ],
            output='screen'
        ),

        # Static TF — separate node with TRANSIENT_LOCAL QoS ← fixes the warning
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/tf_static@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            ],
            ros_arguments=[
                '--ros-args',
                '--remap', '/tf_static:=/tf_static',
                '-p', 'qos_overrides./tf_static.publisher.durability:=transient_local',
            ],
            output='screen'
        ),

        TimerAction(
            period=6.0,
            actions=[
                Node(
                    package='slam_toolbox',
                    executable='async_slam_toolbox_node',
                    name='slam_toolbox',
                    parameters=[slam_params],
                    output='screen',
                )
            ]
        ),

        TimerAction(
            period=10.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        FindPackageShare('nav2_bringup'),
                        '/launch/navigation_launch.py'
                    ]),
                    launch_arguments={
                        'use_sim_time': 'true',
                        'params_file': nav2_params,
                        # SLAM publishes the map — no need for map_server or AMCL
                        'slam': 'False',
                        'use_composition': 'False',
                    }.items(),
                )
            ]
        ),

        # 9. RViz2 with the standard Nav2 layout (2D Nav Goal tool included)
        TimerAction(
            period=6.0,
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    arguments=['-d', rviz_config],
                    parameters=[{'use_sim_time': True}],
                    output='screen',
                )
            ]
        ),
    ])