from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch.substitutions import PathJoinSubstitution, FileContent
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


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
                '/model/my_robot/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
                '/model/my_robot/pose@geometry_msgs/msg/PoseStamped[ignition.msgs.Pose',
                '/camera/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image',
                '/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo'
            ],
            remappings=[
                ('/model/my_robot/tf', '/tf'),
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

    ])