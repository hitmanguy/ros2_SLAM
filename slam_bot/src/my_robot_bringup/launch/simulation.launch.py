from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch.substitutions import PathJoinSubstitution, FileContent
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

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

        # Start Gazebo
        ExecuteProcess(
            cmd=['ign', 'gazebo', world, '-r'],
            output='screen'
        ),

        # Publish robot state to ROS
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': urdf}],
            output='screen'
        ),

        # Spawn robot in Gazebo (delayed to let Gazebo load)
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
        # THE BRIDGE
        Node(
            package='ros_gz_bridge', # Note: on some older Humble installs this is 'ros_ign_bridge'
            executable='parameter_bridge',
            arguments=[
                '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
                '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
                '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
                '/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU'
            ],
            output='screen'
        ),
        
    ])