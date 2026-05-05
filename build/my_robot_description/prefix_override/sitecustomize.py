import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/sahil/Desktop/ros_learn/ros2_SLAM/install/my_robot_description'
