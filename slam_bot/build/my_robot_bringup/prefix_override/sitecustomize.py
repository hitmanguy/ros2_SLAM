import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/sahil/Desktop/ros_learn/slam_bot/install/my_robot_bringup'
