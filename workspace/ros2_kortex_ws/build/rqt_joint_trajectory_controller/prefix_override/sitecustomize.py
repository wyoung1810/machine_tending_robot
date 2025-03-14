import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/mampane/Desktop/machine_tending_robot/workspace/ros2_kortex_ws/install/rqt_joint_trajectory_controller'
