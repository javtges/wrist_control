import sys
if sys.prefix == '/home/james/wrist_26/wrist-odrive':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/james/wrist_26/odrive_ros_ws/install/wrist_control'
