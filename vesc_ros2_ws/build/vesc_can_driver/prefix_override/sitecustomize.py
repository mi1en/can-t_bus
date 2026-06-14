import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/mi1ena/can-t_bus/vesc_ros2_ws/install/vesc_can_driver'
