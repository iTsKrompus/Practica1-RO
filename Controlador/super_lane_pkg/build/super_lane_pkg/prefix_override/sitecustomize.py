import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/hugo/ros2_ws/src/super_lane_pkg/install/super_lane_pkg'
