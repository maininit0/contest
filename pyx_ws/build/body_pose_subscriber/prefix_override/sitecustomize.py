import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/userdata/pyx_ws/install/body_pose_subscriber'
