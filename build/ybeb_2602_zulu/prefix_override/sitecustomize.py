import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/rpi5master/ros2_ws_2602/install/ybeb_2602_zulu'
