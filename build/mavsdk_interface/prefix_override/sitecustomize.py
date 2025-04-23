import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/jazz/Projects/swarm_drone_ws/install/mavsdk_interface'
