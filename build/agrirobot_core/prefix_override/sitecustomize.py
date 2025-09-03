import sys
if sys.prefix == '/home/prukubt/venv':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/prukubt/agrirobot_ws/install/agrirobot_core'
