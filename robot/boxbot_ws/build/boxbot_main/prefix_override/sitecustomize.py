import sys
if sys.prefix == '/home/boxbot/venv':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/boxbot/boxbot_ws/install/boxbot_main'
