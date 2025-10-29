import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/sanjay/ros2_workspaces/rcup_migration/src/rcup_garden/install/rcup_garden'
