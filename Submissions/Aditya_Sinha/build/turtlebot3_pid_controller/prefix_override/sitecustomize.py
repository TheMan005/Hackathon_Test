import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/aditya/turtlebot3_pid_challenge/install/turtlebot3_pid_controller'
