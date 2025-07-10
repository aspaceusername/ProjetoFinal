import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ruben/flask_nav2_web/nav_goal_sender/install/nav_goal_sender'
