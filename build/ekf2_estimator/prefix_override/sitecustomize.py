import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/pdas/UAV_NAV_ROS/install/ekf2_estimator'
