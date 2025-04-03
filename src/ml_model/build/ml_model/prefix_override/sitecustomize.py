import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/alpaka-admin/ros2_ws/src/ml_model/install/ml_model'
