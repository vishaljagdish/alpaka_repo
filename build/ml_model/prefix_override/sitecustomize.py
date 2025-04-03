import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/alpaka-admin/my_alpaka_urdf_ws/install/ml_model'
