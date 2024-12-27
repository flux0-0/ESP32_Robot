import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/bao/my_tf_broadcaster/install/my_tf_broadcaster'
