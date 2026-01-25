import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/endure/TA/yahboomcar_ws/install/yahboomcar_rviz'
