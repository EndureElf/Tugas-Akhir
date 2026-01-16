import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/endure/Robot 3/install/laserscan_to_point_publisher'
