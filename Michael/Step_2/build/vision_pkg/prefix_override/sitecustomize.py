import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/michael/Documents/AGV/AGV-v1/Michael/Step_2/install/vision_pkg'
