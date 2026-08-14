import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/faneva/PNPrepa/fosa-parc2026-aria/install/aria_vision_zed'
