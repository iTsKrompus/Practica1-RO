import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/hugo/Documentos/RO/Practica1-RO/install/signal_det_pkg'
