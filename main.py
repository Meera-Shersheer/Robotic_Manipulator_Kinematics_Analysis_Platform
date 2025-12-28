import os
os.environ['PYOPENGL_PLATFORM'] = 'egl'  # or 'glx' on Linux
os.environ['__NV_PRIME_RENDER_OFFLOAD'] = '1'
os.environ['__GLX_VENDOR_LIBRARY_NAME'] = 'nvidia'
# For PyQt to prefer discrete GPU
os.environ['QT_OPENGL'] = 'desktop'

from imports import *
from gui.main_window import MainWindow
from gui.test_layout import TestMainWindow
from gui.output import *

if __name__ == "__main__": # Only run this block if this file is executed directly,
    app = QApplication(sys.argv) # created once for managing UI stuff
    #app.setStyle('Fusion')
    #window = TestMainWindow() #To test Layout and areas
    window = MainWindow()
    window.show()

    
    app.exec() # Start the event loop.
    sys.exit()