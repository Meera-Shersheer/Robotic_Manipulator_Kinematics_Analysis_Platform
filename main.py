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