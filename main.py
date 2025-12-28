from imports import *
from gui.main_window import MainWindow
from gui.output import *

if __name__ == "__main__": # Only run this block if this file is executed directly,
    app = QApplication(sys.argv) # created once for managing UI stuff
    
    window = MainWindow()
    window.show()

    
    app.exec() # Start the event loop.
    sys.exit()