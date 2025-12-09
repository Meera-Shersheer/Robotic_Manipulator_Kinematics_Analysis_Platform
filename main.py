from imports import *
from gui.main_window import MainWindow


if __name__ == "__main__": # Only run this block if this file is executed directly,
    app = QApplication(sys.argv) # created once for managing UI stuff
#    app.setStyle('Fusion')
    window = MainWindow()
    window.show()
    sys.exit(app.exec())