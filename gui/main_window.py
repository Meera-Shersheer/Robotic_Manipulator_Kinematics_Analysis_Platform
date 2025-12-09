from imports import *

class MainWindow(QWidget): #defining our class (inheriting from QWidget)
    def __init__(self):    #Constructor
        super().__init__()  # calling the parent constructor
        self.resize(1400, 1000) # resizing the window

        self.setWindowTitle("Robotics IK/FK Calculator")   # giving a title to the window 

        layout = QVBoxLayout()

        title = QLabel("Robotics IK/FK Platform")  # QLabel creates text that can apear in the UI
        title.setStyleSheet("font-size:28px; font-weight:bold; font-family:Times New Roman; ") # styling the title
        title.setAlignment(Qt.AlignmentFlag.AlignHCenter)
        layout.addWidget(title) # add title to the window

        # Add placeholders
        layout.addWidget(QLabel("Robot Selection Panel"))
        layout.addWidget(QLabel("DH Parameter Panel"))
        layout.addWidget(QLabel("FK/IK Outputs Panel"))
        layout.addWidget(QLabel("3D Visualization Panel"))

        self.setLayout(layout) # Assigns the layout to the window