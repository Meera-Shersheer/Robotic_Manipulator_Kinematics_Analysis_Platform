from imports import *

class Color(QWidget):
    def __init__(self, color, text=None):
        super().__init__()

        self.setAutoFillBackground(True)
        self.layout = QVBoxLayout(self) 
        self.layout.setContentsMargins(8, 8, 8, 8) 
        self.layout.setSpacing(0)
        if text:
            label = QLabel(text)
            label.setAlignment(Qt.AlignmentFlag.AlignCenter)
            label.setStyleSheet("font-weight: bold; color: white;")
            self.layout.addWidget(label)
       
        # Set border to clearly see divisions
        self.setStyleSheet(f"""
            background-color: {color};
            border: 2px solid black;
        """)  

class TestMainWindow(QMainWindow): #defining our class (inheriting from QMainWindow)
    def __init__(self):    #Constructor
        super().__init__()  # calling the parent constructor

        self.setWindowTitle("Robotics IK/FK Calculator")   # giving a title to the window 
        self.resize(1400, 1000) # resizing the window
        
        central = Color("yellow") # Create a central widget (required in QMainWindow)
        self.setCentralWidget(central)

        outer_layout = central.layout

        # Title
        title = QLabel("Robotics IK/FK Platform")
        title.setStyleSheet("font-size:28px; font-weight:bold; font-family:Times New Roman;")
        title.setAlignment(Qt.AlignmentFlag.AlignHCenter)
        
        outer_layout.addWidget(title)

        main_layout = QHBoxLayout()
        main_layout.setSpacing(0)
        main_layout.setContentsMargins(0, 0, 0, 0)
        outer_layout.addLayout(main_layout)
        
        # ================= LEFT COLUMN =================
        left_widget = QVBoxLayout()
        left_widget.setSpacing(0)
        left_widget.setContentsMargins(0, 0, 0, 0) 
        main_layout.addLayout(left_widget, 2)
        
        inputs_section = QHBoxLayout()
        inputs_section.setSpacing(0)
        inputs_section.setContentsMargins(0, 0, 0, 0)
        left_widget.addLayout(inputs_section, 8)

        
        # Row 1, coulmn 1 : Controls
        controls_widget = QVBoxLayout()
        controls_widget.setSpacing(0)
        controls_widget.setContentsMargins(0, 0, 0, 0) 
        inputs_section.addLayout(controls_widget, 1)
        minpulator_list = Color("#e747c5", "minpulator list")
        ik_fk = Color("#7010ed", "IK OR FK")
        theta_system  = Color("#6f95f5", "Theta's unit")
        sym_num = Color("#0b3bb4", "symbolic or not")
   
       
        controls_widget.addWidget(minpulator_list)
        controls_widget.addWidget(ik_fk)
        controls_widget.addWidget(sym_num)
        controls_widget.addWidget(theta_system)
        
        # Row 1, coulmn 2 :DH Table
        dh_widget = Color("blue", "dh_widget")
        
        # Row 3: Outputs
        output_widget = Color("orange", "OUTPUT")
        execute_widget = Color("purple", "execute")
        
        inputs_section.addWidget(dh_widget, 2)  
        left_widget.addWidget(execute_widget, 1)
        left_widget.addWidget(output_widget, 6)
        
        # ================= RIGHT COLUMN =================
        right_widget = QVBoxLayout()
        right_widget.setSpacing(0)
        right_widget.setContentsMargins(0, 0, 0, 0)
        main_layout.addLayout(right_widget, 1)
   
        view3d_widget = Color("#1cccec", "3D VIEW") 
        choose_2D_sec = Color("#ec1c31", "2D_section") 
        view2d_widget = Color("#f7838f", "2D VIEW")
             
        right_widget.addWidget(view3d_widget, 8)
        right_widget.addWidget(choose_2D_sec, 1)
        right_widget.addWidget(view2d_widget, 8)




