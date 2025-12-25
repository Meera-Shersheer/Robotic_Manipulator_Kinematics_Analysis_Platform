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
        self.resize(1840, 1100) # resizing the window
        
        central = Color("yellow") # Create a central widget (required in QMainWindow)
        self.setCentralWidget(central)

        outer_layout = central.layout

        # Title
        title = QLabel("Robotics IK/FK Platform")
        title.setStyleSheet("font-size:28px; font-weight:bold; font-family:Times New Roman;")
        title.setAlignment(Qt.AlignmentFlag.AlignHCenter)
        
        outer_layout.addWidget(title)

        self.tabs = QTabWidget()
        self.tabs.setStyleSheet("""
            QTabWidget::pane {
                border: 1px solid #cccccc;
            }
            QTabBar::tab {
                background-color: #f0f0f0;
                color: black;
                padding: 8px 20px;
                margin-right: 2px;
                border: 1px solid #cccccc;
                border-bottom: none;
            }
            QTabBar::tab:selected {
                background-color: white;
                border-bottom: 2px solid white;
            }
            QTabBar::tab:hover {
                background-color: #e0e0e0;
            }
        """)



        input_tab = QWidget()
        main_layout = QHBoxLayout(input_tab)
        main_layout.setSpacing(0)
        main_layout.setContentsMargins(0, 0, 0, 0)
    #    outer_layout.addLayout(main_layout)
        
        # ================= LEFT COLUMN =================
        left_widget = QVBoxLayout()
        left_widget.setSpacing(0)
        left_widget.setContentsMargins(0, 0, 0, 0) 
        main_layout.addLayout(left_widget, 2)
        
        inputs_section = QVBoxLayout()
        inputs_section.setSpacing(0)
        inputs_section.setContentsMargins(0, 0, 0, 0)
        left_widget.addLayout(inputs_section, 8)

        
        # Row 1, coulmn 1 : Controls
        controls_widget = QHBoxLayout()
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
        
        tables_matrix_Row = QHBoxLayout()
        # Row 1, coulmn 2 :DH Table
        dh_widget = Color("blue", "dh_widget")
        matrix_Wiget =  Color("#64f491", "input matrix")
        # Row 3: Outputs
       
        execute_widget = Color("purple", "execute")
        
        tables_matrix_Row.addWidget(dh_widget, 1) 
        tables_matrix_Row.addWidget(matrix_Wiget, 1)
        
        inputs_section.addLayout(tables_matrix_Row, 2) 
        left_widget.addWidget(execute_widget, 1)

        
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
        self.tabs.addTab(input_tab, "Inputs")
        
        output_tab = QWidget()
        output_layout = QHBoxLayout(output_tab)
        output_widget = Color("orange", "OUTPUT")
        output_layout.addWidget(output_widget, 6)
        self.tabs.addTab(output_tab, "Outputs")
        
        outer_layout.addWidget(self.tabs)
