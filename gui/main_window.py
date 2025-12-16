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
        
class MainWindow(QMainWindow): #defining our class (inheriting from QMainWindow)
    def __init__(self):    #Constructor
        super().__init__()  # calling the parent constructor

        self.setWindowTitle("Robotics IK/FK Calculator")   # giving a title to the window 
        self.resize(1600, 1200) # resizing the window
       
        central = QWidget()  # Create a central widget (required in QMainWindow)
        #central = Color("yellow")
        self.setCentralWidget(central)

        outer_layout = QVBoxLayout(central)

        # Title
        title = QLabel("Robotics IK/FK Calculation Platform")
        title.setStyleSheet("font-size:28px; font-weight:bold;")
        title.setAlignment(Qt.AlignmentFlag.AlignHCenter)
        
        outer_layout.addWidget(title)

        main_layout = QHBoxLayout()
        # main_layout.setSpacing(0)
        # main_layout.setContentsMargins(0, 0, 0, 0)
        outer_layout.addLayout(main_layout)
        
        # ================= LEFT COLUMN =================
        left_widget = QVBoxLayout()
        # left_widget.setSpacing(0)
        # left_widget.setContentsMargins(0, 0, 0, 0) 
        main_layout.addLayout(left_widget, 2)
        
        
        # ganna create those
        #  controls_widget = self.create_controls_widget()
        # dh_widget = self.create_dh_table_widget()
        # execute_widget = self.create_execute_widget()
        # output_widget = self.create_output_widget()
        
        
        inputs_section = QHBoxLayout()
        # inputs_section.setSpacing(0)
        # inputs_section.setContentsMargins(0, 0, 0, 0)
        left_widget.addLayout(inputs_section, 8)

        
        # Row 1: Controls
        controls_widget = QVBoxLayout()
        controls_widget.setSpacing(0)
        controls_widget.setContentsMargins(0, 0, 0, 0) 
        inputs_section.addLayout(controls_widget)
       
        minpulator_chose_box, self.manipulator_list_widget = self.manipulator_list()
        ik_fk, self.ik_fk_widget = self.ik_fk_selector()
        sym_num, self.sym_num_widget = self.sym_num_selector()
        theta_system, self.theta_system_widget = self.rad_deg_selector()
       
        controls_widget.addWidget(minpulator_chose_box)
        controls_widget.addWidget(ik_fk)
        controls_widget.addWidget(sym_num)
        controls_widget.addWidget(theta_system)
   #     controls_layout = QHBoxLayout()
        #controls_widget.setLayout(controls_layout)
   #     controls_widget.layout.addLayout(controls_layout)
        
        # Row 2: DH Table
        #dh_widget = QWidget()
        dh_widget = Color("blue", "dh_widget")
        # dh_layout = QVBoxLayout()
        #dh_widget.setLayout(dh_layout)
        
        # Row 3: Outputs
       # output_widget = QWidget()
        output_widget = Color("orange", "OUTPUT")
        execute_widget = Color("purple", "execute")
    #    output_layout = QVBoxLayout()
        #output_widget.setLayout(output_layout)
        # inputs_section.addWidget(controls_widget, 1)
        inputs_section.addWidget(dh_widget, 2)  
#        left_widget.addWidget(controls_widget, 1)
        # left_widget.addWidget(control_dh_widget, 3)
        left_widget.addWidget(execute_widget, 1)
        left_widget.addWidget(output_widget, 6)
        
        # ================= RIGHT COLUMN =================
        right_widget = QVBoxLayout()
        # right_widget.setSpacing(0)
        # right_widget.setContentsMargins(0, 0, 0, 0)
        main_layout.addLayout(right_widget, 1)
       
        # right_widget.setSpacing(0)
        # right_widget.setContentsMargins(0, 0, 0, 0)
        view3d_widget = Color("#1cccec", "3D VIEW") 
        choose_2D_sec = Color("#ec1c31", "2D_section") 
      #  view3d_widget = QWidget()
      #  view2d_widget = QWidget()
        view2d_widget = Color("#f7838f", "2D VIEW")
        
#### ganna work on them
        # view3d_widget = self.create_3d_view_widget()
        # choose_2d_widget = self.create_2d_selector_widget()
        # view2d_widget = self.create_2d_view_widget()
        
        
        right_widget.addWidget(view3d_widget, 8)
        right_widget.addWidget(choose_2D_sec, 1)
        right_widget.addWidget(view2d_widget, 8)


    def manipulator_list(self):
        return self.create_selector("Select a manipulator:", ["UR5", "ABB IRB1600", "ABB IRB2600"])

    def ik_fk_selector(self):
        return self.create_selector("Calculate:", ["Forward Kinamatics (FK)", "Inverse Kinamatics (IK)"])

    def sym_num_selector(self):
        return self.create_selector("Computation Mode:", ["Symbolic", "Numeric"])

    def rad_deg_selector(self):
        return self.create_selector("Given θ in:", ["Radians", "Degrees"])

    def create_selector(self, label_text, items, label_font_size=14, list_font_size=12):
        """
        Create a QWidget with a label and a QListWidget.

        Returns:
            (QWidget, QListWidget): The container widget and the list widget for signal handling.
        """
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.setSpacing(2)

        widget.setStyleSheet("""
        QWidget {
                border: 1px solid #cccccc;
                padding: 5px;
                border-radius: 5px;
                background-color: #f9f9f9;
             }
         """)
        
        font = QFont()
        font.setPointSize(label_font_size)
        font.setFamily("Roboto")

        label = QLabel(label_text)
        label.setFont(font)
        label.setStyleSheet("border: none; background: transparent;")
        label.setAlignment(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter)
        layout.addWidget(label)

        # List widget
        list_widget = QListWidget()
        font.setPointSize(list_font_size)
        list_widget.setFont(font)
        for text in items:
            item = QListWidgetItem(text)
            list_widget.addItem(item)
        list_widget.setCurrentRow(0)

        # Style list
        list_widget.setStyleSheet("""
            QListWidget {
                border: 2px solid #cccccc;
                border-radius: 3px;
                padding: 2px;
                background-color: white;
                outline: none;
            }
            QListWidget::item {
                padding: 8px 8px 8px 30px;
                border-bottom: 2px solid #f0f0f0;
                background-color: white;
            }
            QListWidget::item:hover {
                background-color: #e5f3ff;
            }
            QListWidget::item:selected {
                background-color: #cce8ff;
                color: black;
            }
        """)
        
        # Adjust height to remove empty space
        list_widget.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Minimum)
        list_widget.setMinimumHeight(
            list_widget.sizeHintForRow(0) * list_widget.count() + 2 * list_widget.frameWidth())

        # Custom selection indicator
        class SelectionIndicatorDelegate(QStyledItemDelegate):
            def paint(self, painter, option, index):
                super().paint(painter, option, index)  # Call parent to draw the item
                if option.state & QStyle.StateFlag.State_Selected:
                    painter.save()
                    rect = QRect(option.rect.left() + 8, option.rect.center().y() - 4, 8, 8)
                    painter.setRenderHint(QPainter.RenderHint.Antialiasing)
                    painter.setBrush(QColor("#0078d4"))
                    painter.setPen(Qt.PenStyle.NoPen)
                    painter.drawEllipse(rect)
                    painter.restore()

        list_widget.setItemDelegate(SelectionIndicatorDelegate())
        layout.addWidget(list_widget)

        return widget, list_widget












# # class MainWindow(QMainWindow):
# #     def __init__(self, test_mode=False):
# #         super().__init__()
# #         self.setWindowTitle("Robotics IK/FK Calculator")
# #         self.resize(1400, 1000)
        
# #         if test_mode:
# #             self.setup_test_layout()
# #         else:
# #             self.setup_main_layout()
    
# #     def setup_test_layout(self):
# #         """Test layout with colored blocks to verify structure"""
# #         central = Color("yellow")
# #         self.setCentralWidget(central)
# #         outer_layout = central.layout
        
# #         # Title
# #         title = QLabel("Robotics IK/FK Platform")
# #         title.setStyleSheet("font-size:28px; font-weight:bold; font-family:Times New Roman;")
# #         title.setAlignment(Qt.AlignmentFlag.AlignHCenter)
# #         outer_layout.addWidget(title)
        
# #         main_layout = QHBoxLayout()
# #         outer_layout.addLayout(main_layout)
        
# #         # LEFT COLUMN with colored blocks
# #         left_widget = QVBoxLayout()
# #         main_layout.addLayout(left_widget, 2)
        
# #         inputs_section = QHBoxLayout()
# #         left_widget.addLayout(inputs_section, 8)
        
# #         controls_widget = Color("#e747c5", "control widget")
# #         dh_widget = Color("blue", "dh_widget")
# #         execute_widget = Color("purple", "execute")
# #         output_widget = Color("orange", "OUTPUT")
        
# #         inputs_section.addWidget(controls_widget, 1)
# #         inputs_section.addWidget(dh_widget, 2)
# #         left_widget.addWidget(execute_widget, 1)
# #         left_widget.addWidget(output_widget, 5)
        
# #         # RIGHT COLUMN with colored blocks
# #         right_widget = QVBoxLayout()
# #         main_layout.addLayout(right_widget, 1)
        
# #         view3d_widget = Color("#1cccec", "3D VIEW")
# #         choose_2D_sec = Color("#ec1c31", "2D_section")
# #         view2d_widget = Color("#f7838f", "2D VIEW")
        
# #         right_widget.addWidget(view3d_widget, 8)
# #         right_widget.addWidget(choose_2D_sec, 1)
# #         right_widget.addWidget(view2d_widget, 8)
    
# #     def setup_main_layout(self):
# #         """Main production layout with actual widgets"""
# #         central = QWidget()
# #         self.setCentralWidget(central)
# #         outer_layout = QVBoxLayout(central)
        
# #         # Title
# #         title = QLabel("Robotics IK/FK Platform")
# #         title.setStyleSheet("font-size:28px; font-weight:bold; font-family:Times New Roman;")
# #         title.setAlignment(Qt.AlignmentFlag.AlignHCenter)
# #         outer_layout.addWidget(title)
        
# #         main_layout = QHBoxLayout()
# #         outer_layout.addLayout(main_layout)
        
# #         # LEFT COLUMN with real widgets
# #         left_widget = QVBoxLayout()
# #         main_layout.addLayout(left_widget, 2)
        
# #         # Controls section
# #         controls_widget = self.create_controls_widget()
# #         dh_widget = self.create_dh_table_widget()
# #         execute_widget = self.create_execute_widget()
# #         output_widget = self.create_output_widget()
        
# #         inputs_section = QHBoxLayout()
# #         left_widget.addLayout(inputs_section, 8)
# #         inputs_section.addWidget(controls_widget, 1)
# #         inputs_section.addWidget(dh_widget, 2)
# #         left_widget.addWidget(execute_widget, 1)
# #         left_widget.addWidget(output_widget, 5)
        
# #         # RIGHT COLUMN with real widgets
# #         right_widget = QVBoxLayout()
# #         main_layout.addLayout(right_widget, 1)
        
# #         view3d_widget = self.create_3d_view_widget()
# #         choose_2d_widget = self.create_2d_selector_widget()
# #         view2d_widget = self.create_2d_view_widget()
        
# #         right_widget.addWidget(view3d_widget, 8)
# #         right_widget.addWidget(choose_2d_widget, 1)
# #         right_widget.addWidget(view2d_widget, 8)
    
# #     # ============= Widget Creation Methods =============
    
# #     def create_controls_widget(self):
# #         """Create robot selection and control panel"""
# #         widget = QWidget()
# #         layout = QVBoxLayout(widget)
        
# #         # Robot selection dropdown
# #         layout.addWidget(QLabel("Select Robot:"))
# #         robot_combo = QComboBox()
# #         robot_combo.addItems(["PUMA 560", "Stanford Arm", "SCARA", "Custom"])
# #         layout.addWidget(robot_combo)
        
# #         # DOF selector
# #         layout.addWidget(QLabel("Degrees of Freedom:"))
# #         dof_spin = QSpinBox()
# #         dof_spin.setRange(2, 6)
# #         dof_spin.setValue(6)
# #         layout.addWidget(dof_spin)
        
# #         return widget
    
# #     def create_dh_table_widget(self):
# #         """Create DH parameters table"""
# #         widget = QWidget()
# #         layout = QVBoxLayout(widget)
        
# #         layout.addWidget(QLabel("DH Parameters"))
        
# #         table = QTableWidget(6, 4)
# #         table.setHorizontalHeaderLabels(["θ (deg)", "d (m)", "a (m)", "α (deg)"])
# #         layout.addWidget(table)
        
# #         return widget
    
# #     def create_execute_widget(self):
# #         """Create execution buttons"""
# #         widget = QWidget()
# #         layout = QHBoxLayout(widget)
        
# #         fk_button = QPushButton("Calculate FK")
# #         ik_button = QPushButton("Calculate IK")
        
# #         layout.addWidget(fk_button)
# #         layout.addWidget(ik_button)
        
# #         return widget
    
# #     def create_output_widget(self):
# #         """Create output display panel"""
# #         widget = QWidget()
# #         layout = QVBoxLayout(widget)
        
# #         layout.addWidget(QLabel("Results:"))
        
# #         output_text = QTextEdit()
# #         output_text.setReadOnly(True)
# #         layout.addWidget(output_text)
        
# #         return widget
    
# #     def create_3d_view_widget(self):
# #         """Create 3D visualization widget"""
# #         widget = QWidget()
# #         layout = QVBoxLayout(widget)
        
# #         layout.addWidget(QLabel("3D Visualization"))
# #         # Add your 3D view here (matplotlib, PyQtGraph, etc.)
        
# #         return widget
    
# #     def create_2d_selector_widget(self):
# #         """Create 2D plane selector"""
# #         widget = QWidget()
# #         layout = QHBoxLayout(widget)
        
# #         layout.addWidget(QLabel("View Plane:"))
# #         plane_combo = QComboBox()
# #         plane_combo.addItems(["XY", "XZ", "YZ"])
# #         layout.addWidget(plane_combo)
        
# #         return widget
    
# #     def create_2d_view_widget(self):
# #         """Create 2D visualization widget"""
# #         widget = QWidget()
# #         layout = QVBoxLayout(widget)
        
# #         layout.addWidget(QLabel("2D Projection"))
# #         # Add your 2D view here
        
# #         return widget




