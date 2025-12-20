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
        self.current_manipulator = None
        self.setWindowTitle("Robotics IK/FK Calculator")   # giving a title to the window 
        self.resize(1840, 1100) # resizing the window
       
        central = QWidget()  # Create a central widget (required in QMainWindow)
        #central = Color("yellow")
        self.setCentralWidget(central)
        self.standard_font = QFont("Roboto", 12)  # or "Roboto", 11
        self.label_font = QFont("Roboto", 14)
        
        outer_layout = QVBoxLayout(central)

        # Title
        title = QLabel("Robotics IK/FK Calculation Platform")
        title.setStyleSheet("font-size:28px; font-weight:bold;")
        title.setAlignment(Qt.AlignmentFlag.AlignHCenter)
        
        outer_layout.addWidget(title)

        self.tabs = QTabWidget()
        self.tabs.setFont(self.standard_font) 
        self.tabs.setStyleSheet("""
            QTabWidget::pane {
                border: 2px solid #cccccc;
                border-radius: 3px;
                background-color: white;
            }
            QTabBar {
                background-color: white;
            }
            QTabBar::tab {
                background-color: white;
                color: black;
                padding: 8px 20px;
                margin-right: 2px;
                border: 2px solid #cccccc;
                border-bottom: none;
                border-radius: 3px 3px 0px 0px;
            }
            QTabBar::tab:selected {
                background-color: white;
                border: 2px solid #cccccc;
                border-bottom: 2px solid white;
                font-weight: bold;
                color: #0078d4;
            }
            QTabBar::tab:hover {
                background-color: #e5f3ff;
            }
        """)
        input_tab = QWidget()
        main_layout = QHBoxLayout(input_tab)
        # main_layout.setSpacing(0)
        # main_layout.setContentsMargins(0, 0, 0, 0)
       # outer_layout.addLayout(main_layout)
        
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
        
        
        inputs_section = QVBoxLayout()
        # inputs_section.setSpacing(0)
        # inputs_section.setContentsMargins(0, 0, 0, 0)
        left_widget.addLayout(inputs_section, 7)

        
        # Row 1: Controls
        controls_widget = QHBoxLayout()
        controls_widget.setSpacing(5)
        controls_widget.setContentsMargins(0, 0, 0, 0) 
        inputs_section.addLayout(controls_widget, 1)
       
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
        tables_matrix_Row = QHBoxLayout()
        dh_widget = self.create_dh_table_widget()
        matrix_Widget = Color("#38edde", "T-matrix")
        #Color("blue", "dh_widget")
        # dh_layout = QVBoxLayout()
        #dh_widget.setLayout(dh_layout)
        
        # Row 3: Outputs
       # output_widget = QWidget()
       # output_widget = Color("orange", "OUTPUT")
        execute_widget = Color("purple", "execute")
    #    output_layout = QVBoxLayout()
        #output_widget.setLayout(output_layout)
        # inputs_section.addWidget(controls_widget, 1)
     ##   inputs_section.addWidget(dh_widget, 5)  
#        left_widget.addWidget(controls_widget, 1)
        # left_widget.addWidget(control_dh_widget, 3)
        tables_matrix_Row.addWidget(dh_widget, 1) 
        tables_matrix_Row.addWidget(matrix_Widget, 1)   
        inputs_section.addLayout(tables_matrix_Row, 2) 
        left_widget.addWidget(execute_widget, 1)
       # left_widget.addWidget(output_widget, 7)
        
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
        self.tabs.addTab(input_tab, "Inputs")
        output_tab = QWidget()
        output_layout = QHBoxLayout(output_tab)
        output_widget = Color("orange", "OUTPUT")
        output_layout.addWidget(output_widget, 6)
        self.tabs.addTab(output_tab, "Outputs")
        outer_layout.addWidget(self.tabs)
        
        self.toggle_value_column()


    def manipulator_list(self):
        return self.create_selector("Select a manipulator:", ["UR5", "ABB IRB 1600", "KUKA KR16"])

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
                padding: 3px;
                border-radius: 5px;
                background-color: #f9f9f9;
             }
         """)
        

        label = QLabel(label_text)
        label.setFont(self.label_font)
        label.setStyleSheet("border: none; background: transparent;")
        label.setAlignment(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter)
        layout.addWidget(label)

        # List widget
        list_widget = QListWidget()
        list_widget.setFont(self.standard_font)
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


# Create DH parameters table that auto-populates based on selected manipulator.
# Fixed cells are non-editable, only the 'Value' column is editable.
    def create_dh_table_widget(self):
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.setSpacing(5)
        layout.setContentsMargins(5, 5, 5, 5)

        widget.setStyleSheet("""
            QWidget {
                border: 1px solid #cccccc;
                padding: 1px;
                border-radius: 5px;
                background-color: #f9f9f9;
            }
        """)

        # Title

        title_label = QLabel("DH Parameters Table")
        title_label.setFont(self.label_font)
        title_label.setStyleSheet("border: none; background: transparent;")
        title_label.setAlignment(Qt.AlignmentFlag.AlignHCenter | Qt.AlignmentFlag.AlignVCenter)
        layout.addWidget(title_label)

        # Create table
        self.dh_table = QTableWidget()
        self.dh_table.setColumnCount(7)
        self.numeric_delegate = NumericDelegate(self.dh_table)
        self.dh_table.setItemDelegateForColumn(6, self.numeric_delegate)
        self.dh_table.setFont(self.standard_font) 
         
        # Set initial headers (will be updated based on angle unit selection)
        headers = ["Joint", "θ", "d", "a", "α", "Variable", "Value"]
        self.dh_table.setHorizontalHeaderLabels(headers)

        # Style the table
        self.dh_table.setStyleSheet("""
            QTableWidget {
                border: 1px solid #cccccc;
                border-radius: 3px;
                background-color: white;
                gridline-color: #e0e0e0;
            }
            QTableWidget::item {
                padding: 4px;
            }
            QHeaderView::section {
                background-color: #0078d4;
                color: white;
                padding: 10px;
                border: 1px solid #005a9e;
                font-weight: bold;
            }
        """)

        header = self.dh_table.horizontalHeader()
        header.setFont(self.standard_font)
        header.setSectionResizeMode(QHeaderView.ResizeMode.Stretch) 
        self.dh_table.verticalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        self.dh_table.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)

        # Enable alternating row colors for better readability
        self.dh_table.setAlternatingRowColors(True)
        self.dh_table.verticalHeader().setVisible(False)
        layout.addWidget(self.dh_table)

        # Connect signals
        self.manipulator_list_widget.currentRowChanged.connect(self.update_dh_table)
        self.theta_system_widget.currentRowChanged.connect(self.update_dh_headers)
        self.sym_num_widget.currentRowChanged.connect(self.toggle_value_column)
        # Initialize table with first manipulator
        self.update_dh_table(0)
        return widget

    def toggle_value_column(self):
        """Show/hide the Value column based on computation mode"""
        computation_mode = self.sym_num_widget.currentRow()

        if computation_mode == 0:  # Symbolic
            self.dh_table.setColumnHidden(6, True)  # Hide Value column
        else:  # Numeric
            self.dh_table.setColumnHidden(6, False)  # Show Value column
            
#  Update DH table when manipulator selection changes
    def update_dh_table(self, index, update_headers=True):
        manipulators = ["UR5", "ABB_IRB_1600", "KUKA_KR16"]
        if index < 0 or index >= len(manipulators):
            return

        manipulator = manipulators[index]

        # Store the current manipulator instance
        self.current_manipulator = create_manipulator(manipulator)
        dh_params = self.current_manipulator.get_dh_parameters()

        # Set number of rows
        self.dh_table.setRowCount(len(dh_params))

        # Get current angle unit (0 = Radians, 1 = Degrees)
        angle_unit = self.theta_system_widget.currentRow()
        in_degrees = (angle_unit == 1)

        for row, params in enumerate(dh_params):
            joint_item = QTableWidgetItem(str(row + 1))
            joint_item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
            joint_item.setFlags(Qt.ItemFlag.ItemIsEnabled)
            joint_item.setBackground(QColor("#faf8f8"))
            self.dh_table.setItem(row, 0, joint_item)
        
        
            # Column 0: θ (theta) - show ONLY if it's a FIXED value
            if params["variable"] == "theta":
                var_symbol = f"θ{row + 1}"
                theta_item = QTableWidgetItem(var_symbol)
            else:
                # This is fixed, show the value
                theta_val = params["theta"]
                if in_degrees:
                    theta_val = np.rad2deg(theta_val)
                theta_item = QTableWidgetItem(f"{theta_val:.4f}")

            theta_item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
            theta_item.setFlags(Qt.ItemFlag.ItemIsEnabled)
            theta_item.setBackground(QColor("#faf8f8"))
            self.dh_table.setItem(row, 1, theta_item)

            # Column 1: d - show ONLY if it's a FIXED value
            if params["variable"] == "d":
                var_symbol = f"d{row + 1}"
                # This is a variable joint, show empty or dash
                d_item = QTableWidgetItem(var_symbol)
            else:
                # This is fixed, show the value
                d_item = QTableWidgetItem(f"{params['d']:.4f}")

            d_item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
            d_item.setFlags(Qt.ItemFlag.ItemIsEnabled)
            d_item.setBackground(QColor("#faf8f8"))
            self.dh_table.setItem(row, 2, d_item)

            # Column 2: a (always fixed)
            a_item = QTableWidgetItem(f"{params['a']:.4f}")
            a_item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
            a_item.setFlags(Qt.ItemFlag.ItemIsEnabled)
            a_item.setBackground(QColor("#faf8f8"))
            self.dh_table.setItem(row, 3, a_item)

            # Column 3: α (alpha) - always fixed
            alpha_val = params["alpha"]
            alpha_val_num = float(alpha_val)
            if in_degrees:
                alpha_val_num = np.rad2deg(alpha_val_num)
            alpha_item = QTableWidgetItem(f"{alpha_val_num:.4f}")
            alpha_item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
            alpha_item.setFlags(Qt.ItemFlag.ItemIsEnabled)
            alpha_item.setBackground(QColor("#faf8f8"))
            self.dh_table.setItem(row, 4, alpha_item)

            # Column 4: Variable (θ₁, θ₂, etc. or d₁, d₂ for prismatic)
            var_symbol = self.current_manipulator.get_joint_variable_name(row)
            var_item = QTableWidgetItem(var_symbol)
            var_item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
            var_item.setFlags(Qt.ItemFlag.ItemIsEnabled)
            var_item.setBackground(QColor("#e3f2fd"))
            var_item.setForeground(QColor("#0078d4"))
            font = var_item.font()
            font.setBold(True)
            var_item.setFont(font)
            self.dh_table.setItem(row, 5, var_item)

            # Column 5: Value (EDITABLE)
            value_item = QTableWidgetItem("0.0")
            value_item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
            value_item.setFlags(Qt.ItemFlag.ItemIsEnabled | Qt.ItemFlag.ItemIsEditable)
            value_item.setBackground(QColor("#ffffff"))
            self.dh_table.setItem(row, 6, value_item)
        if update_headers:
            self.update_dh_headers()

#    Returns numeric joint values (always in radians internally)
    def get_joint_values(self):
        if self.sym_num_widget.currentRow() == 0:
            return None  # symbolic mode

        values = []
        angle_unit = self.theta_system_widget.currentRow()  # 0=rad, 1=deg

        for row in range(self.dh_table.rowCount()):
            item = self.dh_table.item(row, 6)

            if item is None or item.text().strip() == "":
                raise ValueError(f"Joint {row + 1} has no value")

            value = float(item.text())
            print(f"Joint {row+1}: raw =", item.text())

            # Revolute joint → angle
            joint_type = self.current_manipulator.get_joint_type(row)
            if joint_type == "revolute" and angle_unit == 1:
                value = np.deg2rad(value)

            values.append(value)

        return values

#Update table headers when angle unit changes
    def update_dh_headers(self):
        angle_unit = self.theta_system_widget.currentRow()

        if angle_unit == 0:  # Radians
            headers = ["Joint", "θ (rad)", "d (m)", "a (m)", "α (rad)", "Variable", "Value"]
        else:  # Degrees
            headers = ["Joint", "θ (deg)", "d (m)", "a (m)", "α (deg)", "Variable", "Value"]

        self.dh_table.setHorizontalHeaderLabels(headers)

        # Re-update the table to convert angle values
        if hasattr(self, 'current_manipulator') and self.current_manipulator:
            current_index = self.manipulator_list_widget.currentRow()
            self.update_dh_table(current_index, update_headers=False)


##for execute button
# def execute_fk(self):
#     try:
#         joint_values = self.get_joint_values()
#     except ValueError as e:
#         QMessageBox.warning(self, "Input Error", str(e))
#         return

#     T = self.current_manipulator.forward_kinematics(joint_values)
#     self.display_matrix(T)


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




