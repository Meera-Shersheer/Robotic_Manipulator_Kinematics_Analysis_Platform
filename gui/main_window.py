from imports import *
from gui.output import *
from gui.cad_viewer import *



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
        self.current_T = None
        self.setWindowTitle("  ")   # giving a title to the window 
        self.resize(1800, 1300) # resizing the window
       
        central = QWidget()  # Create a central widget (required in QMainWindow)
        #central = Color("yellow")
        self.setCentralWidget(central)
        self.standard_font = QFont("Roboto", 13)  # or "Roboto", 11
        self.label_font = QFont("Roboto", 15)
        self.large_font = QFont("Roboto", 18)
        
        outer_layout = QVBoxLayout(central)

        # Title
        title = QLabel("Robotic Manipulator Kinematics Analysis Platform")
        title.setStyleSheet("font-size:28px; font-weight:bold; background-color: #8e24aa; color: white; padding : 10px; border-radius : 8px")
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
                color: #8e24aa;
            }
            QTabBar::tab:hover {
                background-color: #f3e5f5;
            }
        """)
        

        input_tab = QWidget()
        main_layout = QHBoxLayout(input_tab)

        
        inputs_section = QVBoxLayout()
        main_layout.addLayout(inputs_section, 7)

        
        # Row 1: Controls
        controls_widget = QHBoxLayout()
        controls_widget.setSpacing(5)
        controls_widget.setContentsMargins(0, 0, 0, 0) 
        inputs_section.addLayout(controls_widget, 2)
       
        minpulator_chose_box, self.manipulator_list_widget = self.manipulator_list()
        ik_fk, self.ik_fk_widget = self.ik_fk_selector()
        sym_num, self.sym_num_widget = self.sym_num_selector()
        theta_system, self.theta_system_widget = self.rad_deg_selector()

        fram_range = QVBoxLayout()
        inputs_section.addLayout(fram_range, 1)
        self.input_options_widget  = self.create_input_options_widget()

        controls_widget.addWidget(minpulator_chose_box)
        controls_widget.addWidget(ik_fk)
        controls_widget.addWidget(sym_num)
        controls_widget.addWidget(theta_system)
        fram_range.addWidget(self.input_options_widget)
        
        # Row 2: DH Table
        tables_matrix_Row = QHBoxLayout()
        dh_widget = self.create_dh_table_widget()
        matrix_Widget = self.create_matrix_display_widget()

        tables_matrix_Row.addWidget(dh_widget, 6) 
        tables_matrix_Row.addWidget(matrix_Widget, 4)   
        inputs_section.addLayout(tables_matrix_Row, 8) 
        
        execute_widget = self.create_execute_widget()
        inputs_section.addWidget(execute_widget, 1)
        self.tabs.addTab(input_tab, "Inputs")
        
        
        output_tab = QWidget()
        output_layout = QHBoxLayout(output_tab)
        output_widget = self.create_output_widget()
        #Color("orange", "OUTPUT")
        output_layout.addWidget(output_widget, 6)
        self.tabs.addTab(output_tab, "Outputs")
        
        

        cad_tab = QWidget()
        cad_layout = QHBoxLayout(cad_tab)
        
        left_widget = QWidget()
        left_layout = QVBoxLayout(left_widget)
        left_layout.setContentsMargins(0, 0, 0, 0)
        left_layout.setSpacing(4)
        # right_widget = QVBoxLayout()
        # main_layout.addLayout(right_widget, 1)
       
        # right_widget.setSpacing(0)
        # right_widget.setContentsMargins(0, 0, 0, 0)
        #Color("#ec1c31", "2D_section")
       
        #Color("#ec1c31", "2D_section")
        #self.create_cad_control_panel()
         
      #  view3d_widget = QWidget()
      #  view2d_widget = QWidget()
        self.view3d_widget = Color("#1cccec", "3D VIEW") 
        # OpenGLViewer()
        control_cad =  Color("#ec1c31", "2D_section")
        # self.create_cad_control_panel()
        view2d_widget = Color("#f7838f", "2D VIEW")
        
        left_layout.addWidget(control_cad, 1)  # top: control
        left_layout.addWidget(view2d_widget, 1)  # bottom: 2D view
#### ganna work on them
        # view3d_widget = self.create_3d_view_widget()
        # choose_2d_widget = self.create_2d_selector_widget()
        # view2d_widget = self.create_2d_view_widget()
        
        #Color("#1cccec", "3D VIEW") 
       
        cad_layout.addWidget(left_widget, 3)      # 25%
        cad_layout.addWidget(self.view3d_widget, 5) 

        self.tabs.addTab(cad_tab, "CAD Model")
        outer_layout.addWidget(self.tabs)
        # test_output_widget(self)
        self.toggle_value_column()
        self.hide_matrix()
        self.hide_frame_selector()


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
                background-color: #F5EFFF;
            }
            QListWidget::item:selected {
                background-color: #f3e5f5;
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
                    rect = QRect(option.rect.left() + 12, option.rect.center().y() - 6, 12, 12)
                    painter.setRenderHint(QPainter.RenderHint.Antialiasing)
                    painter.setBrush(QColor("#8f22ad"))
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
                gridline-color: #cccccc;
            }
            QTableWidget::item {
                padding: 6px;
            }
            QHeaderView::section {
                background-color: #8e24aa;
                color: white;
                padding: 10px;
                border: 1px solid #6a1b9a;
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
        self.ik_fk_widget.currentRowChanged.connect(self.toggle_value_column)
        # Initialize table with first manipulator
        self.update_dh_table(0)
        return widget

    def toggle_value_column(self):
        """Show/hide the Value column based on computation mode"""
        computation_mode = self.sym_num_widget.currentRow()
        ik_or_fk = self.ik_fk_widget.currentRow()

        if (computation_mode == 0) or (ik_or_fk == 1) :  # Symbolic
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
            var_item.setBackground(QColor("#f3e5f5"))
            var_item.setForeground(QColor("#8e24aa"))
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

# #    Returns numeric joint values (always in radians internally)
#     def get_joint_values(self):
#         if self.sym_num_widget.currentRow() == 0:
#             return None  # symbolic mode

#         values = []
#         angle_unit = self.theta_system_widget.currentRow()  # 0=rad, 1=deg

#         for row in range(self.dh_table.rowCount()):
#             item = self.dh_table.item(row, 6)

#             if item is None or item.text().strip() == "":
#                 raise ValueError(f"Joint {row + 1} has no value")

#             value = float(item.text())
#             print(f"Joint {row+1}: raw =", item.text())

#             # Revolute joint → angle
#             joint_type = self.current_manipulator.get_joint_type(row)
#             if joint_type == "revolute" and angle_unit == 1:
#                 value = np.deg2rad(value)

#             values.append(value)

#         return values

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

# Create widget to display transformation matrix
    def create_matrix_display_widget(self):
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.setSpacing(5)
        layout.setContentsMargins(5, 5, 5, 5)

        widget.setStyleSheet("""
            QWidget {
                border: 1px solid #cccccc;
                padding: 5px;
                border-radius: 5px;
                background-color: #f9f9f9;
            }
        """)

        title_label = QLabel("Transformation Matrix T₀⁶")
        title_label.setFont(self.label_font)
        title_label.setStyleSheet("border: none; background: transparent;")
        title_label.setAlignment(Qt.AlignmentFlag.AlignHCenter)
        layout.addWidget(title_label)

        self.matrix_placeholder = QLabel(
            "Transformation matrix will be shown\n"
            "when Inverse Kinematics is selected."
        )
        self.matrix_placeholder.setFont(self.standard_font)
        self.matrix_placeholder.setAlignment(Qt.AlignmentFlag.AlignVCenter | Qt.AlignmentFlag.AlignHCenter)
        self.matrix_placeholder.setStyleSheet("""
            QLabel {
                border: none;
                color: #333333;  
                padding: 30px;
                background: transparent;
            }
        """)
        layout.addWidget(self.matrix_placeholder, alignment=Qt.AlignmentFlag.AlignCenter)

        # Matrix display table
        self.matrix_table = QTableWidget(4, 4)
        self.matrix_table.setItemDelegate(MatrixDelegate(self.matrix_table))
        self.matrix_table.setFont(self.standard_font)
        self.matrix_table.horizontalHeader().setVisible(False)
        self.matrix_table.verticalHeader().setVisible(False)
        
        self.matrix_table.setStyleSheet("""
            QTableWidget {
                border: 1px solid #cccccc;
                border-radius: 3px;
                background-color: white;
                gridline-color: #e0e0e0;
            }
            QTableWidget::item {
                padding: 6px;
                text-align: center;
            }
        """)

        header = self.matrix_table.horizontalHeader()
        header.setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        self.matrix_table.verticalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        
        # Initialize with zeros
        self.reset_matrix()

        self.ik_fk_widget.currentRowChanged.connect(self.hide_matrix)
        self.sym_num_widget.currentRowChanged.connect(self.reset_matrix)
        layout.addWidget(self.matrix_table)
        return widget

    #Show/hide the T matrix based on mode mode
    def hide_matrix(self):
        """Show/hide the T matrix based on mode"""
        kinematics_type = self.ik_fk_widget.currentRow()

        if kinematics_type == 1:  # IK mode
            # Show matrix only if matrix input mode is selected
            use_matrix = self.ik_matrix_radio.isChecked()
            self.matrix_table.setVisible(use_matrix)
            self.matrix_placeholder.setVisible(not use_matrix)
        else:  # FK mode
            self.matrix_table.setVisible(False)
            self.matrix_placeholder.setVisible(True)
        
        
    def toggle_ik_input_mode(self):
        """Toggle between pose input and matrix input for IK"""
        use_pose = self.ik_pose_radio.isChecked()

        # Enable/disable pose input fields
        for input_field in self.pose_inputs.values():
            input_field.setEnabled(use_pose)
        # Update matrix visibility in the matrix widget area
        self.hide_matrix()
        
        
    def read_T_matrix_from_table(self):
        T = [[None]*4 for _ in range(4)]
        comp_mode = self.sym_num_widget.currentRow()
        kinematics_type = self.ik_fk_widget.currentRow()
        
        use_symbolic = (comp_mode == 0) and (kinematics_type == 1)
        
        for i in range(4):
            for j in range(4):
                item = self.matrix_table.item(i, j)
                if item is None:
                    text = "0"
                else:
                    text = item.text().strip()

                if use_symbolic:
                    try:
                        T[i][j] = sympy.sympify(text)
                    except:
                        T[i][j] = sympy.sympify("0")
                else:
                    try:
                        T[i][j] = float(text)
                    except:
                        T[i][j] = 0.0

        return sympy.Matrix(T) if use_symbolic else np.array(T)
    
    def reset_matrix(self):
        """Reset matrix to identity matrix"""
        identity = [["1" if i == j else "0" for j in range(4)] for i in range(4)]
        
        for i in range(4):
            for j in range(4):
                # Create item if it doesn't exist
                item = self.matrix_table.item(i, j)
                if item is None:
                    item = QTableWidgetItem()
                    self.matrix_table.setItem(i, j, item)
                
                # Set the value
                item.setText(identity[i][j])
                item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
                item.setFlags(Qt.ItemFlag.ItemIsEnabled | Qt.ItemFlag.ItemIsEditable)       
    
    # Create execute button widget
    def create_execute_widget(self):
        widget = QWidget()
        layout = QHBoxLayout(widget)
        layout.setSpacing(5)
        layout.setContentsMargins(5, 5, 5, 5)

        widget.setStyleSheet("""
            QWidget {
                border: #f5f5f5;;
                padding: 10px;
                border-radius: 10px;
                background-color: white;
                
            }
        """)
        # Execute button
        self.execute_button = QPushButton("Calculate")
        self.execute_button.setFont(self.label_font)
        self.execute_button.setCursor(Qt.CursorShape.PointingHandCursor)
        self.execute_button.setFixedSize(150, 50)  
        self.execute_button.setStyleSheet("""
            QPushButton {
                background-color: #8e24aa;
                color: #f9f9f9; 
                border: 2px solid #8e24aa; 
                border-radius: 7px;
                padding: 10px 14px;
                font-weight: 600;
            }
            QPushButton:hover {
                background-color: #7b1fa2;
                border-color: #7b1fa2;
            }

            QPushButton:pressed {
                background-color: #6a1b9a;
                border-color: #6a1b9a;
            }
            QPushButton:disabled {
                background-color: #cccccc;
                color: #666666;
                border-color: #cccccc;
            }
            """)
        self.execute_button.clicked.connect(self.execute_calculation)
        #self.execute_button.clicked.connect(self.test_rotation)
        layout.addWidget(self.execute_button)
        
        # self.test_3d_button = QPushButton("Rotate 3D View")
        # self.test_3d_button.setFont(self.label_font)
        # self.test_3d_button.setCursor(Qt.CursorShape.PointingHandCursor)
        # self.test_3d_button.setFixedSize(300, 50)
        # self.test_3d_button.setStyleSheet("""
        #     QPushButton {
        #         background-color: #00897b;
        #         color: #f9f9f9; 
        #         border: 2px solid #00897b; 
        #         border-radius: 7px;
        #         padding: 10px 14px;
        #         font-weight: 600;
        #     }
        #     QPushButton:hover {
        #         background-color: #00695c;
        #         border-color: #00695c;
        #     }
        # """)
        #self.test_3d_button.clicked.connect(lambda: self.view3d_widget.rotate_mesh(0, 30, 'z'))
        #layout.addWidget(self.test_3d_button)
        return widget

#Create output display widget
    def create_output_widget(self):
        widget = QWidget()
        main_layout = QVBoxLayout(widget)
        main_layout.setSpacing(10)
        main_layout.setContentsMargins(10, 10, 10, 10)

        widget.setStyleSheet("""
            QWidget {
                background-color: #f9f9f9;
            }
        """)

        # Title
        title = QLabel("Calculation Results")
        title.setFont(self.large_font)
        title.setStyleSheet("color: #8e24aa; padding: 10px; font-weight: 600;")
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        main_layout.addWidget(title)

        # Create scrollable area
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setStyleSheet("""
        QScrollArea {
            border:none;
            border-radius: 10px;
            background-color: #fdf7fa;
        }
        QScrollBar:vertical {
            border:1px solid  #ef5350;
            background-color: #ffebee;
            width: 14px;
            border-radius: 6px;
            margin: 0px;
        }
        QScrollBar::handle:vertical {
            background: #e53935;
            border-radius: 6px;
            min-height: 30px;
        }
        QScrollBar::handle:vertical:hover {
            background: #d32f2f;
        }
        QScrollBar::handle:vertical:pressed {
            background-color: #d32f2f;
        }
        QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {
            border: none;
            background: none;
            height: 0px;
        }
        QScrollBar::add-page:vertical, QScrollBar::sub-page:vertical {
            background: none;
        }
        """)
        scroll_content = QWidget()
        self.output_layout = QVBoxLayout(scroll_content)
        self.output_layout.setSpacing(15)
        self.output_layout.setContentsMargins(15, 15, 15, 15)

        # Add initial message
        initial_msg = QLabel("No results yet. Click 'Calculate' to see results here.")
        initial_msg.setFont(self.standard_font)
        initial_msg.setStyleSheet("color: #666; padding: 20px;")
        initial_msg.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.output_layout.addWidget(initial_msg)
        self.output_layout.addStretch()

        scroll.setWidget(scroll_content)
        main_layout.addWidget(scroll)

        return widget
    
    def create_input_options_widget(self, dof=6):
        # Container widget
        
        group = QGroupBox()
        layout = QVBoxLayout(group)
        
        group.setStyleSheet("""
        QGroupBox {
            border: 1px solid #cccccc;
            border-radius: 5px;
            background-color: #f9f9f9;
        }
        """)
        
        self.fk_frame_widget = QWidget()
        fk_layout = QVBoxLayout(self.fk_frame_widget)
        # ---- Title ----
        title = QLabel("Selected Frames Whose Transformation Matrices Will Be Displayed:")
        title.setFont(self.label_font)
        title.setStyleSheet("border: none; background: transparent;")
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        fk_layout.addWidget(title)

        # Mode selector: All / Range
        radio_layout = QHBoxLayout()
        self.fk_all_radio = QRadioButton("All")
        self.fk_range_radio = QRadioButton("Range")
        self.fk_all_radio.setChecked(True)  # default
        
        self.fk_all_radio.setFont(self.standard_font)
        self.fk_all_radio.setCursor(Qt.CursorShape.PointingHandCursor)
        self.fk_range_radio.setFont(self.standard_font)
        self.fk_range_radio.setCursor(Qt.CursorShape.PointingHandCursor)


        radio_layout.addWidget(self.fk_all_radio)
        radio_layout.setSpacing(200)
        radio_layout.addWidget(self.fk_range_radio)
        radio_layout.setAlignment(Qt.AlignmentFlag.AlignCenter)
        
        # From/To spinboxes
        spin_layout = QHBoxLayout()
        
        from_widget, self.fk_from_spin, self.fk_from_buttons = self.create_spin_with_buttons(0, dof, 0)
        to_widget, self.fk_to_spin, self.fk_to_buttons = self.create_spin_with_buttons(0, dof, dof)
        
        
        from_label = QLabel("From:")
        to_label = QLabel("To:")
        from_label.setFont(self.standard_font)
        to_label.setFont(self.standard_font)
        self.fk_from_spin.setFont(self.standard_font)
        self.fk_from_spin.setFixedWidth(100)
        self.fk_from_spin.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.fk_to_spin.setFont(self.standard_font)
        self.fk_to_spin.setFixedWidth(100)
        self.fk_to_spin.setAlignment(Qt.AlignmentFlag.AlignCenter)
           
        spin_layout.addSpacing(280)  
        spin_layout.addWidget(from_label)
        spin_layout.addSpacing(5)
        spin_layout.addWidget(from_widget)
        spin_layout.addSpacing(60)
        spin_layout.addWidget(to_label)
        spin_layout.addSpacing(5)
        spin_layout.addWidget(to_widget)
        spin_layout.setAlignment(Qt.AlignmentFlag.AlignCenter)



        self.fk_from_spin.setButtonSymbols(QAbstractSpinBox.ButtonSymbols.NoButtons)
        self.fk_to_spin.setButtonSymbols(QAbstractSpinBox.ButtonSymbols.NoButtons)
    
        
        self.setStyleSheet("""
            QRadioButton {
                padding: 20px 40px;
                border-radius: 5px;
            }

            QRadioButton::indicator:checked {
                background-color: #8e24aa;
                border-radius: 7px;
                border: 2px solid #8e24aa;
            }
            QSpinBox {
                 padding: 4px;
                 min-height: 20px;
                 border: 1px solid #00897b;
                 border-radius: 4px;
             }
                """)
                              
        fk_layout.addLayout(radio_layout)
        fk_layout.addLayout(spin_layout)
        # --- Tooltips ---
        self.fk_all_radio.setToolTip("Display all FK frames")
        self.fk_range_radio.setToolTip("Display only a specific frame range")
        self.fk_from_spin.setToolTip("Start frame (0–6)")
        self.fk_to_spin.setToolTip("End frame (0–6)")
    

        # Initially disable From/To if "All" is selected
        self.fk_from_spin.setEnabled(False)
        self.fk_to_spin.setEnabled(False)
        for btn in self.fk_from_buttons:
            btn.setEnabled(False)

        for btn in self.fk_to_buttons:
            btn.setEnabled(False)
        
        # --- Connections ---
        self.fk_all_radio.toggled.connect(self.update_fk_spinbox_state)
        self.fk_from_spin.valueChanged.connect(self.validate_frame_range)
        self.fk_to_spin.valueChanged.connect(self.validate_frame_range)
        self.ik_fk_widget.currentRowChanged.connect(self.hide_frame_selector)
        
        self.ik_pose_widget = QWidget()
        ik_layout = QVBoxLayout(self.ik_pose_widget)
        #ik_layout.setSpacing(5)  # Tight spacing
        ik_layout.setContentsMargins(0, 0, 0, 0) 

        # Title with input mode selector
        title_layout = QHBoxLayout()
        ik_title = QLabel("Select the method of entering the desired End-Effector Pose")
        ik_title.setFont(self.label_font)
        ik_title.setStyleSheet("border: none; background: transparent;")
        ik_title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        ik_layout.addWidget(ik_title)

        title_layout.addSpacing(5)

        # Radio buttons for input mode
        self.ik_matrix_radio = QRadioButton("Homogeneous Transformation Matrix (4×4)")
        self.ik_pose_radio = QRadioButton("Position & Orientation (x, y, z, α, β, γ)")
        self.ik_pose_radio.setChecked(True)  # Default to pose (since matrix is below)

        self.ik_matrix_radio.setFont(self.standard_font)
        self.ik_pose_radio.setFont(self.standard_font)
        self.ik_matrix_radio.setCursor(Qt.CursorShape.PointingHandCursor)
        self.ik_pose_radio.setCursor(Qt.CursorShape.PointingHandCursor)

        title_layout.addWidget(self.ik_pose_radio)
        #title_layout.addSpacing(10)
        title_layout.addWidget(self.ik_matrix_radio)
        # title_layout.addStretch()
        title_layout.setAlignment(Qt.AlignmentFlag.AlignCenter)
        ik_layout.addLayout(title_layout)

        self.pose_inputs = {}
        
        pos_layout = QHBoxLayout()
        pos_layout.setAlignment(Qt.AlignmentFlag.AlignCenter)
        pos_layout.setSpacing(50)
        for axis in ['X', 'Y', 'Z']:
            axis_layout = QVBoxLayout()
            axis_layout.setSpacing(5)
            axis_layout.setContentsMargins(0, 0, 0, 0)

            label = QLabel(axis.upper())
            label.setFont(self.standard_font)
            label.setAlignment(Qt.AlignmentFlag.AlignCenter)
            label.setStyleSheet("font-weight: bold; color: #00897b;")

            input_field = QLineEdit("0.0")
            input_field.setFont(self.standard_font)
            input_field.setFixedWidth(120)  # Compact width
            input_field.setFixedHeight(40) 
            input_field.setAlignment(Qt.AlignmentFlag.AlignCenter)
            input_field.setValidator(QDoubleValidator())
            input_field.setStyleSheet("""
                QLineEdit {
                    padding: 6px;
                    border: 2px solid #00897b;
                    border-radius: 4px;
                    background-color: white;
                }
                QLineEdit:focus {
                    border: 2px solid #00695c;
                }
            """)

            axis_layout.addWidget(label)
            axis_layout.addWidget(input_field)
            pos_layout.addLayout(axis_layout)
            
            self.pose_inputs[axis] = input_field

        ik_layout.addLayout(pos_layout)



        ori_layout = QHBoxLayout()
        ori_layout.setAlignment(Qt.AlignmentFlag.AlignCenter)
        ori_layout.setSpacing(50)
        for angle, symbol in [('alpha', 'α (Roll)'), ('beta', 'β (Pitch)'), ('gamma', 'γ (Yaw)')]:
            angle_layout = QVBoxLayout()
            angle_layout.setSpacing(5)
            angle_layout.setContentsMargins(0, 0, 0, 0)

            label = QLabel(symbol)
            label.setFont(self.standard_font)
            label.setAlignment(Qt.AlignmentFlag.AlignCenter)
            label.setStyleSheet("font-weight: bold; color: #8e24aa;")

            input_field = QLineEdit("0.0")
            input_field.setFont(self.standard_font)
            input_field.setFixedWidth(120) 
            input_field.setFixedHeight(40)
            input_field.setAlignment(Qt.AlignmentFlag.AlignCenter)
            input_field.setValidator(QDoubleValidator())
            input_field.setStyleSheet("""
                QLineEdit {
                    padding: 6px;
                    border: 2px solid #8e24aa;
                    border-radius: 4px;
                    background-color: white;
                }
                QLineEdit:focus {
                    border: 2px solid #7b1fa2;
                }
            """)

            angle_layout.addWidget(label)
            angle_layout.addWidget(input_field)
            ori_layout.addLayout(angle_layout)

            self.pose_inputs[angle] = input_field
            

        ik_layout.addLayout(ori_layout)


        # Connect radio buttons
        self.ik_pose_radio.toggled.connect(self.toggle_ik_input_mode)

        # ========== Add both widgets to main group ==========
        layout.addWidget(self.fk_frame_widget)
        layout.addWidget(self.ik_pose_widget)

        # Initially hide IK widget
        self.ik_pose_widget.setVisible(False)
        return group

    def update_fk_spinbox_state(self, text):
        is_range = self.fk_range_radio.isChecked()
        self.fk_from_spin.setEnabled(is_range)
        self.fk_to_spin.setEnabled(is_range)
        for btn in self.fk_from_buttons:
            btn.setEnabled(is_range)

        for btn in self.fk_to_buttons:
            btn.setEnabled(is_range)
        if is_range:
             self.validate_frame_range()

    def create_spin_with_buttons(self, min_v, max_v, value):
        container = QWidget()
        layout = QHBoxLayout(container)
        controls_layout = QVBoxLayout()
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(4)

        spin = QSpinBox()
        spin.setRange(min_v, max_v)
        spin.setValue(value)
        spin.setFixedWidth(70)
        

        plus = QPushButton("+")
        minus = QPushButton("–")


        for btn in (plus, minus):
            btn.setFixedSize(30, 15)
            btn.setCursor(Qt.CursorShape.PointingHandCursor)
            btn.setStyleSheet("""
                QPushButton {
                    background-color: #ffebee;
                    border-radius: 4px;
                    border: 1px solid #00897b;
                    font-weight: bold;
                }
                QPushButton:hover {
                    background-color: #ffcdd2;
                }
                QPushButton:pressed {
                    background-color: #8e24aa;
                    color: white;
                }
                QPushButton:disabled {
                    background-color: #f0f0f0;
                    color: #999999;
                }
            """)

        plus.clicked.connect(spin.stepUp)
        minus.clicked.connect(spin.stepDown)

        layout.addWidget(spin)
        controls_layout.addWidget(plus)
        controls_layout.addWidget(minus)
        layout.addLayout(controls_layout)

        return container, spin, (plus, minus)

#Validate that From <= To and show visual feedback
    def validate_frame_range(self):
        from_val = self.fk_from_spin.value()
        to_val = self.fk_to_spin.value()

        # Update spinbox styling based on validity
        if from_val >= to_val:
            # Invalid range - show error styling
            error_style = """        
                QSpinBox {
                    background-color: #ffebee;
                    border: 1px solid #e53935;
                }
            """
            self.fk_from_spin.setStyleSheet(error_style)
            self.fk_to_spin.setStyleSheet(error_style)
            return False
        else:
            # Valid range - show normal styling
            normal_style = """
"""
            self.fk_from_spin.setStyleSheet(normal_style)
            self.fk_to_spin.setStyleSheet(normal_style)
            return True
        
    def hide_frame_selector(self):
        kinematics_type = self.ik_fk_widget.currentRow()
        
        is_fk = (kinematics_type == 0)
        self.fk_frame_widget.setVisible(is_fk)
        self.ik_pose_widget.setVisible(not is_fk)
    
    """Execute FK or IK calculation based on current settings"""
    def execute_calculation(self):
        if self.current_manipulator is None:
            QMessageBox.warning(self, "No Robot Selected", 
                              "Please select a manipulator first.")
            return

        calc_mode = self.ik_fk_widget.currentRow()  # 0=FK, 1=IK
        comp_mode = self.sym_num_widget.currentRow()  # 0=Symbolic, 1=Numeric

        try:
            if calc_mode == 0:  # Forward Kinematics
               self.do_fk()
            else:  # Inverse Kinematics
                self.do_ik()
                # self.current_T = self.read_T_matrix_from_table()
                # self.current_manipulator.do_ik(self)
        except Exception as e:
            QMessageBox.critical(self, "Calculation Error", 
                               f"An error occurred:\n{str(e)}")
    
    
    def do_fk(self):
        comp_mode = self.sym_num_widget.currentRow()

        sym = (comp_mode == 0)

        joint_values = []
        
        unit = self.theta_system_widget.currentRow()
        for row in range(self.dh_table.rowCount()):
            item = self.dh_table.item(row, 6)  # Value column
            if item is None or item.text().strip() == "":
                raise ValueError(f"Joint {row + 1} value is empty")

            value = float(item.text())

            # Convert degrees to radians if needed
            params = self.current_manipulator.get_dh_parameters()[row]
            if params['variable'] == 'theta' and unit == 1:  # degrees
                value = np.deg2rad(value)

            joint_values.append(value)

        if self.fk_all_radio.isChecked():
            frame_range = None
        else:
            start = self.fk_from_spin.value()
            end = self.fk_to_spin.value()
            if start >= end:
                QMessageBox.warning(self, "Invalid Range", 
                                  "Start frame must be less than end frame.")
                return
            frame_range = (start, end)

        if sym:
            individual_Ts, cumulative_Ts, final_T, q_symbols = self.current_manipulator.fk_all(joint_values, sym=True)
            display_fk_symbolic_results(self, individual_Ts, cumulative_Ts, final_T, q_symbols, frame_range)
        else:
            individual_Ts, cumulative_Ts, final_T = self.current_manipulator.fk_all(joint_values, sym=False)
            display_fk_numeric_results(self, individual_Ts, cumulative_Ts, final_T, joint_values, frame_range)
        self.tabs.setCurrentIndex(1)

    def read_pose_from_inputs(self):
        """Read x, y, z, α, β, γ from pose input fields and convert to T matrix"""
        try:
            x = float(self.pose_inputs['X'].text())
            y = float(self.pose_inputs['Y'].text())
            z = float(self.pose_inputs['Z'].text())
            alpha = float(self.pose_inputs['alpha'].text())
            beta = float(self.pose_inputs['beta'].text())
            gamma = float(self.pose_inputs['gamma'].text())

            # Convert angles to radians if needed
            angle_unit = self.theta_system_widget.currentRow()
            if angle_unit == 1:  # degrees
                alpha = np.deg2rad(alpha)
                beta = np.deg2rad(beta)
                gamma = np.deg2rad(gamma)

            T = self.current_manipulator.pose_to_matrix(x, y, z, alpha, beta, gamma)
            return T

        except ValueError as e:
            raise ValueError(f"Invalid pose input: {e}")
        except KeyError as e:
            raise ValueError(f"Missing pose input field: {e}")

    def do_ik(self):
        """Execute inverse kinematics"""
        comp_mode = self.sym_num_widget.currentRow()

        if comp_mode == 0: 
            eq_list , T_symbolic = self.current_manipulator.do_ik_symbolic()
            display_ik_symbolic_results(self, T_symbolic, eq_list)
            self.tabs.setCurrentIndex(1)
            return

        # Get target matrix based on input mode
        if self.ik_pose_radio.isChecked():
            try:
                self.current_T = self.read_pose_from_inputs()
            except ValueError as e:
                QMessageBox.warning(self, "Input Error", str(e))
                return
        else:
            self.current_T = self.read_T_matrix_from_table()
        # Call appropriate IK method based on robot type
        try:
            robot_name = self.current_manipulator.name

            if robot_name == "UR5":
                solutions = self.current_manipulator.ik_ur5_closed_form(self.current_T)
            elif robot_name == "ABB_IRB_1600":
                solutions = self.current_manipulator.ik_irb1600_closed_form(self.current_T)
            elif robot_name == "KUKA_KR16":
                solutions = self.current_manipulator.ik_kuka_kr16_closed_form(self.current_T)
                return
            else:
                QMessageBox.warning(self, "IK Not Implemented", 
                                  f"IK not yet implemented for {robot_name}")
                return

        except Exception as e:
            QMessageBox.critical(self, "IK Error", f"Error computing IK: {str(e)}")
            return
           
        if not solutions:
            display_ik_no_solution(self, self.current_T)
        else:
            display_ik_numeric_results(self, solutions, self.current_T)

        self.tabs.setCurrentIndex(1)
        
        
    def create_cad_control_group(self, title):
        """Create a styled control group for CAD tab"""
        group = QGroupBox(title)
        group.setFont(self.label_font)
        group.setStyleSheet("""
            QGroupBox {
                border: 2px solid #8e24aa;
                border-radius: 8px;
                margin-top: 10px;
                padding-top: 15px;
                background-color: #f9f9f9;
                font-weight: bold;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                subcontrol-position: top center;
                padding: 5px 10px;
                background-color: white;
                border-radius: 4px;
                color: #8e24aa;
            }
        """)
        layout = QVBoxLayout()
        layout.setSpacing(8)
        group.setLayout(layout)
        return group
        

    def create_cad_control_panel(self):
        """Create control panel for CAD viewer with custom view controls"""
        container = QWidget()
        main_layout = QVBoxLayout(container)
        main_layout.setSpacing(10)
        main_layout.setContentsMargins(10, 10, 10, 10)

        container.setStyleSheet("""
            QWidget {
                background-color: #f9f9f9;
            }
        """)

        # ============ Model Selection Group ============
        model_group = self.create_cad_control_group("Model Selection")
        model_layout = QVBoxLayout()

        model_label = QLabel("Select Robot Model:")
        model_label.setFont(self.standard_font)
        model_label.setStyleSheet("border: none; background: transparent; color: #333;")
        model_layout.addWidget(model_label)

        self.cad_model_combo = self.create_selector("Select a manipulator:", ["UR5", "ABB IRB 1600", "KUKA KR16"])
        # self.cad_model_combo.setFont(self.standard_font)
        # self.cad_model_combo.addItems(["UR5", "ABB IRB 1600", "KUKA KR16"])
        # self.cad_model_combo.setStyleSheet("""
        #     QComboBox {
        #         padding: 8px;
        #         border: 2px solid #8e24aa;
        #         border-radius: 5px;
        #         background-color: white;
        #     }
        #     QComboBox:hover {
        #         border: 2px solid #7b1fa2;
        #     }
        #     QComboBox::drop-down {
        #         border: none;
        #     }
        #     QComboBox::down-arrow {
        #         image: none;
        #         border-left: 5px solid transparent;
        #         border-right: 5px solid transparent;
        #         border-top: 5px solid #8e24aa;
        #         margin-right: 10px;
        #     }
        # """)
        self.cad_model_combo.currentIndexChanged.connect(self.load_cad_model)
        model_layout.addWidget(self.cad_model_combo)

        model_group.layout().addLayout(model_layout)
        main_layout.addWidget(model_group)

        # ============ View Presets Group ============
        presets_group = self.create_cad_control_group("View Presets")
        presets_layout = QVBoxLayout()
        presets_layout.setSpacing(8)

        preset_buttons = [
            ("Isometric", 35, 45, -3),
            ("Top", 90, 0, -3),
            ("Front", 0, 0, -3),
            ("Side", 0, 90, -3)
        ]

        for name, elev, azim, zoom in preset_buttons:
            btn = QPushButton(name)
            btn.setFont(self.standard_font)
            btn.setCursor(Qt.CursorShape.PointingHandCursor)
            btn.setFixedHeight(40)
            btn.setStyleSheet("""
                QPushButton {
                    background-color: white;
                    color: #8e24aa;
                    border: 2px solid #8e24aa;
                    border-radius: 5px;
                    padding: 8px;
                    font-weight: 600;
                }
                QPushButton:hover {
                    background-color: #f3e5f5;
                }
                QPushButton:pressed {
                    background-color: #8e24aa;
                    color: white;
                }
            """)
            btn.clicked.connect(lambda checked, e=elev, a=azim, z=zoom: 
                              self.view3d_widget.set_view(e, a, z))
            presets_layout.addWidget(btn)

        presets_group.layout().addLayout(presets_layout)
        main_layout.addWidget(presets_group)

        # ============ Custom View Group ============
        custom_group = self.create_cad_control_group("Custom View")
        custom_layout = QVBoxLayout()
        custom_layout.setSpacing(10)

        # Elevation control
        elev_layout = QHBoxLayout()
        elev_label = QLabel("Elevation:")
        elev_label.setFont(self.standard_font)
        elev_label.setStyleSheet("border: none; background: transparent; color: #333;")
        elev_label.setFixedWidth(80)

        self.elev_spin = QSpinBox()
        self.elev_spin.setFont(self.standard_font)
        self.elev_spin.setRange(-180, 180)
        self.elev_spin.setValue(35)
        self.elev_spin.setSuffix("°")
        self.elev_spin.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.elev_spin.setStyleSheet("""
            QSpinBox {
                padding: 6px;
                border: 2px solid #00897b;
                border-radius: 4px;
                background-color: white;
            }
            QSpinBox:focus {
                border: 2px solid #00695c;
            }
        """)

        elev_layout.addWidget(elev_label)
        elev_layout.addWidget(self.elev_spin)
        custom_layout.addLayout(elev_layout)

        # Azimuth control
        azim_layout = QHBoxLayout()
        azim_label = QLabel("Azimuth:")
        azim_label.setFont(self.standard_font)
        azim_label.setStyleSheet("border: none; background: transparent; color: #333;")
        azim_label.setFixedWidth(80)

        self.azim_spin = QSpinBox()
        self.azim_spin.setFont(self.standard_font)
        self.azim_spin.setRange(-180, 180)
        self.azim_spin.setValue(45)
        self.azim_spin.setSuffix("°")
        self.azim_spin.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.azim_spin.setStyleSheet("""
            QSpinBox {
                padding: 6px;
                border: 2px solid #00897b;
                border-radius: 4px;
                background-color: white;
            }
            QSpinBox:focus {
                border: 2px solid #00695c;
            }
        """)

        azim_layout.addWidget(azim_label)
        azim_layout.addWidget(self.azim_spin)
        custom_layout.addLayout(azim_layout)

        # Set View button
        set_view_btn = QPushButton("Set View")
        set_view_btn.setFont(self.standard_font)
        set_view_btn.setCursor(Qt.CursorShape.PointingHandCursor)
        set_view_btn.setFixedHeight(40)
        set_view_btn.setStyleSheet("""
            QPushButton {
                background-color: #00897b;
                color: white;
                border: 2px solid #00897b;
                border-radius: 5px;
                padding: 8px;
                font-weight: 600;
            }
            QPushButton:hover {
                background-color: #00695c;
                border-color: #00695c;
            }
            QPushButton:pressed {
                background-color: #004d40;
            }
        """)
        set_view_btn.clicked.connect(self.set_custom_view)
        custom_layout.addWidget(set_view_btn)

        custom_group.layout().addLayout(custom_layout)
        main_layout.addWidget(custom_group)

        # ============ Zoom Controls Group ============
        zoom_group = self.create_cad_control_group("Zoom Controls")
        zoom_layout = QVBoxLayout()
        zoom_layout.setSpacing(8)

        zoom_buttons_layout = QHBoxLayout()
        zoom_buttons_layout.setSpacing(8)

        zoom_in_btn = QPushButton("Zoom In (+)")
        zoom_out_btn = QPushButton("Zoom Out (−)")
        zoom_reset_btn = QPushButton("Reset Zoom")

        for btn in [zoom_in_btn, zoom_out_btn, zoom_reset_btn]:
            btn.setFont(self.standard_font)
            btn.setCursor(Qt.CursorShape.PointingHandCursor)
            btn.setFixedHeight(40)
            btn.setStyleSheet("""
                QPushButton {
                    background-color: white;
                    color: #8e24aa;
                    border: 2px solid #8e24aa;
                    border-radius: 5px;
                    padding: 8px;
                    font-weight: 600;
                }
                QPushButton:hover {
                    background-color: #f3e5f5;
                }
                QPushButton:pressed {
                    background-color: #8e24aa;
                    color: white;
                }
            """)

        zoom_in_btn.clicked.connect(lambda: self.zoom_cad_view(0.5))
        zoom_out_btn.clicked.connect(lambda: self.zoom_cad_view(-0.5))
        zoom_reset_btn.clicked.connect(lambda: self.view3d_widget.set_view(
            self.view3d_widget.rotation_x, 
            self.view3d_widget.rotation_y, 
            -3.0
        ))

        zoom_layout.addWidget(zoom_in_btn)
        zoom_layout.addWidget(zoom_out_btn)
        zoom_layout.addWidget(zoom_reset_btn)

        zoom_group.layout().addLayout(zoom_layout)
        main_layout.addWidget(zoom_group)

        # ============ Display Options Group ============
        display_group = self.create_cad_control_group("Display Options")
        display_layout = QVBoxLayout()
        display_layout.setSpacing(8)

        self.cad_edges_check = QCheckBox("Show Edges")
        self.cad_edges_check.setChecked(True)
        self.cad_edges_check.setFont(self.standard_font)
        self.cad_edges_check.setCursor(Qt.CursorShape.PointingHandCursor)
        self.cad_edges_check.setStyleSheet("""
            QCheckBox {
                spacing: 8px;
                color: #333;
            }
            QCheckBox::indicator {
                width: 20px;
                height: 20px;
                border: 2px solid #8e24aa;
                border-radius: 4px;
                background-color: white;
            }
            QCheckBox::indicator:checked {
                background-color: #8e24aa;
                image: none;
            }
            QCheckBox::indicator:checked:after {
                content: "✓";
                color: white;
            }
        """)
        self.cad_edges_check.toggled.connect(self.view3d_widget.toggle_edges)

        self.cad_wireframe_check = QCheckBox("Wireframe Mode")
        self.cad_wireframe_check.setFont(self.standard_font)
        self.cad_wireframe_check.setCursor(Qt.CursorShape.PointingHandCursor)
        self.cad_wireframe_check.setStyleSheet(self.cad_edges_check.styleSheet())
        self.cad_wireframe_check.toggled.connect(self.view3d_widget.toggle_wireframe)

        display_layout.addWidget(self.cad_edges_check)
        display_layout.addWidget(self.cad_wireframe_check)

        display_group.layout().addLayout(display_layout)
        main_layout.addWidget(display_group)

        # ============ Info Display ============
        self.cad_info_label = QLabel("Left-drag: Rotate | Right-drag/Wheel: Zoom")
        self.cad_info_label.setFont(self.standard_font)
        self.cad_info_label.setWordWrap(True)
        self.cad_info_label.setStyleSheet("""
            QLabel {
                background-color: #e1f5fe;
                color: #01579b;
                padding: 10px;
                border-radius: 5px;
                border: 1px solid #81d4fa;
            }
        """)
        main_layout.addWidget(self.cad_info_label)

        main_layout.addStretch()

        # Load initial model
        self.load_cad_model(0)

        return container

    def load_cad_model(self, index):
        """Load CAD model into viewer"""
        models = [
            'cad_models/ur5.obj',
            'cad_models/irb_1600_10kg_1.45m.obj',
            'cad_models/ur5.obj'  # Placeholder for KUKA
        ]

        filepath = models[index]
        success = self.view3d_widget.load_model(filepath)

        if success and self.view3d_widget.mesh:
            self.cad_info_label.setText(
                f"Loaded: {filepath.split('/')[-1]} | "
                f"Vertices: {len(self.view3d_widget.mesh.vertices):,} | "
                f"Faces: {len(self.view3d_widget.mesh.faces):,}"
            )
        else:
            self.cad_info_label.setText(f"Failed to load {filepath}")
            QMessageBox.warning(self, "Load Error", 
                              f"Could not load model: {filepath}")

    def set_custom_view(self):
        """Set custom view from spinbox values"""
        elev = self.elev_spin.value()
        azim = self.azim_spin.value()
        self.view3d_widget.set_view(elev, azim)

    def zoom_cad_view(self, delta):
        """Zoom in or out by specified amount"""
        self.view3d_widget.target_zoom += delta
        self.view3d_widget.target_zoom = max(-20.0, min(-1.0, self.view3d_widget.target_zoom))  










            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
     
    
    
    
    
    
    
    
    











