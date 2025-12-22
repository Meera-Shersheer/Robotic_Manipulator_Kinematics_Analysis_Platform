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
        self.standard_font = QFont("Roboto", 13)  # or "Roboto", 11
        self.label_font = QFont("Roboto", 15)
        
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
        inputs_section.addLayout(controls_widget, 2)
       
        minpulator_chose_box, self.manipulator_list_widget = self.manipulator_list()
        ik_fk, self.ik_fk_widget = self.ik_fk_selector()
        sym_num, self.sym_num_widget = self.sym_num_selector()
        theta_system, self.theta_system_widget = self.rad_deg_selector()

        fram_range = QVBoxLayout()
        inputs_section.addLayout(fram_range, 1)
        self.frame_range_selector = self.create_fk_frame_selector()
        
        controls_widget.addWidget(minpulator_chose_box)
        controls_widget.addWidget(ik_fk)
        controls_widget.addWidget(sym_num)
        controls_widget.addWidget(theta_system)
        fram_range.addWidget(self.frame_range_selector)
   #     controls_layout = QHBoxLayout()
        #controls_widget.setLayout(controls_layout)
   #     controls_widget.layout.addLayout(controls_layout)
        
        # Row 2: DH Table
        #dh_widget = QWidget()
        tables_matrix_Row = QHBoxLayout()
        dh_widget = self.create_dh_table_widget()
        matrix_Widget = self.create_matrix_display_widget()
        #Color("#38edde", "T-matrix")
        #Color("blue", "dh_widget")
        # dh_layout = QVBoxLayout()
        #dh_widget.setLayout(dh_layout)
        
        # Row 3: Outputs
       # output_widget = QWidget()
       # output_widget = Color("orange", "OUTPUT")
        #Color("purple", "execute")
    #    output_layout = QVBoxLayout()
        #output_widget.setLayout(output_layout)
        # inputs_section.addWidget(controls_widget, 1)
     ##   inputs_section.addWidget(dh_widget, 5)  
#        left_widget.addWidget(controls_widget, 1)
        # left_widget.addWidget(control_dh_widget, 3)
        tables_matrix_Row.addWidget(dh_widget, 6) 
        tables_matrix_Row.addWidget(matrix_Widget, 4)   
        inputs_section.addLayout(tables_matrix_Row, 8) 
        
        execute_widget = self.create_execute_widget()
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
        output_widget = self.create_output_widget()
        #Color("orange", "OUTPUT")
        output_layout.addWidget(output_widget, 6)
        self.tabs.addTab(output_tab, "Outputs")
        outer_layout.addWidget(self.tabs)
        
        self.toggle_value_column()
        self.hide_matrix()


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
                padding: 6px;
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
            "when Numeric Inverse Kinematics is selected."
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
        for i in range(4):
            for j in range(4):
                item = QTableWidgetItem("0.0000")
                item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
                item.setFlags(
                    Qt.ItemFlag.ItemIsEnabled | Qt.ItemFlag.ItemIsEditable)
                self.matrix_table.setItem(i, j, item)
      
        self.sym_num_widget.currentRowChanged.connect(self.hide_matrix)
        self.ik_fk_widget.currentRowChanged.connect(self.hide_matrix)
        layout.addWidget(self.matrix_table)
        return widget

    #Show/hide the T matrix based on mode mode
    def hide_matrix(self):
        computation_mode = self.sym_num_widget.currentRow()
        kinematics_type = self.ik_fk_widget.currentRow()
        
        show_matrix = (computation_mode == 1 and kinematics_type == 1)
        self.matrix_table.setVisible(show_matrix)
        self.matrix_placeholder.setVisible(not show_matrix)

            
        # Create execute button widget
    def create_execute_widget(self):
        widget = QWidget()
        layout = QHBoxLayout(widget)
        layout.setSpacing(5)
        layout.setContentsMargins(5, 5, 5, 5)

        widget.setStyleSheet("""
            QWidget {
                border: #f5f5f5;;
                padding: 5px;
                border-radius: 5px;
                background-color: white;
            }
        """)
        # Execute button
        self.execute_button = QPushButton("Calculate")
        self.label_font.setWeight(QFont.Weight.DemiBold) 
        self.execute_button.setFont(self.label_font)
        self.execute_button.setCursor(Qt.CursorShape.PointingHandCursor)
        self.execute_button.setFixedSize(150, 50)  
        self.execute_button.setStyleSheet("""
            QPushButton {
                background-color: #f9f9f9;
                color: #0078d4; 
                border: 2px solid #cccccc; 
                border-radius: 5px;
                padding: 10px 10px;
            }
            QPushButton:hover {
                background-color: #e5f3ff;
            }
            QPushButton:pressed {
                background-color: #cce8ff;
            }
            QPushButton:disabled {
                background-color: #cccccc;
                color: #666666;
            }
        """)
    #    self.execute_button.clicked.connect(self.execute_calculation)
        layout.addWidget(self.execute_button)
        
        return widget

    def create_output_widget(self):
        """Create output display widget"""
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
        title.setFont(self.label_font)
        title.setStyleSheet("color: #0078d4; padding: 10px;")
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        main_layout.addWidget(title)

        # Create scrollable area
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setStyleSheet("""
            QScrollArea {
                border: 1px solid #cccccc;
                border-radius: 5px;
                background-color: white;
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
    
    def create_fk_frame_selector(self, dof=6):
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
        
        # ---- Title ----
        title = QLabel("Selected Frames Whose Transformation Matrices Will Be Displayed:")
        title.setFont(self.label_font)
        title.setStyleSheet("border: none; background: transparent;")
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(title)

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
        
        self.fk_from_spin = QSpinBox()
        self.fk_from_spin.setRange(0, dof)
        self.fk_from_spin.setValue(0)
    
        self.fk_to_spin = QSpinBox()
        self.fk_to_spin.setRange(0, dof)
        self.fk_to_spin.setValue(dof)
        
        from_label = QLabel("From:")
        to_label = QLabel("To:")
        from_label.setFont(self.standard_font)
        to_label.setFont(self.standard_font)
    
        self.fk_from_spin.setFont(self.standard_font)
        self.fk_from_spin.setFixedWidth(100)
        self.fk_to_spin.setFont(self.standard_font)
        self.fk_to_spin.setFixedWidth(100)
      
        spin_layout.addSpacing(275)  
        spin_layout.addWidget(from_label)
        spin_layout.addSpacing(5)
        spin_layout.addWidget(self.fk_from_spin)
        spin_layout.addSpacing(60)
        spin_layout.addWidget(to_label)
        spin_layout.addSpacing(5)
        spin_layout.addWidget(self.fk_to_spin)
        spin_layout.setAlignment(Qt.AlignmentFlag.AlignCenter)
            # QSpinBox {
            #     padding: 4px;
            # }
        
        self.setStyleSheet("""
            QRadioButton {
                padding: 20px 40px;
                border-radius: 5px;
            }

            QRadioButton::indicator:checked {
                background-color: #0078d4;
                border-radius: 7px;
                border: 2px solid #0078d4;
            }
                     QSpinBox {
                padding: 6px 10px;
                border: 1px solid #cccccc;
                border-radius: 4px;
                background-color: #ffffff;
                min-height: 32px;
            }
         
            QSpinBox::up-button, QSpinBox::down-button {
                width: 24px;
                border-left: 1px solid #cccccc;
                background-color: #f0f0f0;
            }
         
            QSpinBox::up-button:hover, QSpinBox::down-button:hover {
                background-color: #e5f3ff;
            }
         
            QSpinBox::up-arrow {
                image: none;
                width: 0;
                height: 0;
                border-left: 5px solid transparent;
                border-right: 5px solid transparent;
                border-bottom: 7px solid #555555;
            }
         
            QSpinBox::down-arrow {
                image: none;
                width: 0;
                height: 0;
                border-left: 5px solid transparent;
                border-right: 5px solid transparent;
                border-top: 7px solid #555555;
            }
                """)
                
        layout.addLayout(radio_layout)
        layout.addLayout(spin_layout)
        # --- Tooltips ---
        self.fk_all_radio.setToolTip("Display all FK frames")
        self.fk_range_radio.setToolTip("Display only a specific frame range")
        self.fk_from_spin.setToolTip("Start frame (0–6)")
        self.fk_to_spin.setToolTip("End frame (0–6)")
    

        # Initially disable From/To if "All" is selected
        self.fk_from_spin.setEnabled(False)
        self.fk_to_spin.setEnabled(False)
        
        # --- Connections ---
        self.fk_all_radio.toggled.connect(self.update_fk_spinbox_state)
        return group

    def update_fk_spinbox_state(self, text):
        is_range = self.fk_range_radio.isChecked()
        self.fk_from_spin.setEnabled(is_range)
        self.fk_to_spin.setEnabled(is_range)










    # def create_fk_frame_selector(self, dof=6):
    #     """
    #     Create FK frame selection widget with centered title and consistent styling.

    #     Args:
    #         dof: Degrees of freedom (number of joints)

    #     Returns:
    #         QWidget containing the frame selector
    #     """
    #     # Main container widget
    #     widget = QWidget()
    #     main_layout = QVBoxLayout(widget)
    #     main_layout.setSpacing(5)
    #     main_layout.setContentsMargins(5, 5, 5, 5)

    #     # Apply consistent widget styling
    #     widget.setStyleSheet("""
    #         QWidget {
    #             border: 1px solid #cccccc;
    #             padding: 5px;
    #             border-radius: 5px;
    #             background-color: #f9f9f9;
    #         }
    #     """)

    #     # Title label - centered
    #     title_label = QLabel("FK Frame Selection")
    #     title_label.setFont(self.label_font)
    #     title_label.setStyleSheet("border: none; background: transparent;")
    #     title_label.setAlignment(Qt.AlignmentFlag.AlignHCenter | Qt.AlignmentFlag.AlignVCenter)
    #     main_layout.addWidget(title_label)

    #     # Content container for radio buttons and spinboxes
    #     content_widget = QWidget()
    #     content_widget.setStyleSheet("QWidget { border: none; background: transparent; }")
    #     content_layout = QVBoxLayout(content_widget)
    #     content_layout.setSpacing(10)
    #     content_layout.setContentsMargins(10, 5, 10, 5)

    #     # --- Radio Button Section ---
    #     radio_container = QWidget()
    #     radio_container.setStyleSheet("QWidget { border: none; background: transparent; }")
    #     radio_layout = QHBoxLayout(radio_container)
    #     radio_layout.setContentsMargins(0, 0, 0, 0)

    #     # Add stretch to center the radio buttons
    #     radio_layout.addStretch(1)

    #     self.fk_all_radio = QRadioButton("All frames")
    #     self.fk_all_radio.setFont(self.standard_font)
    #     self.fk_all_radio.setChecked(True)  # default
    #     self.fk_all_radio.setToolTip("Display all FK transformation matrices (T₀¹ to T₀⁶)")
    #     radio_layout.addWidget(self.fk_all_radio)

    #     # Spacer between radio buttons
    #     radio_layout.addSpacing(20)

    #     self.fk_range_radio = QRadioButton("Custom range")
    #     self.fk_range_radio.setFont(self.standard_font)
    #     self.fk_range_radio.setToolTip("Display only specific frames in the selected range")
    #     radio_layout.addWidget(self.fk_range_radio)

    #     radio_layout.addStretch(1)

    #     # Style radio buttons
    #     radio_style = """
    #         QRadioButton {
    #             border: none;
    #             background: transparent;
    #             padding: 5px;
    #         }
    #         QRadioButton::indicator {
    #             width: 16px;
    #             height: 16px;
    #         }
    #         QRadioButton::indicator::unchecked {
    #             border: 2px solid #0078d4;
    #             border-radius: 8px;
    #             background: white;
    #         }
    #         QRadioButton::indicator::checked {
    #             border: 2px solid #0078d4;
    #             border-radius: 8px;
    #             background: #0078d4;
    #         }
    #         QRadioButton::indicator::unchecked:hover {
    #             background: #e5f3ff;
    #         }
    #     """
    #     self.fk_all_radio.setStyleSheet(radio_style)
    #     self.fk_range_radio.setStyleSheet(radio_style)

    #     content_layout.addWidget(radio_container)

    #     # --- Spinbox Section ---
    #     spin_container = QWidget()
    #     spin_container.setStyleSheet("QWidget { border: none; background: transparent; }")
    #     spin_layout = QHBoxLayout(spin_container)
    #     spin_layout.setContentsMargins(0, 0, 0, 0)
    #     spin_layout.setSpacing(10)

    #     # Add stretch to center the spinboxes
    #     spin_layout.addStretch(1)

    #     # From label and spinbox
    #     from_label = QLabel("From:")
    #     from_label.setFont(self.standard_font)
    #     from_label.setStyleSheet("border: none; background: transparent;")
    #     spin_layout.addWidget(from_label)

    #     self.fk_from_spin = QSpinBox()
    #     self.fk_from_spin.setRange(0, dof)
    #     self.fk_from_spin.setValue(0)
    #     self.fk_from_spin.setFont(self.standard_font)
    #     self.fk_from_spin.setEnabled(False)  # Initially disabled
    #     self.fk_from_spin.setToolTip(f"Start frame (0 to {dof})")
    #     self.fk_from_spin.setMinimumWidth(70)
    #     self.fk_from_spin.setStyleSheet("""
    #         QSpinBox {
    #             border: 2px solid #cccccc;
    #             border-radius: 3px;
    #             padding: 5px;
    #             background-color: white;
    #         }
    #         QSpinBox:focus {
    #             border: 2px solid #0078d4;
    #         }
    #         QSpinBox:disabled {
    #             background-color: #f0f0f0;
    #             color: #999999;
    #             border: 2px solid #e0e0e0;
    #         }
    #     """)
    #     spin_layout.addWidget(self.fk_from_spin)

    #     # Spacer between From and To
    #     spin_layout.addSpacing(15)

    #     # To label and spinbox
    #     to_label = QLabel("To:")
    #     to_label.setFont(self.standard_font)
    #     to_label.setStyleSheet("border: none; background: transparent;")
    #     spin_layout.addWidget(to_label)

    #     self.fk_to_spin = QSpinBox()
    #     self.fk_to_spin.setRange(0, dof)
    #     self.fk_to_spin.setValue(dof)
    #     self.fk_to_spin.setFont(self.standard_font)
    #     self.fk_to_spin.setEnabled(False)  # Initially disabled
    #     self.fk_to_spin.setToolTip(f"End frame (0 to {dof})")
    #     self.fk_to_spin.setMinimumWidth(70)
    #     self.fk_to_spin.setStyleSheet("""
    #         QSpinBox {
    #             border: 2px solid #cccccc;
    #             border-radius: 3px;
    #             padding: 5px;
    #             background-color: white;
    #         }
    #         QSpinBox:focus {
    #             border: 2px solid #0078d4;
    #         }
    #         QSpinBox:disabled {
    #             background-color: #f0f0f0;
    #             color: #999999;
    #             border: 2px solid #e0e0e0;
    #         }
    #     """)
    #     spin_layout.addWidget(self.fk_to_spin)

    #     spin_layout.addStretch(1)

    #     content_layout.addWidget(spin_container)

    #     # Add content to main layout
    #     main_layout.addWidget(content_widget)

    #     # --- Connections ---
    #     self.fk_all_radio.toggled.connect(self.update_fk_spinbox_state)
    #     self.fk_from_spin.valueChanged.connect(self.validate_fk_range)
    #     self.fk_to_spin.valueChanged.connect(self.validate_fk_range)

    #     # Store DOF for later updates
    #     self.fk_max_frames = dof

    #     return widget


    # def update_fk_spinbox_state(self, checked):
    #     """Enable/disable spinboxes based on radio button selection"""
    #     is_range = self.fk_range_radio.isChecked()
    #     self.fk_from_spin.setEnabled(is_range)
    #     self.fk_to_spin.setEnabled(is_range)

    #     # If switching to range mode, validate current values
    #     if is_range:
    #         self.validate_fk_range()


    # def validate_fk_range(self):
    #     """Validate that From <= To and show visual feedback"""
    #     from_val = self.fk_from_spin.value()
    #     to_val = self.fk_to_spin.value()

    #     # Update spinbox styling based on validity
    #     if from_val > to_val:
    #         # Invalid range - show error styling
    #         error_style = """
    #             QSpinBox {
    #                 border: 2px solid #d32f2f;
    #                 border-radius: 3px;
    #                 padding: 5px;
    #                 background-color: #ffebee;
    #             }
    #             QSpinBox:focus {
    #                 border: 2px solid #d32f2f;
    #             }
    #         """
    #         self.fk_from_spin.setStyleSheet(error_style)
    #         self.fk_to_spin.setStyleSheet(error_style)
    #         return False
    #     else:
    #         # Valid range - show normal styling
    #         normal_style = """
    #             QSpinBox {
    #                 border: 2px solid #cccccc;
    #                 border-radius: 3px;
    #                 padding: 5px;
    #                 background-color: white;
    #             }
    #             QSpinBox:focus {
    #                 border: 2px solid #0078d4;
    #             }
    #             QSpinBox:disabled {
    #                 background-color: #f0f0f0;
    #                 color: #999999;
    #                 border: 2px solid #e0e0e0;
    #             }
    #         """
    #         self.fk_from_spin.setStyleSheet(normal_style)
    #         self.fk_to_spin.setStyleSheet(normal_style)
    #         return True


    # def get_fk_frame_selection(self):
    #     """
    #     Get the current FK frame selection.

    #     Returns:
    #         tuple: (display_all: bool, from_frame: int, to_frame: int)
    #                or None if range is invalid
    #     """
    #     if self.fk_all_radio.isChecked():
    #         return (True, 0, self.fk_max_frames)
    #     else:
    #         from_val = self.fk_from_spin.value()
    #         to_val = self.fk_to_spin.value()

    #         if from_val > to_val:
    #             return None  # Invalid range

    #         return (False, from_val, to_val)


    # def update_fk_max_frames(self, max_frames):
    #     """Update the maximum frame value when manipulator changes"""
    #     self.fk_max_frames = max_frames
    #     self.fk_from_spin.setMaximum(max_frames)
    #     self.fk_to_spin.setMaximum(max_frames)

    #     # Update tooltips
    #     self.fk_from_spin.setToolTip(f"Start frame (0 to {max_frames})")
    #     self.fk_to_spin.setToolTip(f"End frame (0 to {max_frames})")

    #     # Update radio button text to show current max
    #     self.fk_all_radio.setText(f"All frames (0 to {max_frames})")

    #     # Reset to default values
    #     self.fk_to_spin.setValue(max_frames)
































#    if self.fk_all_radio.isChecked():
#     frame_range = range(1, self.current_manipulator.dof + 1)
# else:
#     start = self.fk_from_spin.value()
#     end = self.fk_to_spin.value()

#     if start > end:
#         QMessageBox.warning(self, "Invalid Range", "'From' must be ≤ 'To'")
#         return

#     frame_range = range(start, end + 1)
 
    
    # def execute_calculation(self):
    #     """Execute FK or IK calculation based on selection"""
    #     if self.current_manipulator is None:
    #         QMessageBox.warning(self, "No Robot Selected", 
    #                           "Please select a manipulator first.")
    #         return

    #     calc_mode = self.ik_fk_widget.currentRow()  # 0=FK, 1=IK
    #     comp_mode = self.sym_num_widget.currentRow()  # 0=Symbolic, 1=Numeric

    #     if calc_mode == 0:  # Forward Kinematics
    #         if comp_mode == 0:  # Symbolic
    #             self.execute_fk_symbolic()
    #         else:  # Numeric
    #             self.execute_fk_numeric()
    #     else:  # Inverse Kinematics
    #         QMessageBox.information(self, "Coming Soon", 
    #                                "Inverse Kinematics will be implemented next!")

    # def execute_fk_numeric(self):
    #     """Execute numeric forward kinematics"""
    #     try:
    #         # Get joint values
    #         joint_values = self.get_joint_values()
            
    #         if joint_values is None:
    #             QMessageBox.warning(self, "Invalid Mode", 
    #                               "Numeric mode required for numeric calculation.")
    #             return

    #         # Compute FK
    #         transforms, T_final = self.fk_engine.compute_numeric(joint_values)
            
    #         # Update matrix display
    #         self.update_matrix_display(T_final)
            
    #         # Format and display results
    #         results = self.fk_engine.format_results_numeric(transforms, T_final)
    #         self.display_fk_results(results, joint_values)
            
    #         # Switch to output tab
    #         self.tabs.setCurrentIndex(1)
            
    #         QMessageBox.information(self, "Success", 
    #                                "Forward Kinematics calculated successfully!")
            
    #     except ValueError as e:
    #         QMessageBox.warning(self, "Input Error", str(e))
    #     except Exception as e:
    #         QMessageBox.critical(self, "Calculation Error", 
    #                            f"An error occurred: {str(e)}")

    # def execute_fk_symbolic(self):
    #     """Execute symbolic forward kinematics"""
    #     try:
    #         # Compute symbolic FK
    #         transforms, T_final = self.fk_engine.compute_symbolic()
            
    #         # Display symbolic results
    #         self.display_fk_symbolic_results(transforms, T_final)
            
    #         # Switch to output tab
    #         self.tabs.setCurrentIndex(1)
            
    #         QMessageBox.information(self, "Success", 
    #                                "Symbolic Forward Kinematics computed!")
            
    #     except Exception as e:
    #         QMessageBox.critical(self, "Calculation Error", 
    #                            f"An error occurred: {str(e)}")

    # def update_matrix_display(self, T):
    #     """Update the transformation matrix display"""
    #     for i in range(4):
    #         for j in range(4):
    #             item = QTableWidgetItem(f"{T[i, j]:.6f}")
    #             item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
    #             item.setFlags(Qt.ItemFlag.ItemIsEnabled)
                
    #             # Highlight rotation vs translation
    #             if i < 3 and j < 3:
    #                 item.setBackground(QColor("#e3f2fd"))  # Rotation (blue tint)
    #             elif j == 3 and i < 3:
    #                 item.setBackground(QColor("#fff3e0"))  # Translation (orange tint)
    #             else:
    #                 item.setBackground(QColor("#f5f5f5"))  # Bottom row
                    
    #             self.matrix_table.setItem(i, j, item)

    # def display_fk_results(self, results, joint_values):
    #     """Display FK results in output panel"""
    #     # Clear previous results
    #     while self.output_layout.count():
    #         child = self.output_layout.takeAt(0)
    #         if child.widget():
    #             child.widget().deleteLater()

    #     # Joint values section
    #     joint_group = self.create_result_group("Input Joint Values")
    #     joint_layout = QVBoxLayout()
        
    #     angle_unit = "deg" if self.theta_system_widget.currentRow() == 1 else "rad"
        
    #     for i, val in enumerate(joint_values):
    #         params = self.current_manipulator.get_dh_parameters()[i]
    #         if params['variable'] == 'theta':
    #             display_val = np.rad2deg(val) if angle_unit == "deg" else val
    #             label = QLabel(f"θ{i+1} = {display_val:.6f} {angle_unit}")
    #         else:
    #             label = QLabel(f"d{i+1} = {val:.6f} m")
    #         label.setFont(self.standard_font)
    #         joint_layout.addWidget(label)
        
    #     joint_group.layout().addLayout(joint_layout)
    #     self.output_layout.addWidget(joint_group)

    #     # End-effector position
    #     pos_group = self.create_result_group("End-Effector Position")
    #     pos_layout = QVBoxLayout()
    #     position = results['position']
        
    #     for axis, val in zip(['X', 'Y', 'Z'], position):
    #         label = QLabel(f"{axis} = {val:.6f} m")
    #         label.setFont(QFont("Roboto", 12))
    #         pos_layout.addWidget(label)
        
    #     pos_group.layout().addLayout(pos_layout)
    #     self.output_layout.addWidget(pos_group)

    #     # End-effector orientation
    #     ori_group = self.create_
    
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




