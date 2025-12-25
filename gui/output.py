from imports import *


def run_output_test(self):
    """Display comprehensive test of all output capabilities"""
    # Clear existing content
    clear_output(self)
    
    # 1. Section Header with styled background
    add_section_header(self,"Forward Kinematics Results", "#0078d4")
    
    # 2. Joint Values Display (Grid Layout)
    add_joint_values_section(self)
    # add_spacing(self, self.output_layout, 15) 
    # 3. Transformation Matrix Display
    add_matrix_section(self)
    # add_spacing(self, self.output_layout, 15) 
    # 4. Position & Orientation Results
    add_pose_section(self)
    # add_spacing(self, self.output_layout, 15)  
    # 5. Symbolic Equations Display
    add_equations_section(self)
    # add_spacing(self, self.output_layout, 15) 
    # 6. Multiple Solutions (for IK)
    add_solutions_section(self)
    # add_spacing(self, self.output_layout, 15) 
    # 7. Error/Warning Messages
    add_status_messages(self)
    # add_spacing(self, self.output_layout, 15) 
    # 8. Collapsible Advanced Section
    add_collapsible_section(self)
    # add_spacing(self, self.output_layout, 15) 
    # 9. Export Options
    add_export_section(self)
    
    # spacer = QWidget()
    # spacer.setFixedHeight(20)  # Fixed height instead of stretch
    # self.output_layout.addWidget(spacer)

def clear_output(self):
    """Remove all widgets from output layout"""
    while self.output_layout.count():
        child = self.output_layout.takeAt(0)
        if child.widget():
            child.widget().deleteLater()

def add_section_header(self, title, color="#6d3d52"):
    """Add a prominent section header"""
    header = QLabel(title)
    header.setFont(QFont("Roboto", 16, QFont.Weight.Bold))
    header.setStyleSheet(f"""
        QLabel {{
            background-color: {color};
            color: white;
            padding: 12px;
            border-radius: 5px;
            margin-top: 10px;
        }}
    """)
    header.setAlignment(Qt.AlignmentFlag.AlignCenter)
    self.output_layout.addWidget(header)

def add_joint_values_section(self):
    """Display joint values in a styled grid"""
    group = create_result_group(self,"📊 Input Joint Values")
    
    # Create grid layout for joint values
    grid = QGridLayout()
    grid.setSpacing(10)
    
    joint_data = [
        ("θ₁", "45.0000°", "#e3f2fd"),
        ("θ₂", "-30.0000°", "#e8f5e9"),
        ("θ₃", "90.0000°", "#fff3e0"),
        ("θ₄", "0.0000°", "#fce4ec"),
        ("θ₅", "-60.0000°", "#f3e5f5"),
        ("θ₆", "120.0000°", "#e0f2f1"),
    ]
    
    for i, (label, value, color) in enumerate(joint_data):
        row = i // 3
        col = (i % 3) * 2
        
        # Label
        lbl = QLabel(label)
        lbl.setFont(QFont("Roboto", 13, QFont.Weight.Bold))
        lbl.setStyleSheet(f"""
            QLabel {{
                background-color: {color};
                padding: 8px;
                border-radius: 3px;
                color: #333;
            }}
        """)
        lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        grid.addWidget(lbl, row, col)
        
        # Value
        val = QLabel(value)
        val.setFont(QFont("Roboto", 13))
        val.setStyleSheet(f"""
            QLabel {{
                background-color: white;
                padding: 8px;
                border: 1px solid {color};
                border-radius: 3px;
            }}
        """)
        val.setAlignment(Qt.AlignmentFlag.AlignCenter)
        grid.addWidget(val, row, col + 1)
    
    group.layout().addLayout(grid)
    self.output_layout.addWidget(group)

def add_matrix_section(self):
    """Display transformation matrix in a formatted table"""
    group = create_result_group(self, "Transformation Matrix T₀⁶")
    
    # Create matrix table
    matrix_table = QTableWidget(4, 4)
    matrix_table.setFont(QFont("Courier New", 11))
    matrix_table.horizontalHeader().setVisible(False)
    matrix_table.verticalHeader().setVisible(False)
    matrix_table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
    
    matrix_table.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Minimum)

    # Sample transformation matrix
    matrix_values = [
        [0.5000, -0.8660, 0.0000, 0.4330],
        [0.8660, 0.5000, 0.0000, 0.2500],
        [0.0000, 0.0000, 1.0000, 0.5000],
        [0.0000, 0.0000, 0.0000, 1.0000]
    ]
    
    for i in range(4):
        for j in range(4):
            item = QTableWidgetItem(f"{matrix_values[i][j]:.4f}")
            item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
            
            # Color code different parts of the matrix
            if i < 3 and j < 3:  # Rotation part
                item.setBackground(QColor("#f6e3fd"))
            elif i < 3 and j == 3:  # Translation part
                item.setBackground(QColor("#fff3e0"))
            else:  # Bottom row
                item.setBackground(QColor("#f5f5f5"))
            
            matrix_table.setItem(i, j, item)
    
    matrix_table.setStyleSheet("""
        QTableWidget {
            border: 2px solid #0078d4;
            border-radius: 5px;
            gridline-color: #ccc;
        }
    """)
    
    # Set uniform cell size
    matrix_table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
    matrix_table.verticalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
    matrix_table.setMinimumHeight(180)  # Change from setMaximumHeight
    matrix_table.setMaximumHeight(220)  # Add this too
    
    group.layout().addWidget(matrix_table)
    
    # Add matrix legend
    legend = QLabel(" Rotation (pink)|  Translation(orange) |  Homogeneous")
    legend.setFont(QFont("Roboto", 10))
    legend.setStyleSheet("color: #666; margin-top: 5px;")
    legend.setAlignment(Qt.AlignmentFlag.AlignCenter)
    group.layout().addWidget(legend)
    
    self.output_layout.addWidget(group)

def add_pose_section(self):
    """Display end-effector pose with visual indicators"""
    group = create_result_group(self, "End-Effector Pose")
    
    # Position
    pos_label = QLabel("Position (meters):")
    pos_label.setFont(QFont("Roboto", 12, QFont.Weight.Bold))
    group.layout().addWidget(pos_label)
    
    pos_layout = QHBoxLayout()
    for axis, value, color in [("X", "0.4330", "#ef5350"), 
                                 ("Y", "0.2500", "#66bb6a"), 
                                 ("Z", "0.5000", "#42a5f5")]:
        axis_widget = QWidget()
        axis_layout = QVBoxLayout(axis_widget)
        axis_layout.setContentsMargins(5, 5, 5, 5)
        
        axis_lbl = QLabel(axis)
        axis_lbl.setFont(QFont("Roboto", 11, QFont.Weight.Bold))
        axis_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        axis_lbl.setStyleSheet(f"color: {color};")
        
        val_lbl = QLabel(value)
        val_lbl.setFont(QFont("Roboto", 13))
        val_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        val_lbl.setStyleSheet(f"""
            QLabel {{
                background-color: {color};
                color: white;
                padding: 8px;
                border-radius: 5px;
                font-weight: bold;
            }}
        """)
        
        axis_layout.addWidget(axis_lbl)
        axis_layout.addWidget(val_lbl)
        pos_layout.addWidget(axis_widget)
    
    group.layout().addLayout(pos_layout)
    
    # Orientation (Euler angles)
    add_spacing(self, group.layout(), 10)
    ori_label = QLabel("Orientation (Roll-Pitch-Yaw):")
    ori_label.setFont(QFont("Roboto", 12, QFont.Weight.Bold))
    group.layout().addWidget(ori_label)
    
    ori_layout = QHBoxLayout()
    for angle, value in [("Roll (φ)", "45.0°"), 
                          ("Pitch (θ)", "-30.0°"), 
                          ("Yaw (ψ)", "90.0°")]:
        lbl = QLabel(f"{angle}\n{value}")
        lbl.setFont(QFont("Roboto", 11))
        lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        lbl.setStyleSheet("""
            QLabel {
                background-color: #f5f5f5;
                padding: 10px;
                border: 2px solid #0078d4;
                border-radius: 5px;
            }
        """)
        ori_layout.addWidget(lbl)
    
    group.layout().addLayout(ori_layout)
    self.output_layout.addWidget(group)

def add_equations_section(self):
    """Display equations using matplotlib's LaTeX renderer"""

    add_section_header(self, "📐 Symbolic Equations", "#673ab7")
    
    group = create_result_group(self, "Forward Kinematics Equations")
    
    # Create figure for equations
    fig = Figure(figsize=(8, 4), facecolor='#fafafa')
    canvas = FigureCanvas(fig)
    ax = fig.add_subplot(111)
    ax.axis('off')
    
    canvas.setMinimumHeight(300)
    canvas.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Minimum)
    # LaTeX equations
    equations = [
        r'$\mathbf{Position\ Vector:}$',
        r'$x = a_2 \cos(\theta_1) \cos(\theta_2) + a_3 \cos(\theta_1) \cos(\theta_2 + \theta_3)$',
        r'$y = a_2 \sin(\theta_1) \cos(\theta_2) + a_3 \sin(\theta_1) \cos(\theta_2 + \theta_3)$',
        r'$z = d_1 + a_2 \sin(\theta_2) + a_3 \sin(\theta_2 + \theta_3)$',
        '',
        r'$\mathbf{Rotation\ Matrix:}$',
        r'$R_0^6 = R_0^1 \cdot R_1^2 \cdot R_2^3 \cdot R_3^4 \cdot R_4^5 \cdot R_5^6$',
    ]
    
    y_pos = 0.9
    for eq in equations:
        if eq:
            fontsize = 14 if 'mathbf' in eq else 12
            ax.text(0.5, y_pos, eq, fontsize=fontsize, verticalalignment='top', horizontalalignment='center')
            y_pos -= 0.15
        else:
            y_pos -= 0.1
    
    canvas.setStyleSheet("""
        QWidget {
            background-color: white;
            border-left: 5px solid #673ab7;
            border-radius: 8px;
            border: 1px solid #e0e0e0;
            padding: 15px;
        }
    """)
    group.layout().addWidget(canvas)
    self.output_layout.addWidget(group)


def add_solutions_section(self):
    """Display multiple IK solutions"""
    add_section_header(self, " Inverse Kinematics Solutions", "#ff9800")
    
    solutions = [
        ("Solution 1", [10, -45, 90, 30, -60, 120], True),
        ("Solution 2", [10, -45, -90, 210, 60, -60], True),
        ("Solution 3", [170, 135, 90, 30, -60, 120], False),
        ("Solution 4", [170, 135, -90, 210, 60, -60], False),
    ]
    
    for sol_name, angles, is_valid in solutions:
        group = create_result_group(self, sol_name)
        
        # Validity indicator
        valid_widget = QWidget()
        valid_layout = QHBoxLayout(valid_widget)
        valid_layout.setContentsMargins(0, 0, 0, 5)
        
        status_icon = QLabel("✓" if is_valid else "✗")
        status_icon.setFont(QFont("Roboto", 14, QFont.Weight.Bold))
        status_text = QLabel("Valid (within joint limits)" if is_valid else "Invalid (exceeds limits)")
        status_text.setFont(self.standard_font)
        
        if is_valid:
            status_icon.setStyleSheet("color: #4caf50;")
            status_text.setStyleSheet("color: #4caf50;")
        else:
            status_icon.setStyleSheet("color: #f44336;")
            status_text.setStyleSheet("color: #f44336;")
        
        valid_layout.addWidget(status_icon)
        valid_layout.addWidget(status_text)
        valid_layout.addStretch()
        group.layout().addWidget(valid_widget)
        
        # Joint angles in compact format
        angles_layout = QHBoxLayout()
        for i, angle in enumerate(angles):
            lbl = QLabel(f"θ{i+1}: {angle}°")
            lbl.setFont(self.standard_font)
            lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
            lbl.setStyleSheet("""
                QLabel {
                    background-color: #fafafa;
                    padding: 5px 10px;
                    border-radius: 3px;
                    border: 1px solid #ddd;
                }
            """)
            angles_layout.addWidget(lbl)
        group.layout().addLayout(angles_layout)
        self.output_layout.addWidget(group)

def add_status_messages(self):
    """Display various status message types"""
    add_section_header(self, "💬 Status Messages", "#607d8b")
    
    messages = [
        ("✓ Success", "Calculation completed successfully!", "#4caf50", "#e8f5e9"),
        ("⚠ Warning", "Solution near singularity point", "#ff9800", "#fff3e0"),
        ("ℹ Info", "Using numeric computation with 1e-6 tolerance", "#2196f3", "#e3f2fd"),
        ("✗ Error", "Joint limits exceeded for θ₃", "#f44336", "#ffebee"),
    ]
    
    for icon, text, color, bg_color in messages:
        msg_widget = QWidget()
        msg_layout = QHBoxLayout(msg_widget)
        msg_layout.setContentsMargins(10, 8, 10, 8)
        
        icon_lbl = QLabel(icon)
        icon_lbl.setFont(QFont("Roboto", 14, QFont.Weight.Bold))
        icon_lbl.setStyleSheet(f"color: {color};")
        
        text_lbl = QLabel(text)
        text_lbl.setFont(QFont("Roboto", 11))
        
        msg_widget.setStyleSheet(f"""
            QWidget {{
                background-color: {bg_color};
                border-left: 4px solid {color};
                border-radius: 3px;
            }}
        """)
        
        msg_layout.addWidget(icon_lbl)
        msg_layout.addWidget(text_lbl)
        msg_layout.addStretch()
        
        self.output_layout.addWidget(msg_widget)

def add_collapsible_section(self):
    """Add expandable section for advanced details"""
    add_section_header(self, "⚙️ Advanced Details", "#795548")
    
    # Create collapsible group box
    group = QGroupBox("Jacobian Matrix and Singularity Analysis")
    group.setFont(QFont("Roboto", 12, QFont.Weight.Bold))
    group.setCheckable(True)
    group.setChecked(False)
    group.setStyleSheet("""
        QGroupBox {
            border: 2px solid #795548;
            border-radius: 5px;
            margin-top: 10px;
            padding-top: 15px;
            background-color: white;
        }
        QGroupBox::title {
            subcontrol-origin: margin;
            left: 10px;
            padding: 0 5px;
            color: #795548;
        }
    """)
    
    content_layout = QVBoxLayout()
    
    # Jacobian
    jac_label = QLabel("Jacobian Matrix J(θ):")
    jac_label.setFont(QFont("Roboto", 11, QFont.Weight.Bold))
    content_layout.addWidget(jac_label)
    
    jac_table = QTableWidget(6, 6)
    jac_table.setFont(QFont("Courier New", 9))
    jac_table.horizontalHeader().setVisible(False)
    jac_table.verticalHeader().setVisible(False)
    jac_table.setMinimumHeight(180)
    jac_table.setMaximumHeight(220)
    jac_table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
    jac_table.verticalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
    
    for i in range(6):
        for j in range(6):
            val = np.random.randn() * 0.5
            item = QTableWidgetItem(f"{val:.3f}")
            item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
            item.setFlags(Qt.ItemFlag.ItemIsEnabled)
            jac_table.setItem(i, j, item)
    
    content_layout.addWidget(jac_table)
    
    # Singularity info
    sing_label = QLabel("Manipulability Index: μ = 0.847")
    sing_label.setFont(QFont("Roboto", 11))
    sing_label.setStyleSheet("padding: 10px; color: #666;")
    content_layout.addWidget(sing_label)
    
    group.setLayout(content_layout)
    self.output_layout.addWidget(group)

def add_export_section(self):
    """Add export options"""
    add_section_header(self, "💾 Export Options", "#00897b")
    
    export_widget = QWidget()
    export_layout = QHBoxLayout(export_widget)
    export_layout.setContentsMargins(10, 10, 10, 10)
    
    buttons_data = [
        ("📄 Copy to Clipboard", "#2196f3"),
        ("📊 Export to CSV", "#4caf50"),
        ("📝 Save as PDF", "#f44336"),
        ("🖼️ Export Plot", "#ff9800"),
    ]
    
    for btn_text, color in buttons_data:
        btn = QPushButton(btn_text)
        btn.setFont(QFont("Roboto", 11))
        btn.setStyleSheet(f"""
            QPushButton {{
                background-color: white;
                color: {color};
                border: 2px solid {color};
                border-radius: 5px;
                padding: 8px 15px;
            }}
            QPushButton:hover {{
                background-color: {color};
                color: white;
            }}
        """)
        btn.setCursor(Qt.CursorShape.PointingHandCursor)
        export_layout.addWidget(btn)
    
    self.output_layout.addWidget(export_widget)

def create_result_group(self, title):
    """Create a styled group box for results"""
    group = QGroupBox(title)
    group.setFont(QFont("Roboto", 13, QFont.Weight.Bold))
    group.setStyleSheet("""
        QGroupBox {
            border: 2px solid #0078d4;
            border-radius: 5px;
            margin-top: 10px;
            padding-top: 15px;
            background-color: white;
        }
        QGroupBox::title {
            subcontrol-origin: margin;
            left: 10px;
            padding: 0 5px;
            color: #0078d4;
        }
    """)
    group.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Minimum)
    layout = QVBoxLayout()
    layout.setSpacing(8)
    layout.setContentsMargins(10, 15, 10, 10) 
    group.setLayout(layout)
    
    return group

def add_spacing(self, layout, height):
    """Add vertical spacing"""
    spacer = QWidget()
    spacer.setFixedHeight(height)
    layout.addWidget(spacer)