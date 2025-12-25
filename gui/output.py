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

    group = QGroupBox()
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
    layout = QVBoxLayout(group)
    layout.setSpacing(8)
    layout.setContentsMargins(10, 15, 10, 10) 
    group.setLayout(layout)
    
    title_label = QLabel(title)
    title_label.setTextFormat(Qt.TextFormat.RichText)
    title_label.setFont(QFont("Roboto", 13, QFont.Weight.Bold))
    title_label.setStyleSheet("color: #0078d4;")
    title_label.setAlignment(Qt.AlignmentFlag.AlignLeft)
    
    layout.addWidget(title_label)
    return group

def add_spacing(self, layout, height):
    """Add vertical spacing"""
    spacer = QWidget()
    spacer.setFixedHeight(height)
    layout.addWidget(spacer)







# Add these functions to gui/output.py
# They match the existing style and reuse your beautiful components!

def display_fk_numeric_results(self, transforms, final_T, joint_values, frame_range):
    """Display numeric FK results using existing style"""
    clear_output(self)
    
    # Title
    add_section_header(self, "✓ Forward Kinematics Results (Numeric)", "#0078d4")
    
    # 1. Input Joint Values (using your existing style)
    add_joint_values_display(self, joint_values)
    add_spacing(self, self.output_layout, 15)
    
    # 2. Transformation Matrices
    if frame_range is None:
        start_frame = 0
        display_transforms = transforms
    else:
        start_frame = frame_range[0]
        display_transforms = transforms
    
    for i, T in enumerate(display_transforms):
        frame_num = start_frame + i + 1
        add_transformation_matrix_section(self, T, f"Transformation Matrix T<sub>{frame_num  -1}</sub><sup>{frame_num}</sup>")
        add_spacing(self, self.output_layout, 15)
    
    # 3. End-Effector Pose (using your existing style)
    add_end_effector_pose_section(self, final_T)
    add_spacing(self, self.output_layout, 15)


def display_fk_symbolic_results(self, transforms, final_T, q_symbols, frame_range):
    """Display symbolic FK results using existing style"""
    clear_output(self)
    
    add_section_header(self, "Forward Kinematics Results (Symbolic)", "#673ab7")
    
    # Show joint variables
    group = create_result_group(self, "Joint Variables")

    var_layout = QHBoxLayout()
    var_layout.setSpacing(10)
    for sym in q_symbols:
        var_box = QLabel(str(sym))
        var_box.setFont(QFont(self.standard_font))
        var_box.setAlignment(Qt.AlignmentFlag.AlignCenter)
        var_box.setStyleSheet("""
            QLabel {
                background-color: #e3f2fd;
                border: 2px solid #0078d4;
                border-radius: 5px;
                padding: 10px 20px;
                color: #0078d4;
            }
        """)
    var_layout.addWidget(var_box)
    var_layout.addStretch()
    group.layout().addLayout(var_layout)
    
    
    self.output_layout.addWidget(group)
    add_spacing(self, self.output_layout, 15)
    # Display each transformation matrix
    if frame_range is None:
        start_frame = 0
        display_transforms = transforms
    else:
        start_frame = frame_range[0]
        display_transforms = transforms
    for i, T in enumerate(display_transforms):
        frame_num = start_frame + i + 1
    add_symbolic_transformation_matrix_simple(self, T, f"Symbolic Matrix T<sub>{frame_num  -1}</sub><sup>{frame_num}</sup>")
    add_spacing(self, self.output_layout, 15)


def display_ik_numeric_results(self, solutions, target_matrix):
    """Display numeric IK results using existing solutions style"""
    clear_output(self)
    
    add_section_header(self, "✓ Inverse Kinematics Results", "#ff9800")
    
    # Target Matrix Display
    add_transformation_matrix_section(self, target_matrix, "🎯 Target Transformation Matrix")
    add_spacing(self, self.output_layout, 15)
    
    # Solutions Header
    add_section_header(self, f"Found {len(solutions)} Solution(s)", "#4caf50")
    
    angle_unit = "deg" if self.theta_system_widget.currentRow() == 1 else "rad"
    
    # Display each solution (matching your existing solutions style)
    for sol_idx, solution in enumerate(solutions, 1):
        add_ik_solution_display(self, solution, sol_idx, target_matrix, angle_unit)
        add_spacing(self, self.output_layout, 15)


def display_ik_no_solution(self, target_matrix):
    """Display message when no IK solution found"""
    clear_output(self)
    
    add_section_header(self, "❌ Inverse Kinematics - No Solution", "#f44336")
    
    # Error message (using your existing status message style)
    msg_widget = QWidget()
    msg_layout = QHBoxLayout(msg_widget)
    msg_layout.setContentsMargins(10, 8, 10, 8)
    
    icon_lbl = QLabel("✗")
    icon_lbl.setFont(QFont("Roboto", 14, QFont.Weight.Bold))
    icon_lbl.setStyleSheet("color: #f44336;")
    
    text_lbl = QLabel(
        "No valid inverse kinematics solution found.\n"
        "The target pose may be outside the robot's workspace or unreachable."
    )
    text_lbl.setFont(QFont("Roboto", 11))
    text_lbl.setWordWrap(True)
    
    msg_widget.setStyleSheet("""
        QWidget {
            background-color: #ffebee;
            border-left: 4px solid #f44336;
            border-radius: 3px;
        }
    """)
    
    msg_layout.addWidget(icon_lbl)
    msg_layout.addWidget(text_lbl)
    msg_layout.addStretch()
    
    self.output_layout.addWidget(msg_widget)
    add_spacing(self, self.output_layout, 15)
    
    # Show target matrix
    add_transformation_matrix_section(self, target_matrix, "Target Matrix (Unreachable)")


# ==================== HELPER FUNCTIONS (Matching Your Style) ====================

def add_joint_values_display(self, joint_values):
    """Display joint values using your existing grid style"""
    group = create_result_group(self, "📊 Input Joint Values")
    
    grid = QGridLayout()
    grid.setSpacing(10)
    
    angle_unit = "deg" if self.theta_system_widget.currentRow() == 1 else "rad"
    dh_params = self.current_manipulator.get_dh_parameters()
    
    colors = ["#e3f2fd", "#e8f5e9", "#fff3e0", "#fce4ec", "#f3e5f5", "#e0f2f1"]
    
    for i, (params, val) in enumerate(zip(dh_params, joint_values)):
        row = i // 3
        col = (i % 3) * 2
        
        # Joint label
        if params['variable'] == 'theta':
            label_text = f"θ{i+1}"
            if angle_unit == "deg":
                value_text = f"{np.rad2deg(val):.4f}°"
            else:
                value_text = f"{val:.6f} rad"
        else:
            label_text = f"d{i+1}"
            value_text = f"{val:.6f} m"
        
        # Label with color background
        lbl = QLabel(label_text)
        lbl.setFont(QFont("Roboto", 13, QFont.Weight.Bold))
        lbl.setStyleSheet(f"""
            QLabel {{
                background-color: {colors[i % len(colors)]};
                padding: 8px;
                border-radius: 3px;
                color: #333;
            }}
        """)
        lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        grid.addWidget(lbl, row, col)
        
        # Value with border
        val_lbl = QLabel(value_text)
        val_lbl.setFont(QFont("Roboto", 13))
        val_lbl.setStyleSheet(f"""
            QLabel {{
                background-color: white;
                padding: 8px;
                border: 1px solid {colors[i % len(colors)]};
                border-radius: 3px;
            }}
        """)
        val_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        grid.addWidget(val_lbl, row, col + 1)
    
    group.layout().addLayout(grid)
    self.output_layout.addWidget(group)


def add_transformation_matrix_section(self, T, title):
    """Display transformation matrix using your existing matrix style"""
    group = create_result_group(self, title)
    
    # Create matrix table (matching your existing style)
    matrix_table = QTableWidget(4, 4)
    matrix_table.setFont(QFont("Courier New", 11))
    matrix_table.horizontalHeader().setVisible(False)
    matrix_table.verticalHeader().setVisible(False)
    matrix_table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
    matrix_table.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Minimum)
    
    for i in range(4):
        for j in range(4):
            item = QTableWidgetItem(f"{T[i, j]:.6f}")
            item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
            
            # Color code like your existing matrix
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
    
    matrix_table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
    matrix_table.verticalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
    matrix_table.setMinimumHeight(180)
    matrix_table.setMaximumHeight(220)
    
    group.layout().addWidget(matrix_table)
    
    # Add legend (matching your existing style)
    legend = QLabel("🟪 Rotation | 🟧 Translation | ⬜ Homogeneous")
    legend.setFont(QFont("Roboto", 10))
    legend.setStyleSheet("color: #666; margin-top: 5px;")
    legend.setAlignment(Qt.AlignmentFlag.AlignCenter)
    group.layout().addWidget(legend)
    
    self.output_layout.addWidget(group)


def add_symbolic_transformation_matrix(self, T_sym, title):
    """
    Display symbolic matrix beautifully using matplotlib LaTeX rendering
    """
    group = create_result_group(self, title)
    
    # Create matplotlib figure for LaTeX rendering
    fig = Figure(figsize=(10, 5), facecolor='white')
    canvas = FigureCanvas(fig)
    ax = fig.add_subplot(111)
    ax.axis('off')
    
    canvas.setMinimumHeight(400)
    canvas.setMaximumHeight(500)
    canvas.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Minimum)
    
    # Convert SymPy matrix to LaTeX
    latex_str = sp.latex(T_sym, mat_delim='[', mat_str='matrix')
    
    # Create beautiful LaTeX display with color coding
    full_latex = r'$\mathbf{T} = ' + latex_str + r'$'
    
    # Display the matrix
    ax.text(0.5, 0.5, full_latex, 
            fontsize=16, 
            verticalalignment='center',
            horizontalalignment='center',
            bbox=dict(boxstyle='round,pad=1', facecolor='#f6f8fa', edgecolor='#0078d4', linewidth=2))
    
    canvas.setStyleSheet("""
        QWidget {
            background-color: white;
            border: 1px solid #e0e0e0;
            border-radius: 5px;
            padding: 15px;
        }
    """)
    
    group.layout().addWidget(canvas)
    
    # Add expandable detailed view
    add_symbolic_matrix_details(self, group, T_sym)
    
    self.output_layout.addWidget(group)

def add_symbolic_matrix_details(self, group, T_sym):
    """
    Add collapsible detailed view of symbolic matrix elements
    """
    # Create collapsible section
    details_group = QGroupBox("📋 Detailed Matrix Elements (Click to expand)")
    details_group.setFont(QFont("Roboto", 11, QFont.Weight.Bold))
    details_group.setCheckable(True)
    details_group.setChecked(False)  # Collapsed by default
    details_group.setStyleSheet("""
        QGroupBox {
            border: 2px solid #9c27b0;
            border-radius: 5px;
            margin-top: 10px;
            padding-top: 15px;
            background-color: #fafafa;
        }
        QGroupBox::title {
            subcontrol-origin: margin;
            left: 10px;
            padding: 0 5px;
            color: #9c27b0;
        }
        QGroupBox::indicator {
            width: 15px;
            height: 15px;
        }
        QGroupBox::indicator:unchecked {
            image: none;
            border: 2px solid #9c27b0;
            background-color: white;
            border-radius: 3px;
        }
        QGroupBox::indicator:checked {
            image: none;
            border: 2px solid #9c27b0;
            background-color: #9c27b0;
            border-radius: 3px;
        }
    """)
    
    content_layout = QVBoxLayout()
    
    # Create a grid for matrix elements
    grid_widget = QWidget()
    grid_layout = QGridLayout(grid_widget)
    grid_layout.setSpacing(8)
    grid_layout.setContentsMargins(10, 10, 10, 10)
    
    # Labels for sections
    rotation_label = QLabel("🟪 Rotation Matrix (3×3)")
    rotation_label.setFont(QFont("Roboto", 11, QFont.Weight.Bold))
    rotation_label.setStyleSheet("color: #9c27b0; padding: 5px;")
    
    translation_label = QLabel("🟧 Translation Vector (3×1)")
    translation_label.setFont(QFont("Roboto", 11, QFont.Weight.Bold))
    translation_label.setStyleSheet("color: #ff9800; padding: 5px; margin-top: 10px;")
    
    content_layout.addWidget(rotation_label)
    
    # Display rotation part (3x3) in colored boxes
    rotation_grid = QGridLayout()
    rotation_grid.setSpacing(5)
    
    colors_rotation = [
        ["#f3e5f5", "#e1bee7", "#ce93d8"],
        ["#e1bee7", "#ce93d8", "#ba68c8"],
        ["#ce93d8", "#ba68c8", "#ab47bc"]
    ]
    
    for i in range(3):
        for j in range(3):
            element = T_sym[i, j]
            element_str = sp.pretty(element, use_unicode=True)
            
            # Simplify display for common cases
            if element == 0:
                element_str = "0"
            elif element == 1:
                element_str = "1"
            elif element == -1:
                element_str = "-1"
            
            label = QLabel(element_str)
            label.setFont(QFont("Courier New", 10))
            label.setAlignment(Qt.AlignmentFlag.AlignCenter)
            label.setStyleSheet(f"""
                QLabel {{
                    background-color: {colors_rotation[i][j]};
                    border: 1px solid #9c27b0;
                    border-radius: 3px;
                    padding: 8px;
                    min-width: 120px;
                    min-height: 40px;
                }}
            """)
            label.setWordWrap(True)
            rotation_grid.addWidget(label, i, j)
    
    content_layout.addLayout(rotation_grid)
    content_layout.addWidget(translation_label)
    
    # Display translation part (3x1) in colored boxes
    translation_layout = QHBoxLayout()
    translation_layout.setSpacing(5)
    
    colors_translation = ["#fff3e0", "#ffe0b2", "#ffcc80"]
    axes = ["X", "Y", "Z"]
    
    for i in range(3):
        element = T_sym[i, 3]
        element_str = sp.pretty(element, use_unicode=True)
        
        if element == 0:
            element_str = "0"
        
        axis_widget = QWidget()
        axis_layout = QVBoxLayout(axis_widget)
        axis_layout.setContentsMargins(5, 5, 5, 5)
        axis_layout.setSpacing(5)
        
        axis_label = QLabel(axes[i])
        axis_label.setFont(QFont("Roboto", 11, QFont.Weight.Bold))
        axis_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        axis_label.setStyleSheet("color: #ff9800;")
        
        value_label = QLabel(element_str)
        value_label.setFont(QFont("Courier New", 10))
        value_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        value_label.setStyleSheet(f"""
            QLabel {{
                background-color: {colors_translation[i]};
                border: 2px solid #ff9800;
                border-radius: 3px;
                padding: 8px;
                min-width: 100px;
                min-height: 40px;
            }}
        """)
        value_label.setWordWrap(True)
        
        axis_layout.addWidget(axis_label)
        axis_layout.addWidget(value_label)
        translation_layout.addWidget(axis_widget)
    
    content_layout.addLayout(translation_layout)
    
    # Add note about homogeneous row
    note = QLabel("ℹ️ Bottom row: [0  0  0  1] (Homogeneous coordinates)")
    note.setFont(QFont("Roboto", 10))
    note.setStyleSheet("color: #666; padding: 10px; margin-top: 5px;")
    content_layout.addWidget(note)
    
    details_group.setLayout(content_layout)
    group.layout().addWidget(details_group)


# ALTERNATIVE: Simpler version using styled text (if matplotlib has issues)
def add_symbolic_transformation_matrix_simple(self, T_sym, title):
    """
    Display symbolic matrix with beautiful styled text (no matplotlib)
    """
    group = create_result_group(self, title)
    
    # Main container with styled background
    container = QWidget()
    container_layout = QVBoxLayout(container)
    container_layout.setContentsMargins(15, 15, 15, 15)
    container.setStyleSheet("""
        QWidget {
            background: qlineargradient(x1:0, y1:0, x2:1, y2:1,
                                       stop:0 #f6f8fa, stop:1 #e9ecef);
            border: 2px solid #0078d4;
            border-radius: 8px;
        }
    """)
    
    # Title for matrix
    matrix_title = QLabel("⎡  Transformation Matrix  ⎤")
    matrix_title.setFont(QFont("Roboto", 14, QFont.Weight.Bold))
    matrix_title.setAlignment(Qt.AlignmentFlag.AlignCenter)
    matrix_title.setStyleSheet("color: #0078d4; background: transparent; border: none;")
    container_layout.addWidget(matrix_title)
    
    # Create grid for matrix display
    matrix_widget = QWidget()
    matrix_layout = QGridLayout(matrix_widget)
    matrix_layout.setSpacing(8)
    matrix_widget.setStyleSheet("background: transparent; border: none;")
    
    # Display 4x4 matrix with color coding
    for i in range(4):
        for j in range(4):
            element = T_sym[i, j]
            element_str = sp.pretty(element, use_unicode=True)
            
            # Simplify display
            if element == 0:
                element_str = "0"
            elif element == 1:
                element_str = "1"
            elif element == -1:
                element_str = "-1"
            
            label = QLabel(element_str)
            label.setFont(QFont("Courier New", 11))
            label.setAlignment(Qt.AlignmentFlag.AlignCenter)
            
            # Color coding
            if i < 3 and j < 3:  # Rotation
                bg_color = "#f6e3fd"
                border_color = "#9c27b0"
            elif i < 3 and j == 3:  # Translation
                bg_color = "#fff3e0"
                border_color = "#ff9800"
            else:  # Homogeneous row
                bg_color = "#f5f5f5"
                border_color = "#cccccc"
            
            label.setStyleSheet(f"""
                QLabel {{
                    background-color: {bg_color};
                    border: 2px solid {border_color};
                    border-radius: 4px;
                    padding: 12px;
                    min-width: 100px;
                    min-height: 50px;
                }}
            """)
            label.setWordWrap(True)
            
            matrix_layout.addWidget(label, i, j)
    
    container_layout.addWidget(matrix_widget)
    
    # Add legend
    legend_layout = QHBoxLayout()
    legend_layout.setSpacing(15)
    
    legend_items = [
        ("🟪 Rotation (3×3)", "#f6e3fd"),
        ("🟧 Translation (3×1)", "#fff3e0"),
        ("⬜ Homogeneous (1×4)", "#f5f5f5")
    ]
    
    for text, color in legend_items:
        legend_label = QLabel(text)
        legend_label.setFont(QFont("Roboto", 10))
        legend_label.setStyleSheet(f"""
            QLabel {{
                background-color: {color};
                padding: 5px 10px;
                border-radius: 3px;
                border: 1px solid #ddd;
            }}
        """)
        legend_layout.addWidget(legend_label)
    
    legend_layout.addStretch()
    container_layout.addLayout(legend_layout)
    
    group.layout().addWidget(container)
    
    # Add expandable detailed view
    add_symbolic_matrix_details(self, group, T_sym)
    
    self.output_layout.addWidget(group)



def add_end_effector_pose_section(self, T):
    """Display end-effector pose using your existing style"""
    group = create_result_group(self, "🎯 End-Effector Pose")
    
    # Position section (matching your existing style)
    position = self.current_manipulator.extract_position(T)
    
    pos_label = QLabel("Position (meters):")
    pos_label.setFont(QFont("Roboto", 12, QFont.Weight.Bold))
    group.layout().addWidget(pos_label)
    
    pos_layout = QHBoxLayout()
    for axis, value, color in [("X", position[0], "#ef5350"),
                                 ("Y", position[1], "#66bb6a"),
                                 ("Z", position[2], "#42a5f5")]:
        axis_widget = QWidget()
        axis_layout = QVBoxLayout(axis_widget)
        axis_layout.setContentsMargins(5, 5, 5, 5)
        
        axis_lbl = QLabel(axis)
        axis_lbl.setFont(QFont("Roboto", 11, QFont.Weight.Bold))
        axis_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        axis_lbl.setStyleSheet(f"color: {color};")
        
        val_lbl = QLabel(f"{value:.6f}")
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
    
    # Orientation section (matching your existing style)
    add_spacing(self, group.layout(), 10)
    
    R = self.current_manipulator.extract_rotation(T)
    roll, pitch, yaw = self.current_manipulator.rotation_to_euler(R, 'xyz')
    
    angle_unit = "deg" if self.theta_system_widget.currentRow() == 1 else "rad"
    if angle_unit == "deg":
        roll, pitch, yaw = np.rad2deg([roll, pitch, yaw])
        unit_str = "°"
    else:
        unit_str = " rad"
    
    ori_label = QLabel("Orientation (Roll-Pitch-Yaw):")
    ori_label.setFont(QFont("Roboto", 12, QFont.Weight.Bold))
    group.layout().addWidget(ori_label)
    
    ori_layout = QHBoxLayout()
    for angle_name, value in [("Roll (φ)", roll),
                               ("Pitch (θ)", pitch),
                               ("Yaw (ψ)", yaw)]:
        lbl = QLabel(f"{angle_name}\n{value:.4f}{unit_str}")
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


def add_ik_solution_display(self, solution, sol_idx, target_matrix, angle_unit):
    """Display IK solution using your existing solution style"""
    group = create_result_group(self, f"Solution {sol_idx}")
    
    # Validity check using FK
    _, T_verify = self.current_manipulator.compute_fk_numeric(solution)
    error = np.linalg.norm(T_verify - target_matrix)
    is_valid = error < 1e-5
    
    # Validity indicator (matching your existing style)
    valid_widget = QWidget()
    valid_layout = QHBoxLayout(valid_widget)
    valid_layout.setContentsMargins(0, 0, 0, 5)
    
    status_icon = QLabel("✓" if is_valid else "⚠")
    status_icon.setFont(QFont("Roboto", 14, QFont.Weight.Bold))
    
    if is_valid:
        status_text = QLabel(f"Valid Solution (error: {error:.2e})")
        status_icon.setStyleSheet("color: #4caf50;")
        status_text.setStyleSheet("color: #4caf50;")
    else:
        status_text = QLabel(f"Solution found (verification error: {error:.2e})")
        status_icon.setStyleSheet("color: #ff9800;")
        status_text.setStyleSheet("color: #ff9800;")
    
    status_text.setFont(self.standard_font)
    
    valid_layout.addWidget(status_icon)
    valid_layout.addWidget(status_text)
    valid_layout.addStretch()
    group.layout().addWidget(valid_widget)
    
    # Joint angles in grid (matching your existing compact format)
    angles_layout = QHBoxLayout()
    colors = ["#e3f2fd", "#e8f5e9", "#fff3e0", "#fce4ec", "#f3e5f5", "#e0f2f1"]
    
    for i, angle in enumerate(solution):
        if angle_unit == "deg":
            value_text = f"{np.rad2deg(angle):+.4f}°"
        else:
            value_text = f"{angle:+.6f} rad"
        
        lbl = QLabel(f"θ{i+1}:\n{value_text}")
        lbl.setFont(self.standard_font)
        lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        lbl.setStyleSheet(f"""
            QLabel {{
                background-color: {colors[i % len(colors)]};
                padding: 8px 10px;
                border-radius: 3px;
                border: 1px solid #ddd;
            }}
        """)
        angles_layout.addWidget(lbl)
    
    group.layout().addLayout(angles_layout)
    self.output_layout.addWidget(group)