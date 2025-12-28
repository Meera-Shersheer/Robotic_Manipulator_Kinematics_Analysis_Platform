from imports import *

# def run_output_test(self):
#     """Display comprehensive test of all output capabilities"""
#     # Clear existing content
#     clear_output(self)
    
#     # 1. Section Header with styled background
#     add_section_header(self,"Forward Kinematics Results", "#0078d4")
    
#     # 2. Joint Values Display (Grid Layout)
#     add_joint_values_section(self)
#     # add_spacing(self, self.output_layout, 15) 
#     # 3. Transformation Matrix Display
#     add_matrix_section(self)
#     # add_spacing(self, self.output_layout, 15) 
#     # 4. Position & Orientation Results
#     add_pose_section(self)
#     # add_spacing(self, self.output_layout, 15)  
#     # 5. Symbolic Equations Display
#     add_equations_section(self)
#     # add_spacing(self, self.output_layout, 15) 
#     # 6. Multiple Solutions (for IK)
#     add_solutions_section(self)
#     # add_spacing(self, self.output_layout, 15) 
#     # 7. Error/Warning Messages
#     add_status_messages(self)
#     # add_spacing(self, self.output_layout, 15) 
#     # 8. Collapsible Advanced Section
#     add_collapsible_section(self)
#     # add_spacing(self, self.output_layout, 15) 
#     # 9. Export Options
#     add_export_section(self)
    
#     # spacer = QWidget()
#     # spacer.setFixedHeight(20)  # Fixed height instead of stretch
#     # self.output_layout.addWidget(spacer)

def clear_output(self):
    """Remove all widgets from output layout"""
    while self.output_layout.count():
        child = self.output_layout.takeAt(0)
        if child.widget():
            child.widget().deleteLater()

def add_section_header(self, title, color="#d2598e"):
    """Add a prominent section header"""
    header = QLabel(title)
    header.setFont(QFont("Roboto", 16, QFont.Weight.Bold))
    header.setStyleSheet(f"""
        QLabel {{
            background-color: {color};
            color: white;
            padding: 6px;
            border-radius: 5px;
        }}
    """)
    header.setAlignment(Qt.AlignmentFlag.AlignCenter)
    header.setFixedHeight(60)
    self.output_layout.addWidget(header)




# def add_equations_section(self, eq_list):
#     """Display equations using native Qt with beautiful styling"""
    
#     add_section_header(self, "Symbolic Equations", "#673ab7")
    
#     group = create_result_group(self, "Inverse Kinematics Equations")
    
#     # Create scrollable area
#     scroll_area = QScrollArea()
#     scroll_area.setWidgetResizable(True)
#     scroll_area.setMinimumHeight(450)
#     scroll_area.setMaximumHeight(650)
    
#     scroll_area.setStyleSheet("""
#         QScrollArea {
#             border: 2px solid #673ab7;
#             border-radius: 8px;
#             background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
#                 stop:0 #fafafa, stop:1 #f5f5f5);
#         }
#         QScrollBar:vertical {
#             border: none;
#             background-color: #e8e8e8;
#             width: 10px;
#             border-radius: 5px;
#             margin: 2px;
#         }
#         QScrollBar::handle:vertical {
#             background: #673ab7;
#             border-radius: 5px;
#             min-height: 20px;
#         }
#         QScrollBar::handle:vertical:hover {
#             background: #7b1fa2;
#         }
#         QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {
#             height: 0px;
#         }
#     """)
    
#     # Container for equations
#     equations_container = QWidget()
#     equations_layout = QVBoxLayout(equations_container)
#     equations_layout.setSpacing(10)
#     equations_layout.setContentsMargins(20, 20, 20, 20)
    
#     # Track sections for card-based layout
#     current_section_header = None
#     current_section_eqs = []
    
#     for eq in eq_list:
#         if not eq:
#             # Empty line - create card if we have equations
#             if current_section_eqs:
#                 card = create_equation_card(self, current_section_header, current_section_eqs)
#                 equations_layout.addWidget(card)
#                 current_section_eqs = []
#                 current_section_header = None
#             continue
        
#         # Check if it's a section header
#         is_header = 'mathbf' in eq and any(f'{i})' in eq for i in range(1, 10))
        
#         if is_header:
#             # Save previous section
#             if current_section_eqs:
#                 card = create_equation_card(self, current_section_header, current_section_eqs)
#                 equations_layout.addWidget(card)
#                 current_section_eqs = []
#             current_section_header = eq
#         else:
#             current_section_eqs.append(eq)
    
#     # Add last section
#     if current_section_eqs:
#         card = create_equation_card(self, current_section_header, current_section_eqs)
#         equations_layout.addWidget(card)
    
#     equations_layout.addStretch()
#     scroll_area.setWidget(equations_container)
    
#     group.layout().addWidget(scroll_area)
#     self.output_layout.addWidget(group)


# def create_equation_card(self, header, equations):
#     """Create a beautiful card for a section of equations"""
    
#     card = QFrame()
#     card.setStyleSheet("""
#         QFrame {
#             background-color: white;
#             border: 2px solid #e0e0e0;
#             border-radius: 10px;
#         }
#     """)
    
#     card_layout = QVBoxLayout(card)
#     card_layout.setSpacing(0)
#     card_layout.setContentsMargins(0, 0, 0, 0)
    
#     # Header
#     if header:
#         header_widget = QWidget()
#         header_widget.setStyleSheet("""
#             QWidget {
#                 background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
#                     stop:0 #673ab7, stop:1 #8e24aa);
#                 border-radius: 8px 8px 0px 0px;
#             }
#         """)
#         header_layout = QHBoxLayout(header_widget)
#         header_layout.setContentsMargins(15, 12, 15, 12)
        
#         # Add icon
#         icon_label = QLabel("📐")
#         icon_label.setFont(QFont("Segoe UI Emoji", 14))
#         header_layout.addWidget(icon_label)
        
#         header_text = clean_latex_to_unicode(header)
#         header_label = QLabel(header_text)
#         header_label.setFont(QFont("Roboto", 13, QFont.Weight.Bold))
#         header_label.setStyleSheet("color: white; margin-left: 5px;")
#         header_label.setTextFormat(Qt.TextFormat.RichText)
#         header_layout.addWidget(header_label)
#         header_layout.addStretch()
        
#         card_layout.addWidget(header_widget)
    
#     # Equations
#     eq_container = QWidget()
#     eq_layout = QVBoxLayout(eq_container)
#     eq_layout.setSpacing(6)
#     eq_layout.setContentsMargins(15, 12, 15, 12)
    
#     for eq in equations:
#         eq_text = clean_latex_to_unicode(eq)
#         eq_label = QLabel(eq_text)
#         eq_label.setFont(QFont("Roboto", 12))
#         eq_label.setStyleSheet("""
#             QLabel {
#                 color: #2c3e50;
#                 padding: 10px 12px;
#                 background-color: #f8f9fa;
#                 border-left: 4px solid #9c27b0;
#                 border-radius: 5px;
#             }
#         """)
#         eq_label.setTextFormat(Qt.TextFormat.RichText)
#         eq_label.setWordWrap(True)
#         eq_layout.addWidget(eq_label)
    
#     card_layout.addWidget(eq_container)
    
#     return card
    
def create_result_group(self, title):
    """Create a styled group box for results"""

    group = QGroupBox()
    group.setStyleSheet("""
        QGroupBox {
            border: 2px solid #00897b;
            border-radius: 5px;
            margin-top: 5px;
            padding-top: 10px;
            background-color: white;
        }
        QGroupBox::title {
            subcontrol-origin: margin;
            left: 10px;
            padding: 0 3px;
            color: #6a1b9a;
        }
    """)
    layout = QVBoxLayout(group)
    layout.setSpacing(3)
    layout.setContentsMargins(8, 8, 8, 8) 
    group.setLayout(layout)
    
    title_label = QLabel(title)
    title_label.setTextFormat(Qt.TextFormat.RichText)
    title_label.setFont(QFont("Roboto", 13, QFont.Weight.Bold))
    title_label.setStyleSheet("color: #6a1b9a;")
    title_label.setAlignment(Qt.AlignmentFlag.AlignLeft)
    title_label.setFixedHeight(25) 
    layout.addWidget(title_label)
    return group

def add_spacing(self, layout, height):
    """Add vertical spacing"""
    spacer = QWidget()
    spacer.setFixedHeight(height)
    layout.addWidget(spacer)


def display_fk_numeric_results(self, individual_Ts, cumulative_Ts, final_T, joint_values, frame_range):
    """Display numeric FK results using existing style"""
    clear_output(self)
    
    # Header
    add_section_header(self, "Forward Kinematics Results (Numeric)", "#8e24aa")
    
    # Joints Variables
    add_joint_values_display(self, joint_values)
    add_spacing(self, self.output_layout, 15)
    add_section_header(self, "Individual Link Transformations", "#00897b")
    
    # Transformation Matrices
    if frame_range is None:
        start_frame = 0
        display_individual = individual_Ts
    else:
        start_frame = frame_range[0]
        display_individual = individual_Ts[frame_range[0]:frame_range[1]]
    
    for i, T in enumerate(display_individual):
        frame_from = start_frame + i
        frame_to = frame_from + 1
        add_transformation_matrix_section(self, T, f"Transformation Matrix T<sub>{frame_from}</sub><sup>{frame_to}</sup>")
        add_spacing(self, self.output_layout, 10)
    
    add_section_header(self, "Cumulative Transformations (from Base)", "#673ab7")
    if frame_range is None:
        display_cumulative = cumulative_Ts
    else:
        display_cumulative = cumulative_Ts[frame_range[0]:frame_range[1]]
    
    for i, T in enumerate(display_cumulative):
        frame_num = start_frame + i + 1
        add_transformation_matrix_section(self, T, f"Transformation Matrix T<sub>0</sub><sup>{frame_num}</sup>")
        add_spacing(self, self.output_layout, 10)
        
    add_section_header(self, "Overall Forward Kinematics Transformation", "#ff9800")
    add_transformation_matrix_section(self, final_T, f"Transformation Matrix T<sub>0</sub><sup>6</sup>")
    add_spacing(self, self.output_layout, 10)    
    # 3. End-Effector Pose 
    add_end_effector_pose_section(self, final_T)
    add_spacing(self, self.output_layout, 15)


def display_fk_symbolic_results(self, individual_Ts, cumulative_Ts, final_T, q_symbols, frame_range):
    """Display symbolic FK results using existing style"""
    clear_output(self)
    
    add_section_header(self, "Forward Kinematics Results (Symbolic)", "#00897b")
    
    # joint variables
    group = create_result_group(self, "Joint Variables")

    var_layout = QHBoxLayout()
    var_layout.setSpacing(8)  
    var_layout.setContentsMargins(0, 0, 0, 0) 

    for sym in q_symbols:
        var_box = QLabel(str(sym))
        var_box.setFont(QFont(self.standard_font))
        var_box.setAlignment(Qt.AlignmentFlag.AlignVCenter | Qt.AlignmentFlag.AlignHCenter )
        var_box.setStyleSheet("""
            QLabel {
                background-color: #e0f7fa;
                border: 2px solid #00897b;
                border-radius: 5px;
                padding: 2px 6px;
                color: #024f48;
            }
        """)
        var_box.setFixedHeight(35)
        var_layout.addWidget(var_box)
    
    group.layout().addLayout(var_layout)
    group.layout().setSpacing(5)  
    group.layout().setContentsMargins(10, 10, 10, 10) 
    
    self.output_layout.addWidget(group)
    
    add_section_header(self, "Individual Link Transformations", "#00897b")
    
    if frame_range is None:
        start_frame = 0
        display_individual = individual_Ts
    else:
        start_frame = frame_range[0]
        display_individual = individual_Ts[frame_range[0]:frame_range[1]]
    
    for i, T in enumerate(display_individual):
        frame_from = start_frame + i
        frame_to = frame_from + 1
        add_symbolic_transformation_matrix_simple(self, T, f"Symbolic Matrix T<sub>{frame_from}</sub><sup>{frame_to}</sup>")
        add_spacing(self, self.output_layout, 10)
    
    # Cumulative Transformations
    add_section_header(self, "Cumulative Transformations (from Base)", "#673ab7")
    
    if frame_range is None:
        display_cumulative = cumulative_Ts
    else:
        display_cumulative = cumulative_Ts[frame_range[0]:frame_range[1]]
    
    for i, T in enumerate(display_cumulative):
        frame_num = start_frame + i + 1
        add_symbolic_transformation_matrix_simple(self, T, f"Symbolic Matrix T<sub>0</sub><sup>{frame_num}</sup>")
        add_spacing(self, self.output_layout, 10)
    add_section_header(self, "Overall Forward Kinematics Transformation", "#ff9800")
    add_symbolic_transformation_matrix_simple(self, final_T, f"Transformation Matrix T<sub>0</sub><sup>6</sup>")
    add_spacing(self, self.output_layout, 10)    




def display_ik_numeric_results(self, solutions, target_matrix):
    """Display numeric IK results using existing solutions style"""
    clear_output(self)
    
    add_section_header(self, "Inverse Kinematics Results", "#8e24aa")
    
    # Target Matrix Display
    add_section_header(self, "Target Transformation Matrix and End-Effector Pose", "#00897b")
    add_transformation_matrix_section(self, target_matrix, "Target Transformation Matrix")
    add_spacing(self, self.output_layout, 10)
    
    add_end_effector_pose_section(self, target_matrix)
    add_spacing(self, self.output_layout, 10)
    
    # Solutions Header
    add_section_header(self, f"Found {len(solutions)} Solution(s)", "#673ab7")
    
    angle_unit = "deg" if self.theta_system_widget.currentRow() == 1 else "rad"
    
    # Display each solution
    for sol_idx, solution in enumerate(solutions, 1):
        add_ik_solution_display(self, solution, sol_idx, target_matrix, angle_unit)
        add_spacing(self, self.output_layout, 10)


# Add this to your gui/output.py file

def display_ik_symbolic_detailed(self, ik_result, T_symbolic):
    """
    Display detailed symbolic IK derivation with step-by-step visualization
    """
    clear_output(self)
    
    # Main header
    add_section_header(self, "Inverse Kinematics - Symbolic Derivation", "#8e24aa")
    
    # ========== Target Pose Section ==========
    add_section_header(self, "Target End-Effector Pose (Symbolic)", "#00897b")
    
    group = create_result_group(self, "Symbolic Variables")
    
    # Display symbolic variables in a nice grid
    vars_container = QWidget()
    vars_layout = QGridLayout(vars_container)
    vars_layout.setSpacing(10)
    vars_layout.setContentsMargins(10, 10, 10, 10)
    
    var_info = [
        ("x", "X Position", "#ef5350"),
        ("y", "Y Position", "#66bb6a"),
        ("z", "Z Position", "#42a5f5"),
        ("α", "Roll Angle", "#ff9800"),
        ("β", "Pitch Angle", "#ab47bc"),
        ("γ", "Yaw Angle", "#26c6da")
    ]
    
    for idx, (symbol, description, color) in enumerate(var_info):
        row = idx // 3
        col = (idx % 3) * 2
        
        # Symbol label
        sym_lbl = QLabel(symbol)
        sym_lbl.setFont(QFont("Roboto", 16, QFont.Weight.Bold))
        sym_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        sym_lbl.setStyleSheet(f"""
            QLabel {{
                background-color: {color};
                color: white;
                padding: 12px;
                border-radius: 5px;
                min-width: 50px;
            }}
        """)
        vars_layout.addWidget(sym_lbl, row, col)
        
        # Description label
        desc_lbl = QLabel(description)
        desc_lbl.setFont(self.standard_font)
        desc_lbl.setAlignment(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter)
        desc_lbl.setStyleSheet(f"""
            QLabel {{
                background-color: white;
                padding: 12px;
                border: 2px solid {color};
                border-radius: 5px;
            }}
        """)
        vars_layout.addWidget(desc_lbl, row, col + 1)
    
    group.layout().addWidget(vars_container)
    self.output_layout.addWidget(group)
    add_spacing(self, self.output_layout, 15)
    
    # ========== Transformation Matrix ==========
    add_symbolic_transformation_matrix_simple(self, T_symbolic, "Target Transformation Matrix T₀⁶")
    add_spacing(self, self.output_layout, 15)
    
    # ========== Derivation Steps ==========
    add_section_header(self, "Solution Derivation Steps", "#673ab7")
    
    # Step 1: Wrist Center
    create_derivation_step(self, 
        step_num=1,
        title="Computing Wrist Center (p₀₅)",
        description="The wrist center is computed by offsetting from the target position along the approach vector:",
        equations=[
            ("p₀₅ₓ", f"{sp.pretty(ik_result['wrist_center'][0], use_unicode=True)}"),
            ("p₀₅ᵧ", f"{sp.pretty(ik_result['wrist_center'][1], use_unicode=True)}"),
            ("p₀₅ᵤ", f"{sp.pretty(ik_result['wrist_center'][2], use_unicode=True)}")
        ],
        color="#00897b"
    )
    add_spacing(self, self.output_layout, 10)
    
    # Step 2: θ₁ Solutions
    create_derivation_step(self,
        step_num=2,
        title="Solving for θ₁ (Base Rotation)",
        description="Two solutions exist due to geometric ambiguity:",
        equations=[
            ("θ₁⁽¹⁾", f"{sp.pretty(ik_result['theta1'][0], use_unicode=True)}"),
            ("θ₁⁽²⁾", f"{sp.pretty(ik_result['theta1'][1], use_unicode=True)}")
        ],
        color="#1976d2"
    )
    add_spacing(self, self.output_layout, 10)
    
    # Step 3: θ₅ Solutions
    create_derivation_step(self,
        step_num=3,
        title="Solving for θ₅ (Wrist Rotation)",
        description="Computed from the geometric constraint:",
        equations=[
            ("θ₅⁽¹⁾", f"{sp.pretty(ik_result['theta5'][0], use_unicode=True)}"),
            ("θ₅⁽²⁾", f"{sp.pretty(ik_result['theta5'][1], use_unicode=True)}")
        ],
        color="#d32f2f"
    )
    add_spacing(self, self.output_layout, 10)
    
    # Step 4: θ₆ Solution
    create_derivation_step(self,
        step_num=4,
        title="Solving for θ₆ (Wrist Orientation)",
        description="Derived from rotation matrix elements:",
        equations=[
            ("θ₆", f"{sp.pretty(ik_result['theta6'], use_unicode=True)}")
        ],
        color="#7b1fa2"
    )
    add_spacing(self, self.output_layout, 10)
    
    # Step 5: θ₃ Solutions
    create_derivation_step(self,
        step_num=5,
        title="Solving for θ₃ (Elbow Configuration)",
        description="Elbow-up and elbow-down configurations:",
        equations=[
            ("θ₃⁽¹⁾", f"{sp.pretty(ik_result['theta3'][0], use_unicode=True)}"),
            ("θ₃⁽²⁾", f"{sp.pretty(ik_result['theta3'][1], use_unicode=True)}")
        ],
        color="#f57c00"
    )
    add_spacing(self, self.output_layout, 10)
    
    # Step 6: θ₂ Solution
    create_derivation_step(self,
        step_num=6,
        title="Solving for θ₂ (Shoulder Rotation)",
        description="Computed from projected geometry:",
        equations=[
            ("θ₂", f"{sp.pretty(ik_result['theta2'], use_unicode=True)}")
        ],
        color="#00796b"
    )
    add_spacing(self, self.output_layout, 10)
    
    # Step 7: θ₄ Note
    create_derivation_step(self,
        step_num=7,
        title="Solving for θ₄ (Arm Rotation)",
        description="Computed from rotation matrix constraint R₃₄ = R₀₃ᵀ × R₀₆",
        equations=[
            ("θ₄", "Derived from rotation matrices R₀₃ and R₀₄")
        ],
        color="#5e35b1"
    )
    add_spacing(self, self.output_layout, 15)
    
    # ========== Solution Summary ==========
    add_section_header(self, "Solution Branches", "#ff6f00")
    
    summary_group = create_result_group(self, "Total Possible Solutions")
    
    summary_text = QLabel(
        "The symbolic IK produces multiple solution branches:\n\n"
        "• θ₁: 2 solutions (shoulder left/right)\n"
        "• θ₃: 2 solutions (elbow up/down)\n"
        "• θ₅: 2 solutions (wrist flip)\n\n"
        "Total: 2 × 2 × 2 = 8 possible configurations\n\n"
        "Each configuration represents a valid way to reach the target pose."
    )
    summary_text.setFont(self.standard_font)
    summary_text.setWordWrap(True)
    summary_text.setStyleSheet("""
        QLabel {
            background-color: #fff3e0;
            padding: 15px;
            border-left: 4px solid #ff6f00;
            border-radius: 5px;
            color: #333;
            line-height: 1.6;
        }
    """)
    
    summary_group.layout().addWidget(summary_text)
    self.output_layout.addWidget(summary_group)
    add_spacing(self, self.output_layout, 15)
    
    # ========== Note about Numeric Mode ==========
    note_widget = QWidget()
    note_layout = QHBoxLayout(note_widget)
    note_layout.setContentsMargins(15, 12, 15, 12)
    
    icon_lbl = QLabel("💡")
    icon_lbl.setFont(QFont("Segoe UI Emoji", 18))
    
    text_lbl = QLabel(
        "<b>Switch to Numeric Mode</b> to compute actual joint angles for a specific target pose. "
        "The numeric solver will evaluate all 8 solution branches and return only the valid, "
        "reachable configurations."
    )
    text_lbl.setFont(QFont("Roboto", 12))
    text_lbl.setWordWrap(True)
    
    note_widget.setStyleSheet("""
        QWidget {
            background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                stop:0 #e3f2fd, stop:1 #bbdefb);
            border-left: 5px solid #2196f3;
            border-radius: 5px;
        }
    """)
    
    note_layout.addWidget(icon_lbl)
    note_layout.addWidget(text_lbl, stretch=1)
    
    self.output_layout.addWidget(note_widget)


def create_derivation_step(self, step_num, title, description, equations, color):
    """
    Create a styled card for each derivation step
    """
    # Main card container
    card = QFrame()
    card.setStyleSheet(f"""
        QFrame {{
            background-color: white;
            border: 2px solid {color};
            border-radius: 8px;
        }}
    """)
    
    card_layout = QVBoxLayout(card)
    card_layout.setSpacing(0)
    card_layout.setContentsMargins(0, 0, 0, 0)
    
    # Header section
    header = QWidget()
    header.setStyleSheet(f"""
        QWidget {{
            background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                stop:0 {color}, stop:1 {adjust_color_brightness(color, 1.2)});
            border-radius: 6px 6px 0px 0px;
        }}
    """)
    
    header_layout = QHBoxLayout(header)
    header_layout.setContentsMargins(15, 10, 15, 10)
    
    # Step number badge
    step_badge = QLabel(f"{step_num}")
    step_badge.setFont(QFont("Roboto", 14, QFont.Weight.Bold))
    step_badge.setAlignment(Qt.AlignmentFlag.AlignCenter)
    step_badge.setFixedSize(35, 35)
    step_badge.setStyleSheet("""
        QLabel {
            background-color: white;
            color: #333;
            border-radius: 17px;
            padding: 5px;
        }
    """)
    
    # Title
    title_lbl = QLabel(title)
    title_lbl.setFont(QFont("Roboto", 13, QFont.Weight.Bold))
    title_lbl.setStyleSheet("color: white; margin-left: 10px;")
    
    header_layout.addWidget(step_badge)
    header_layout.addWidget(title_lbl)
    header_layout.addStretch()
    
    card_layout.addWidget(header)
    
    # Content section
    content = QWidget()
    content_layout = QVBoxLayout(content)
    content_layout.setContentsMargins(15, 12, 15, 12)
    content_layout.setSpacing(10)
    
    # Description
    desc_lbl = QLabel(description)
    desc_lbl.setFont(QFont("Roboto", 11))
    desc_lbl.setWordWrap(True)
    desc_lbl.setStyleSheet("color: #555; padding-bottom: 5px;")
    content_layout.addWidget(desc_lbl)
    
    # Equations
    for eq_name, eq_expr in equations:
        eq_container = QWidget()
        eq_layout = QHBoxLayout(eq_container)
        eq_layout.setContentsMargins(10, 8, 10, 8)
        eq_layout.setSpacing(10)
        
        eq_container.setStyleSheet(f"""
            QWidget {{
                background-color: #fafafa;
                border-left: 3px solid {color};
                border-radius: 4px;
            }}
        """)
        
        # Equation name
        name_lbl = QLabel(f"<b>{eq_name}</b> =")
        name_lbl.setFont(QFont("Roboto", 12))
        name_lbl.setStyleSheet("background: transparent; color: #333;")
        name_lbl.setFixedWidth(80)
        
        # Equation expression
        expr_lbl = QLabel(eq_expr)
        expr_lbl.setFont(QFont("Roboto Mono", 11))
        expr_lbl.setStyleSheet("background: transparent; color: #1a1a1a;")
        expr_lbl.setWordWrap(True)
        
        eq_layout.addWidget(name_lbl)
        eq_layout.addWidget(expr_lbl, stretch=1)
        
        content_layout.addWidget(eq_container)
    
    card_layout.addWidget(content)
    
    self.output_layout.addWidget(card)


def adjust_color_brightness(hex_color, factor):
    """
    Adjust the brightness of a hex color
    factor > 1 makes it brighter, factor < 1 makes it darker
    """
    # Remove '#' if present
    hex_color = hex_color.lstrip('#')
    
    # Convert to RGB
    r, g, b = tuple(int(hex_color[i:i+2], 16) for i in (0, 2, 4))
    
    # Adjust brightness
    r = min(255, int(r * factor))
    g = min(255, int(g * factor))
    b = min(255, int(b * factor))
    
    # Convert back to hex
    return f"#{r:02x}{g:02x}{b:02x}"


# ========== Update your main_window.py do_ik() method ==========
# Replace the symbolic IK section with:



    # ... rest of your numeric IK code remains the same ...

# def display_ik_symbolic_results(self, T_symbolic, eq_list):
#     """Display symbolic IK representation"""
#     clear_output(self)
    
#     add_section_header(self, "Inverse Kinematics (Symbolic)", "#8e24aa")
    
#     group = create_result_group(self, "Symbolic Target Pose Representation")
    
#     info = QLabel(
#         "For inverse kinematics, the target end-effector pose is represented symbolically as:\n"
#         "• Position: (x, y, z)\n"
#         "• Orientation: Roll-Pitch-Yaw angles (α, β, γ)\n\n"
#         "The transformation matrix is constructed from these parameters:"
#     )
#     info.setFont(self.standard_font)
#     info.setWordWrap(True)
#     info.setStyleSheet("padding: 10px; color: #333;")
#     group.layout().addWidget(info)
    
#     self.output_layout.addWidget(group)
#     add_spacing(self, self.output_layout, 15)
    
#     # Display the symbolic transformation matrix
#     add_symbolic_transformation_matrix_simple(self, T_symbolic, "Target Transformation Matrix T₀⁶")
#     add_spacing(self, self.output_layout, 15)
    
#     # Note about numeric mode
#     note_widget = QWidget()
#     note_layout = QHBoxLayout(note_widget)
#     note_layout.setContentsMargins(10, 8, 10, 8)
    
#     icon_lbl = QLabel("ℹ")
#     icon_lbl.setFont(QFont("Roboto", 14, QFont.Weight.Bold))
#     icon_lbl.setStyleSheet("color: #2196f3;")
    
#     text_lbl = QLabel(
#         "Symbolic IK shows the mathematical relationships and solution procedure. "
#         "To compute numeric solutions, switch to 'Numeric' mode and provide a target pose."
#     )
#     text_lbl.setFont(QFont("Roboto", 11))
#     text_lbl.setWordWrap(True)
    
#     note_widget.setStyleSheet("""
#         QWidget {
#             background-color: #e3f2fd;
#             border-left: 4px solid #2196f3;
#             border-radius: 3px;
#         }
#     """)
    
#     note_layout.addWidget(icon_lbl)
#     note_layout.addWidget(text_lbl)
#     note_layout.addStretch()
    
#     self.output_layout.addWidget(note_widget)
#     add_spacing(self, self.output_layout, 15)
    
    
    
#     # add_equations_section(self, eq_list)
#     add_spacing(self, self.output_layout, 15)
#     self.output_layout.addWidget(group)
    
#     add_spacing(self, self.output_layout, 15)
    
#     # Add export option

    
    
def display_ik_no_solution(self, target_matrix):
    """Display message when no IK solution found"""
    clear_output(self)
    
    add_section_header(self, "Inverse Kinematics Results", "#8e24aa")
    
    # Target Matrix Display
    add_section_header(self, "Target Transformation Matrix and End-Effector Pose", "#00897b")
    add_transformation_matrix_section(self, target_matrix, "Target Transformation Matrix")
    add_spacing(self, self.output_layout, 10)
    
    add_end_effector_pose_section(self, target_matrix)
    add_spacing(self, self.output_layout, 10)
   
   
    add_section_header(self, " ✗ Inverse Kinematics - No Solution", "#c02c2c")
    
    # Error message
    msg_widget = QWidget()
    msg_layout = QHBoxLayout(msg_widget)
    msg_layout.setContentsMargins(10, 8, 10, 8)
    
    icon_lbl = QLabel("✗")
    icon_lbl.setFont(QFont("Roboto", 16, QFont.Weight.Bold))
    icon_lbl.setStyleSheet("color: #f44336;")
    
    text_lbl = QLabel(
        "No valid inverse kinematics solution found.\n"
        "The target pose may be outside the robot's workspace or unreachable."
    )
    text_lbl.setFont(QFont("Roboto", 14))
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
    msg_layout.setAlignment(Qt.AlignmentFlag.AlignCenter)
    
    self.output_layout.addWidget(msg_widget)
    add_spacing(self, self.output_layout, 10)
    

def add_joint_values_display(self, joint_values):
    """Display joint values using your existing grid style"""
    group = create_result_group(self, "Input Joint Values")
    
    grid = QGridLayout()
    grid.setSpacing(10)
    
    angle_unit = "deg" if self.theta_system_widget.currentRow() == 1 else "rad"
    dh_params = self.current_manipulator.get_dh_parameters()
    
    colors = ["#e0f2f1", "#f3e5f5", "#fce4ec", "#b2dfdb", "#e1bee7", "#f8bbd0"]
    
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
    matrix_table.setFont(self.standard_font)
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
                item.setBackground(QColor("#f3e5f5"))
            elif i < 3 and j == 3:  # Translation part
                item.setBackground(QColor("#e0f7fa"))
            else:  # Bottom row
                item.setBackground(QColor("#f5f5f5"))
            
            matrix_table.setItem(i, j, item)
    
    matrix_table.setStyleSheet("""
        QTableWidget {
            border: 2px solid #673ab7;
            border-radius: 5px;
            gridline-color: #ccc;
        }
    """)
    
    matrix_table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
    matrix_table.verticalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
    matrix_table.setMinimumHeight(180)
    matrix_table.setMaximumHeight(220)
    
    group.layout().addWidget(matrix_table)
    
    # Add legend 
    legend = QLabel("Rotation | Translation | Homogeneous")
    legend.setFont(QFont("Roboto", 10))
    legend.setStyleSheet("color: #666; margin-top: 5px;")
    legend.setAlignment(Qt.AlignmentFlag.AlignCenter)
    group.layout().addWidget(legend)
    
    self.output_layout.addWidget(group)


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
            border: 2px solid #673ab7;
            border-radius: 8px;
        }
    """)
    
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
            label.setFont(self.standard_font)
            label.setAlignment(Qt.AlignmentFlag.AlignCenter)
            
            # Color coding
            if i < 3 and j < 3:  # Rotation
                bg_color = "#f3e5f5"
                border_color = "#9c27b0"
            elif i < 3 and j == 3:  # Translation
                bg_color = "#e0f7fa"
                border_color = "#00897b"
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
                    color: #000000;
                }}
            """)
            label.setWordWrap(True)
            
            matrix_layout.addWidget(label, i, j)
    
    container_layout.addWidget(matrix_widget)
    
    # legend
    legend_layout = QHBoxLayout()
    legend_layout.setSpacing(15)
    
    legend_items = [
        (" Rotation (3×3)", "#f3e5f5"),
        (" Translation (3×1)", "#e0f7fa"),
        (" Homogeneous (1×4)", "#f5f5f5")
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
    
    self.output_layout.addWidget(group)



def add_end_effector_pose_section(self, T):
    """Display end-effector pose using your existing style"""
    group = create_result_group(self, "End-Effector Pose")
    
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
        axis_lbl.setFont(self.standard_font)
        axis_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        axis_lbl.setStyleSheet(f"color: {color};")
        
        val_lbl = QLabel(f"{value:.6f}")
        val_lbl.setFont(self.standard_font)
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
    ori_label.setFont(self.standard_font)
    group.layout().addWidget(ori_label)
    
    ori_layout = QHBoxLayout()
    for angle_name, value, color in [("Roll (φ)", roll , "#ef5350"),
                               ("Pitch (θ)", pitch, "#66bb6a"),
                               ("Yaw (ψ)", yaw, "#42a5f5")]:
        lbl = QLabel(f"{angle_name}\n{value:.4f}{unit_str}")
        lbl.setFont(self.standard_font)
        lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        lbl.setStyleSheet(f"""
            QLabel {{
                background-color: #f5f5f5;
                padding: 10px;
                border: 2px solid {color};
                border-radius: 5px;
            }}
        """)
        ori_layout.addWidget(lbl)
    
    group.layout().addLayout(ori_layout)
    self.output_layout.addWidget(group)


def add_ik_solution_display(self, solution, sol_idx, target_matrix, angle_unit):
    """Display IK solution using your existing solution style"""
    group = create_result_group(self, f"Solution {sol_idx}")
    
    # Validity check using FK
    _, _, T_verify = self.current_manipulator.fk_all(solution, sym=False)
    error = np.linalg.norm(T_verify - target_matrix)
    is_valid = error < 1e-5
    
    # Validity indicator 
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
    
    # Joint angles in grid 
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