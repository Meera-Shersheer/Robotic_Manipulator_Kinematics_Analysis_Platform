# 🤖 Robotic Manipulator Kinematics Analysis Platform

A comprehensive desktop application for forward and inverse kinematics analysis of industrial robotic manipulators, featuring both symbolic and numeric computation modes with interactive 3D visualization.

---

## 📋 Table of Contents

- [Features](#-features)
- [Supported Robot Models](#-supported-robot-models)
- [System Requirements](#-system-requirements)
- [Installation](#-installation)
- [Quick Start](#-quick-start)
- [User Guide](#-user-guide)
- [Project Structure](#-project-structure)
- [Technical Details](#-technical-details)
- [Troubleshooting](#-troubleshooting)

---

## ✨ Features

### Core Functionality

#### Forward Kinematics (FK)
- **Numeric Mode**: Calculate end-effector pose from joint angles
- **Symbolic Mode**: Generate symbolic transformation matrices with joint variables
- **Frame Selection**: Display all frames or select specific range (Frame 0 to Frame 6)
- **Individual & Cumulative Transformations**: View both link-to-link and base-to-frame transformations

#### Inverse Kinematics (IK)
- **Closed-Form Solutions**: Analytical IK for all supported robots
- **Multiple Solutions**: Find all valid configurations (up to 8 solutions per target)
- **Symbolic Derivation**: Step-by-step symbolic solution with detailed explanation
- **Two Input Methods**:
  - Position & Orientation (x, y, z, α, β, γ)
  - 4×4 Homogeneous Transformation Matrix

### Visualization

#### 3D CAD Viewer
- Interactive OpenGL-based 3D model viewer
- Smooth rotation with momentum-based controls
- Preset views: Isometric, Top, Front, Side
- Custom view controls with elevation and azimuth
- Zoom controls with smooth animation
- Display options:
  - Show/Hide edges
  - Wireframe mode
  - Coordinate axes indicator

#### 2D Technical Drawings
- DH parameter diagrams for each robot
- Cross-sectional views and technical specifications
- High-quality engineering drawings

### User Interface

- **Professional Layout**: Clean, organized tabbed interface
- **Real-time Updates**: All calculations update instantly
- **Unit Conversion**: Switch between radians and degrees
- **DH Parameter Table**: Interactive table with automatic population
- **Color-coded Outputs**: Visual distinction between position/orientation components

---

## 🤖 Supported Robot Models

### 1. Universal Robots UR5
- **Type**: 6-DOF Industrial Collaborative Robot
- **Reach**: ~850mm
- **Configuration**: All revolute joints
- **DH Convention**: Modified DH parameters
- **Applications**: Assembly, pick-and-place, machine tending

### 2. ABB IRB 1600
- **Type**: 6-DOF Industrial Robot
- **Reach**: ~1.45m (with 10kg payload variant)
- **Configuration**: All revolute joints
- **Workspace**: Large working envelope
- **Applications**: Material handling, welding, assembly

### 3. KUKA KR16
- **Type**: 6-DOF Industrial Robot
- **Reach**: ~1.6m
- **Configuration**: All revolute joints
- **Payload**: 16kg
- **Applications**: Arc welding, handling, assembly


## 🚀 Installation

1. **Clone or Download the Repository**
   ```bash
   git clone <repository-url>
   cd Robotic_Manipulator_Kinematics_Analysis_Platform
   ```

2. **Run Setup with Make**
   ```bash
   make venv    # Create virtual environment
   make install # Install dependencies
   make run     # Launch application
   ```

   Or use the all-in-one command:
   ```bash
   make all
   ```

### Manual Installation (Without Make)

If Make is not available:

```bash

# Linux/wsl
python3 -m venv .venv
source .venv/bin/activate
pip install --upgrade pip
pip install -r requirements.txt
python main.py
```

---

## 🎮 Quick Start

### Running the Application

Using Make (recommended):
```bash
make run
```

Or directly with Python:
```bash
# Activate virtual environment first
# Windows:
.venv\Scripts\activate

# Linux/macOS:
source .venv/bin/activate

# Then run:
python main.py
```

### Basic Workflow

1. **Select Robot**: Choose from UR5, ABB IRB 1600, or KUKA KR16
2. **Choose Mode**: Forward Kinematics (FK) or Inverse Kinematics (IK)
3. **Select Computation**: Symbolic or Numeric
4. **Enter Input**: Joint values (FK) or target pose (IK)
5. **Calculate**: Click the "Calculate" button
6. **View Results**: Check the "Outputs" tab for detailed results
7. **Visualize**: Switch to "3D CAD Model" tab to see the robot

---

## 📖 User Guide

### Tab 1: Inputs

#### Robot Selection
- Use the **"Select a manipulator"** list to choose your robot
- The DH parameters table updates automatically

#### Calculation Type
- **Forward Kinematics (FK)**: Calculate end-effector pose from joint angles
- **Inverse Kinematics (IK)**: Calculate joint angles for desired end-effector pose

#### Computation Mode
- **Symbolic**: Shows mathematical expressions and relationships
- **Numeric**: Computes actual numerical values

#### Angle Units
- **Radians**: Standard mathematical unit (default for calculations)
- **Degrees**: More intuitive for engineering applications
- Note: All angles are stored internally in radians

#### DH Parameters Table
- **Fixed Parameters**: θ, d, a, α (grayed out cells)
- **Variable Column**: Shows which parameter is the joint variable (θᵢ or dᵢ)
- **Value Column**: Enter joint values here (only in Numeric FK mode)

#### Forward Kinematics Options
- **All Frames**: Display transformations for all frames (0→1, 0→2, ..., 0→6)
- **Range**: Select specific frame range using From/To spinboxes

#### Inverse Kinematics Input Methods

**Method 1: Position & Orientation**
- Enter X, Y, Z coordinates (meters)
- Enter Roll (α), Pitch (β), Yaw (γ) angles
- Convention: ZYX Euler angles

**Method 2: Transformation Matrix**
- Enter full 4×4 homogeneous transformation matrix
- Rotation part (3×3, top-left)
- Translation part (3×1, top-right)
- Homogeneous row [0, 0, 0, 1]

### Tab 2: Outputs

#### Result Display Sections

**Forward Kinematics Results**
1. **Input Joint Values**: Color-coded display of entered joint angles/positions
2. **Individual Link Transformations**: Transformation from frame i to frame i+1
3. **Cumulative Transformations**: Transformation from base (frame 0) to frame i
4. **End-Effector Pose**: Final position and orientation
   - Position in X, Y, Z (meters)
   - Orientation in Roll, Pitch, Yaw

**Inverse Kinematics Results**
1. **Target Pose**: Input transformation matrix and pose breakdown
2. **Solution Count**: Number of valid configurations found
3. **Individual Solutions**: Each solution shows:
   - All 6 joint angles
   - Validation status (✓ or ⚠)
   - FK verification error
4. **Symbolic Derivation** (Symbolic mode):
   - Step-by-step solution procedure
   - Wrist center calculation
   - Joint angle derivations
   - Solution branches explanation

### Tab 3: 3D CAD Model

#### Model Controls
- **Mouse Controls**:
  - Left-click + Drag: Rotate model
  - Right-click + Drag: Zoom
  - Scroll Wheel: Zoom in/out
  - Smooth momentum-based rotation

#### View Presets
- **Isometric**: 35° elevation, 45° azimuth (default engineering view)
- **Top**: Bird's eye view (90° elevation)
- **Front**: Straight-on view (0° elevation, 0° azimuth)
- **Side**: Side profile (0° elevation, 90° azimuth)

#### Custom View
- **Elevation**: Vertical angle (-180° to +180°)
- **Azimuth**: Horizontal rotation (-180° to +180°)
- Use + / - buttons or enter values directly

#### Display Options
- **Show Edges**: Highlight mesh edges (black wireframe overlay)
- **Wireframe Mode**: Show only edges (no filled surfaces)
- **Show Coordinate Axes**: Display X-Y-Z axes indicator in corner
  - Red: X-axis
  - Green: Y-axis
  - Blue: Z-axis

### Tab 4: 2D Sections

- View engineering drawings and DH parameter diagrams
- Technical cross-sections for each robot
- Reference diagrams showing frame assignments

---

## 📁 Project Structure

```
robotics-kinematics/
├── main.py                    # Application entry point
├── imports.py                 # Centralized imports
├── base_manipulator.py        # Robot base class & implementations
├── input_constrains.py        # Input validators
├── requirements.txt           # Python dependencies
├── Makefile                   # Build automation
├── .gitignore                # Git exclusions
│
├── gui/
│   ├── main_window.py        # Main application window
│   ├── output.py             # Result display functions
│   └── cad_viewer.py         # 3D OpenGL viewer
│
└── cad_models/
    ├── ur5.obj               # UR5 3D model
    ├── UR5_DH.jpeg           # UR5 DH diagram
    ├── UR5_2D.png            # UR5 2D sections
    ├── irb_1600_10kg_1.45m.obj
    ├── ABB_IRB_1600_DH.jpeg
    ├── ABB_2D.png
    ├── kuka_kr16_assembly.obj
    ├── KUKA_DH.jpeg
    └── KUKA_2D.png
```

---

## 🔧 Technical Details

### Denavit-Hartenberg (DH) Convention

The application uses **Standard DH Convention** with parameters:
- **θ (theta)**: Joint angle around Z-axis
- **d**: Link offset along Z-axis
- **a**: Link length along X-axis
- **α (alpha)**: Link twist around X-axis

Transformation matrix formula:
```
T = [ cos(θ)  -sin(θ)cos(α)   sin(θ)sin(α)   a·cos(θ) ]
    [ sin(θ)   cos(θ)cos(α)  -cos(θ)sin(α)   a·sin(θ) ]
    [   0         sin(α)          cos(α)          d     ]
    [   0           0               0            1     ]
```

### Inverse Kinematics Algorithm

**Method**: Closed-form analytical solution for 6R manipulators with spherical wrist

**Solution Strategy**:
1. **Wrist Center Calculation**: p₀₅ = p₀₆ - d₆·R₀₆[:, 2]
2. **First Three Joints** (Position): Geometric solution for shoulder and elbow
3. **Last Three Joints** (Orientation): Euler angle extraction from rotation matrix

**Solution Branches**:
- θ₁: 2 solutions (shoulder left/right)
- θ₃: 2 solutions (elbow up/down)
- θ₅: 2 solutions (wrist flip)
- **Total**: Up to 8 unique configurations

**Verification**: Each solution is verified using forward kinematics

### Coordinate Systems

- **Base Frame**: World coordinate system at robot base
- **End-Effector Frame**: Tool center point (TCP)
- **Euler Angles**: ZYX convention (Yaw-Pitch-Roll)
  - Roll (α): Rotation around X-axis
  - Pitch (β): Rotation around Y-axis
  - Yaw (γ): Rotation around Z-axis

---

## 🛠️ Makefile Commands

The Makefile provides cross-platform automation:

```bash
make venv      # Create virtual environment
make install   # Install Python dependencies
make run       # Launch the application
make clean     # Remove cache files (__pycache__)
make fclean    # Remove virtual environment + cache
make re        # Clean rebuild (fclean + venv + install)
make all       # Full setup and run (default)
```

**Note**: The Makefile automatically detects your operating system (Windows, Linux, macOS) and uses appropriate commands.

---

## 🐛 Troubleshooting

### Common Issues

#### 1. **"Python not found" or "python3 not found"**
**Solution**: Ensure Python 3.8+ is installed and in your PATH
```bash
# Check Python version
python --version  # or python3 --version
```

#### 2. **OpenGL/Graphics Issues**
**Symptoms**: Black screen, graphics glitches, slow rendering

**Solutions**:
- Update graphics drivers
- Install OpenGL libraries:
  ```bash
  # Ubuntu/Debian
  sudo apt install libgl1-mesa-glx libglu1-mesa
  
  # macOS - usually pre-installed
  ```

#### 3. **"ModuleNotFoundError" when running**
**Solution**: Activate virtual environment first
```bash
# Windows
.venv\Scripts\activate

# Linux/macOS
source .venv/bin/activate
```

#### 4. **CAD Models Not Loading**
**Solution**: Ensure `cad_models/` folder exists with all .obj and image files

#### 5. **Make Commands Not Working on Windows**
**Solutions**:
- Install Make for Windows via [Chocolatey](https://chocolatey.org/):
  ```bash
  choco install make
  ```
- Or use manual installation commands (see Installation section)

#### 6. **High DPI Display Issues**
**Solution**: Add this to your Python environment before running:
```bash
export QT_AUTO_SCREEN_SCALE_FACTOR=1  # Linux/macOS
set QT_AUTO_SCREEN_SCALE_FACTOR=1     # Windows
```

### Performance Tips

1. **3D Viewer Lag**:
   - Models are automatically simplified if they have >50,000 faces
   - Try disabling "Show Edges" for complex models
   - Close other GPU-intensive applications

2. **Symbolic Computation Speed**:
   - Symbolic mode is slower due to mathematical expression manipulation
   - For quick calculations, use Numeric mode

---

### Project Highlights

- **No External Dependencies for DH Parameters**: All robot configurations are hardcoded
- **Efficient 3D Rendering**: Automatic mesh simplification for performance
- **Robust IK Solver**: Multiple solution branches with FK verification
- **User-Friendly**: Intuitive interface with helpful tooltips and visual feedback

---

## 🎓 Educational Value

This application is ideal for:
- **Robotics Students**: Understanding FK/IK concepts with visualization
- **Engineers**: Quick calculations for robot programming
- **Researchers**: Prototyping and verification of kinematic algorithms
- **Educators**: Demonstrating robotics principles with interactive examples

---

## 📧 Support

For issues, questions, or contributions:
1. Check the Troubleshooting section above
2. Review the code documentation in source files
3. Create an issue in the repository

---

## 📄 License

[Add your license information here]

---

**Enjoy exploring robotic kinematics! 🤖✨**
