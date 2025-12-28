"""
Fast 3D Model Viewer using PyQt6 and OpenGL
Smooth rotation, proper colors, no lag

Installation:
pip install PyQt6 PyOpenGL PyOpenGL_accelerate numpy trimesh
"""

from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QPushButton, QLabel, QComboBox, 
                             QGroupBox, QCheckBox)
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtOpenGLWidgets import QOpenGLWidget
from OpenGL.GL import *
from OpenGL.GLU import *
import numpy as np
import trimesh
import sys
import math


class OpenGLViewer(QOpenGLWidget):
    """Fast OpenGL-based 3D viewer"""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.mesh = None
        self.rotation_x = 20
        self.rotation_y = 45
        self.zoom = -3.0
        self.last_pos = None
        self.show_edges = True
        self.wireframe_mode = False
        
        self.show_axes = False
        
        # Vertex and color data
        self.vertices = None
        self.faces = None
        self.colors = None
        self.normals = None
        
        self.target_rotation_x = self.rotation_x
        self.target_rotation_y = self.rotation_y
        self.velocity_x = 0.0
        self.velocity_y = 0.0
        self.target_zoom = self.zoom
        
        # --- SolidWorks-like rotation parameters ---
        self.mouse_sensitivity = 0.5      # lower → slower, more controlled rotation
        self.smoothing_factor = 0.15      # lower → smoother, floaty rotation
        self.momentum_decay = 0.85        # closer to 1 → longer glide after release
        self.velocity_threshold = 0.01

        self.is_dragging = False
        
        # Timer for smooth updates (~60 FPS)
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_rotation)
        self.timer.start(16)  # 16 ms ≈ 60 FPS
        
    def initializeGL(self):
        """Initialize OpenGL settings"""
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_LIGHTING)
        glEnable(GL_LIGHT0)
        glEnable(GL_COLOR_MATERIAL)
        glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE)
        
        glEnable(GL_NORMALIZE)
        glShadeModel(GL_SMOOTH)
        
        # Set up lighting
        glLightfv(GL_LIGHT0, GL_POSITION, [1.0, 1.0, 1.0, 0.0])
        glLightfv(GL_LIGHT0, GL_AMBIENT, [0.3, 0.3, 0.3, 1.0])
        glLightfv(GL_LIGHT0, GL_DIFFUSE, [0.8, 0.8, 0.8, 1.0])
        glLightfv(GL_LIGHT0, GL_SPECULAR, [1.0, 1.0, 1.0, 1.0])
        
        # Background color (white)
        glClearColor(1.0, 1.0, 1.0, 1.0)
        
    def resizeGL(self, w, h):
        """Handle window resize"""
        glViewport(0, 0, w, h)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(45, w / h if h != 0 else 1, 0.1, 100.0)
        glMatrixMode(GL_MODELVIEW)
        
    def paintGL(self):
        """Render the scene"""
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()
        
        # Camera position
        glTranslatef(0.0, 0.0, self.zoom)
        glRotatef(self.rotation_x, 1, 0, 0)
        glRotatef(self.rotation_y, 0, 1, 0)
        
        self.draw_axes()
        
        if self.mesh is not None and self.vertices is not None:
            self.draw_mesh()
            
    def draw_mesh(self):
        """Draw the 3D mesh"""
        if self.wireframe_mode:
            glDisable(GL_LIGHTING)
            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE)
            glColor3f(0.0, 0.0, 0.0)
        else:
            glEnable(GL_LIGHTING)
            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL)

        # Draw faces
        glBegin(GL_TRIANGLES)
        for i in range(len(self.faces)):  # Changed from 'for i, face in enumerate(self.faces)'
            face = self.faces[i]

            # Set color for this face
            if self.colors is not None and i < len(self.colors):
                color = self.colors[i]
                glColor3f(float(color[0]), float(color[1]), float(color[2]))
            else:
                glColor3f(0.7, 0.7, 0.7)

            # Draw triangle with normals
            for j in range(len(face)):  # Changed from 'for j, vertex_idx in enumerate(face)'
                vertex_idx = face[j]

                if self.normals is not None and i < len(self.normals):
                    normal = self.normals[i]
                    glNormal3f(float(normal[0]), float(normal[1]), float(normal[2]))

                vertex = self.vertices[vertex_idx]
                glVertex3f(float(vertex[0]), float(vertex[1]), float(vertex[2]))
        glEnd()

    # Draw edges if enabled
        if self.show_edges and not self.wireframe_mode:
            glDisable(GL_LIGHTING)
            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE)
            glColor3f(0.3, 0.3, 0.3)
            glLineWidth(1.0)

            glBegin(GL_TRIANGLES)
            for i in range(len(self.faces)):  # Changed iteration
                face = self.faces[i]
                for j in range(len(face)):  # Changed iteration
                    vertex_idx = face[j]
                    vertex = self.vertices[vertex_idx]
                    glVertex3f(float(vertex[0]), float(vertex[1]), float(vertex[2]))
            glEnd()

            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL)
            glEnable(GL_LIGHTING)

    def load_model(self, filepath):
        """Load 3D model from file"""
        try:
            loaded = trimesh.load(filepath, force='mesh', process=False)
            if loaded.visual.kind != 'vertex' and hasattr(loaded.visual, 'to_color'):
                loaded.visual = loaded.visual.to_color()
            
            # Handle Scene with multiple parts
            if isinstance(loaded, trimesh.Scene):
                print(f"\nScene with {len(loaded.geometry)} parts detected")
                meshes = []
                for name, geom in loaded.geometry.items():
                    if isinstance(geom, trimesh.Trimesh):
                        meshes.append(geom)
                        print(f"  Part: {name}, Vertices: {len(geom.vertices)}, Faces: {len(geom.faces)}")
                
                if meshes:
                    self.mesh = trimesh.util.concatenate(meshes)
                    print(f"\nCombined: {len(self.mesh.vertices)} vertices, {len(self.mesh.faces)} faces")
                else:
                    return False
            else:
                self.mesh = loaded
                print(f"Single mesh: {len(self.mesh.vertices)} vertices, {len(self.mesh.faces)} faces")
            
            # Simplify if too complex
            if len(self.mesh.faces) > 50000:
                print(f"Simplifying from {len(self.mesh.faces)} faces...")
                self.mesh = self.simplify_mesh(self.mesh, 30000)
                print(f"Simplified to {len(self.mesh.faces)} faces")
            
            # Prepare data for OpenGL
            self.prepare_mesh_data()
            
            # Center and scale the mesh
            self.center_mesh()
            
            self.update()
            return True
            
        except Exception as e:
            print(f"Error loading: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def simplify_mesh(self, mesh, target_faces):
        """Simplify mesh"""
        try:
            return mesh.simplify_quadric_decimation(target_faces)
        except:
            try:
                spacing = mesh.scale / 30
                return mesh.simplify_vertex_clustering(spacing=spacing)
            except:
                return mesh
    
    def prepare_mesh_data(self):
        """Extract and prepare mesh data for rendering"""
        self.vertices = np.array(self.mesh.vertices, dtype=np.float32)
        self.faces = np.array(self.mesh.faces, dtype=np.int32)
    # Extract colors
        self.colors = self.get_face_colors()
    # Calculate face normals for lighting
        self.normals = np.array(self.mesh.face_normals, dtype=np.float32)
        
    def get_face_colors(self):
        """Extract face colors from mesh"""
        try:
            # Try direct face_colors
            if hasattr(self.mesh.visual, 'face_colors'):
                colors = self.mesh.visual.face_colors
                # Normalize to 0-1
                if colors.max() > 1.0:
                    colors = colors / 255.0
                return colors[:, :3]
        except:
            pass
        
        # Try to get material color
        try:
            if hasattr(self.mesh.visual, 'material'):
                mat = self.mesh.visual.material
                if hasattr(mat, 'diffuse'):
                    color = mat.diffuse[:3]
                    if color.max() > 1.0:
                        color = color / 255.0
                    return np.tile(color, (len(self.mesh.faces), 1))
        except:
            pass
        
        # Try to convert TextureVisuals to ColorVisuals
        try:
            self.mesh.visual = self.mesh.visual.to_color()
            if hasattr(self.mesh.visual, 'face_colors'):
                colors = self.mesh.visual.face_colors
                if colors.max() > 1.0:
                    colors = colors / 255.0
                return colors[:, :3]
        except:
            pass
        
        # Default gray
        return np.array([[0.7, 0.7, 0.7]] * len(self.mesh.faces))
    
    def center_mesh(self):
        """Center and scale mesh to fit view"""
        if self.mesh is None:
            return
        
        # Center
        center = self.mesh.bounds.mean(axis=0)
        self.vertices = self.vertices - center
        
        # Scale to unit size
        max_extent = np.max(self.mesh.extents)
        if max_extent > 0:
            scale = 2.0 / max_extent
            self.vertices = self.vertices * scale
    
    def mousePressEvent(self, event):
        """Handle mouse press"""
        self.last_pos = event.position()
        self.is_dragging = True
        # Stop momentum when user starts dragging
        self.velocity_x = 0.0
        self.velocity_y = 0.0
    
    def mouseReleaseEvent(self, event):
        """Handle mouse release"""
        self.is_dragging = False
    
    def mouseMoveEvent(self, event):
        """Handle mouse drag for rotation"""
        if self.last_pos is None:
            return
        
        dx = event.position().x() - self.last_pos.x()
        dy = event.position().y() - self.last_pos.y()
        
        if event.buttons() & Qt.MouseButton.LeftButton:
            self.target_rotation_x += dy * self.mouse_sensitivity
            self.target_rotation_y += dx * self.mouse_sensitivity
            if self.is_dragging:
                self.velocity_x = dy * self.mouse_sensitivity * 0.2
                self.velocity_y = dx * self.mouse_sensitivity * 0.2
 
            # self.rotation_x += (self.target_rotation_x - self.rotation_x) * 0.2
            # self.rotation_y += (self.target_rotation_y - self.rotation_y) * 0.2
        elif event.buttons() & Qt.MouseButton.RightButton:
            self.target_zoom += dy * 0.01
            # self.zoom += dy * 0.01
        
        self.last_pos = event.position()
        
    def update_rotation(self):
        self.target_zoom = max(-20.0, min(-1.0, self.target_zoom))
        # Smoothly move rotation toward target
        self.rotation_x += (self.target_rotation_x - self.rotation_x) * self.smoothing_factor
        self.rotation_y += (self.target_rotation_y - self.rotation_y) * self.smoothing_factor

        if not self.is_dragging:
            # Stop momentum if velocity is very small
            if abs(self.velocity_x) < self.velocity_threshold:
                self.velocity_x = 0.0
            if abs(self.velocity_y) < self.velocity_threshold:
                self.velocity_y = 0.0
                
        # Apply momentum
        self.rotation_x += self.velocity_x
        self.rotation_y += self.velocity_y
        self.target_rotation_x += self.velocity_x
        self.target_rotation_y += self.velocity_y
        
        self.velocity_x *= self.momentum_decay
        self.velocity_y *= self.momentum_decay

        # Smooth zoom
        self.zoom += (self.target_zoom - self.zoom) * self.smoothing_factor

        self.update()

    def wheelEvent(self, event):
        """Handle mouse wheel for zoom"""
        delta = event.angleDelta().y() / 120 
        self.target_zoom += delta * 0.15
        self.target_zoom = max(-20.0, min(-1.0, self.target_zoom))
        self.update()
    
    def set_view(self, elev, azim, zoom=None):
        """Set view angle"""
        self.target_rotation_x = elev
        self.target_rotation_y = azim
        self.rotation_x = elev
        self.rotation_y = azim
        
        self.velocity_x = 0
        self.velocity_y = 0
        
        if zoom is not None:
            self.target_zoom = zoom
            self.zoom = zoom
        self.update()
    
    def toggle_edges(self, show):
        """Toggle edge display"""
        self.show_edges = show
        self.update()
    
    def toggle_wireframe(self, wireframe):
        """Toggle wireframe mode"""
        self.wireframe_mode = wireframe
        self.update()

    def draw_axes(self):
        """Draw a small axis indicator in the bottom-left corner"""
        if not self.show_axes:
            return
        # Save current matrices
        glMatrixMode(GL_PROJECTION)
        glPushMatrix()
        glMatrixMode(GL_MODELVIEW)
        glPushMatrix()

        # Disable lighting for axes
        lighting_was_enabled = glIsEnabled(GL_LIGHTING)
        glDisable(GL_LIGHTING)
        
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)

        # Set up a small viewport in the corner
        viewport = glGetIntegerv(GL_VIEWPORT)
        width = viewport[2]
        height = viewport[3]

        # Create small viewport in bottom-left corner (100x100 pixels)
        axis_size = 150         
        axis_line_length = 1.0   # How long the axis lines are (increase for longer)
        axis_line_width =1.0    # Thickness of axis lines (increase for thicker)
        label_size = 0.12
        glViewport(10, 10, axis_size, axis_size)

        # Set up orthographic projection for the axis indicator
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        glOrtho(-1.5, 1.5, -1.5, 1.5, -10, 10)

        # Set up modelview with same rotation as main view
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        glRotatef(self.rotation_x, 1, 0, 0)
        glRotatef(self.rotation_y, 0, 1, 0)
        
        glDisable(GL_DEPTH_TEST)
        glColor4f(0.7, 0.95, 0.95, 0.9)  # Light teal (more visible)
        glBegin(GL_TRIANGLE_FAN)
        glVertex3f(0, 0, -0.1)

        segments = 32
        radius = 1.3
        for i in range(segments + 1):
            angle = 2.0 * math.pi * i / segments
            glVertex3f(radius * math.cos(angle), radius * math.sin(angle), -0.1)
        glEnd()
        glEnable(GL_DEPTH_TEST)
        # Draw the axes
        glLineWidth(axis_line_width)
        glBegin(GL_LINES)

        # X-axis (Red)
        glColor3f(0.9, 0.2, 0.2)
        glVertex3f(0, 0, 0)
        glVertex3f(axis_line_length, 0, 0)

        # Y-axis (Green)
        glColor3f(0.2, 0.8, 0.2)
        glVertex3f(0, 0, 0)
        glVertex3f(0, axis_line_length, 0)

        # Z-axis (Blue)
        glColor3f(0.2, 0.2, 0.9)
        glVertex3f(0, 0, 0)
        glVertex3f(0, 0, axis_line_length)

        glEnd()

        # Draw arrow heads for each axis
        self.draw_arrow_head(axis_line_length, 0, 0, 0.9, 0.2, 0.2)  # X
        self.draw_arrow_head(0, axis_line_length, 0, 0.2, 0.8, 0.2)  # Y
        self.draw_arrow_head(0, 0, axis_line_length, 0.2, 0.2, 0.9) # Z

        # Draw axis labels using simple geometry
        glLineWidth(2.5)
        label_offset = axis_line_length + 0.3
        self.draw_axis_label_X(label_offset, 0, 0, 0.9, 0.2, 0.2, label_size)
        self.draw_axis_label_Y(0, label_offset, 0, 0.2, 0.8, 0.2, label_size)
        self.draw_axis_label_Z(0, 0, label_offset, 0.2, 0.2, 0.9, label_size)

        glLineWidth(1.0)

        # Restore viewport and matrices
        glViewport(0, 0, width, height)

        glMatrixMode(GL_PROJECTION)
        glPopMatrix()
        glMatrixMode(GL_MODELVIEW)
        glPopMatrix()

        # Restore lighting state
        glDisable(GL_BLEND)
        if lighting_was_enabled:
            glEnable(GL_LIGHTING)
            
        # Add this method to OpenGLViewer class
    def toggle_axes(self, show):
        """Toggle coordinate axes display"""
        self.show_axes = show
        self.update()

    def draw_arrow_head(self, x, y, z, r, g, b):
        """Draw a small arrow head at the end of an axis"""
        glColor3f(r, g, b)

        # Determine which axis this is
        cone_base = 0.8
        cone_height = 0.25
        cone_radius = 0.1

        if x > 0.5:  # X-axis
            # Cone pointing in +X direction
            self.draw_cone(cone_base, 0, 0, 1, 0, 0, cone_radius, cone_height)
        elif y > 0.5:  # Y-axis
            # Cone pointing in +Y direction
            self.draw_cone(0, cone_base, 0, 0, 1, 0, cone_radius, cone_height)
        elif z > 0.5:  # Z-axis
            # Cone pointing in +Z direction
            self.draw_cone(0, 0, cone_base, 0, 0, 1, cone_radius, cone_height)

    def draw_cone(self, base_x, base_y, base_z, dir_x, dir_y, dir_z, radius, height):
        """Draw a simple cone for arrow head"""
        segments = 12  # More segments for smoother cone
        glBegin(GL_TRIANGLE_FAN)
        # Tip of cone
        glVertex3f(base_x + dir_x * height, 
                   base_y + dir_y * height, 
                   base_z + dir_z * height)
        # Base circle
        for i in range(segments + 1):
            angle = 2.0 * math.pi * i / segments
            # Calculate perpendicular vectors for the base circle
            if abs(dir_x) < 0.9:
                perp1_x, perp1_y, perp1_z = 1, 0, 0
            else:
                perp1_x, perp1_y, perp1_z = 0, 1, 0
            # Cross product to get second perpendicular
            perp2_x = dir_y * perp1_z - dir_z * perp1_y
            perp2_y = dir_z * perp1_x - dir_x * perp1_z
            perp2_z = dir_x * perp1_y - dir_y * perp1_x
            # Normalize
            length = math.sqrt(perp2_x**2 + perp2_y**2 + perp2_z**2)
            if length > 0:
                perp2_x /= length
                perp2_y /= length
                perp2_z /= length
            # Calculate point on base circle
            px = base_x + radius * (math.cos(angle) * perp1_x + math.sin(angle) * perp2_x)
            py = base_y + radius * (math.cos(angle) * perp1_y + math.sin(angle) * perp2_y)
            pz = base_z + radius * (math.cos(angle) * perp1_z + math.sin(angle) * perp2_z)
            glVertex3f(px, py, pz)
        glEnd()
        
    def draw_axis_label_X(self, x, y, z, r, g, b, size=0.08):
        """Draw 'X' label using lines"""
        glColor3f(r, g, b)
        glBegin(GL_LINES)
        
        # X as two crossing lines
        glVertex3f(x - size, y - size, z)
        glVertex3f(x + size, y + size, z)
        glVertex3f(x - size, y + size, z)
        glVertex3f(x + size, y - size, z)
        glEnd()

    def draw_axis_label_Y(self, x, y, z, r, g, b, size=0.08):
        """Draw 'Y' label using lines"""
        glColor3f(r, g, b)
        glBegin(GL_LINES)
        # Y as two lines meeting at center, plus stem
        glVertex3f(x - size, y + size, z)
        glVertex3f(x, y, z)
        glVertex3f(x + size, y + size, z)
        glVertex3f(x, y, z)
        glVertex3f(x, y, z)
        glVertex3f(x, y - size, z)
        glEnd()

    def draw_axis_label_Z(self, x, y, z, r, g, b, size=0.08):
        """Draw 'Z' label using lines"""
        glColor3f(r, g, b)
        glBegin(GL_LINES)
        # Z as horizontal line, diagonal, horizontal line
        glVertex3f(x - size, y + size, z)
        glVertex3f(x + size, y + size, z)
        glVertex3f(x + size, y + size, z)
        glVertex3f(x - size, y - size, z)
        glVertex3f(x - size, y - size, z)
        glVertex3f(x + size, y - size, z)
        glEnd()


    # Optional: Add a background circle/square behind the axes for better visibility
    def draw_axis_background(self):
        """Draw a subtle background behind the axis indicator"""
        glDisable(GL_DEPTH_TEST)

        # Draw a semi-transparent circle
        glColor4f(0.95, 0.95, 0.95, 0.8)
        glBegin(GL_TRIANGLE_FAN)
        glVertex3f(0, 0, -0.1)

        segments = 32
        radius = 1.3
        for i in range(segments + 1):
            angle = 2.0 * math.pi * i / segments
            glVertex3f(radius * math.cos(angle), radius * math.sin(angle), -0.1)

        glEnd()

        glEnable(GL_DEPTH_TEST)


class ManipulatorViewer(QWidget):
    """Main viewer widget"""
    
    def __init__(self):
        super().__init__()
        self.viewer = None
        self.init_ui()
    
    def init_ui(self):
        layout = QVBoxLayout()
        
        # Control panel
        control_group = QGroupBox("Controls")
        control_layout = QHBoxLayout()
        
        # Model selector
        self.model_combo = QComboBox()
        self.model_combo.addItems(["UR5", "Manipulator 2", "Manipulator 3"])
        self.model_combo.currentIndexChanged.connect(self.load_model)
        control_layout.addWidget(QLabel("Model:"))
        control_layout.addWidget(self.model_combo)
        
        # View presets
        views_group = QGroupBox("View Presets")
        views_layout = QHBoxLayout()
        
        iso_btn = QPushButton("Isometric")
        iso_btn.clicked.connect(lambda: self.viewer.set_view(35, 45, zoom=-3))
        views_layout.addWidget(iso_btn)
        
        top_btn = QPushButton("Top")
        top_btn.clicked.connect(lambda: self.viewer.set_view(90, 0, zoom=-3))
        views_layout.addWidget(top_btn)
        
        front_btn = QPushButton("Front")
        front_btn.clicked.connect(lambda: self.viewer.set_view(0, 0, zoom=-3))
        views_layout.addWidget(front_btn)
        
        side_btn = QPushButton("Side")
        side_btn.clicked.connect(lambda: self.viewer.set_view(0, 90, zoom=-3))
        views_layout.addWidget(side_btn)
        
        views_group.setLayout(views_layout)
        control_layout.addWidget(views_group)
        
        # Display options
        self.edges_check = QCheckBox("Show Edges")
        self.edges_check.setChecked(True)
        self.edges_check.toggled.connect(self.toggle_edges)
        control_layout.addWidget(self.edges_check)
        
        self.wireframe_check = QCheckBox("Wireframe")
        self.wireframe_check.toggled.connect(self.toggle_wireframe)
        control_layout.addWidget(self.wireframe_check)
        
        control_layout.addStretch()
        control_group.setLayout(control_layout)
        layout.addWidget(control_group)
        
        # OpenGL viewer
        self.viewer = OpenGLViewer(self)
        layout.addWidget(self.viewer, stretch=1)
        
        # Info label
        self.info_label = QLabel("Left-click drag: Rotate | Right-click drag: Zoom | Mouse wheel: Zoom")
        layout.addWidget(self.info_label)
        
        self.setLayout(layout)
        
        # Load first model
        self.load_model(0)
        
    def load_model(self, index):
        """Load model file"""
        models = [
            'cad_models/ur5.obj',
            'cad_models/irb_1600_10kg_1.45m.obj',
            'cad_models/ur5.obj'
        ]
        
        filepath = models[index]
        success = self.viewer.load_model(filepath)
        
        if success and self.viewer.mesh:
            self.info_label.setText(
                f"Loaded: {filepath.split('/')[-1]} | "
                f"Vertices: {len(self.viewer.mesh.vertices):,} | "
                f"Faces: {len(self.viewer.mesh.faces):,} | "
                f"Left drag: Rotate | Right drag/Wheel: Zoom"
            )
        else:
            self.info_label.setText(f"Failed to load {filepath}")
    
    def toggle_edges(self, checked):
        self.viewer.toggle_edges(checked)
    
    def toggle_wireframe(self, checked):
        self.viewer.toggle_wireframe(checked)


class MainWindow(QMainWindow):
    """Main window"""
    
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robot Manipulator Viewer - OpenGL")
        self.setGeometry(100, 100, 1400, 900)
        
        self.viewer = ManipulatorViewer()
        self.setCentralWidget(self.viewer)


def main():
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())


if __name__ == '__main__':
    main()