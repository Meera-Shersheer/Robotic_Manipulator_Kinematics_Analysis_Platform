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
        self.smoothing_factor = 0.1      # lower → smoother, floaty rotation
        self.momentum_decay = 0.6        # closer to 1 → longer glide after release


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
        for i, face in enumerate(self.faces):
            # Set color for this face
            if self.colors is not None and i < len(self.colors):
                color = self.colors[i]
                glColor3f(color[0], color[1], color[2])
            else:
                glColor3f(0.7, 0.7, 0.7)
            
            # Draw triangle with normals
            for j, vertex_idx in enumerate(face):
                if self.normals is not None and i < len(self.normals):
                    normal = self.normals[i]
                    glNormal3f(normal[0], normal[1], normal[2])
                
                vertex = self.vertices[vertex_idx]
                glVertex3f(vertex[0], vertex[1], vertex[2])
        glEnd()
        
        # Draw edges if enabled
        if self.show_edges and not self.wireframe_mode:
            glDisable(GL_LIGHTING)
            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE)
            glColor3f(0.3, 0.3, 0.3)
            glLineWidth(1.0)
            
            glBegin(GL_TRIANGLES)
            for face in self.faces:
                for vertex_idx in face:
                    vertex = self.vertices[vertex_idx]
                    glVertex3f(vertex[0], vertex[1], vertex[2])
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
        self.vertices = self.mesh.vertices
        self.faces = self.mesh.faces
        
        # Extract colors
        self.colors = self.get_face_colors()
        
        # Calculate face normals for lighting
        self.normals = self.mesh.face_normals
        
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
        
    def mouseMoveEvent(self, event):
        """Handle mouse drag for rotation"""
        if self.last_pos is None:
            return
        
        dx = event.position().x() - self.last_pos.x()
        dy = event.position().y() - self.last_pos.y()
        
        if event.buttons() & Qt.MouseButton.LeftButton:
            self.target_rotation_x += dy * self.mouse_sensitivity
            self.target_rotation_y += dx * self.mouse_sensitivity
            self.velocity_x = dy * self.mouse_sensitivity * 0.3
            self.velocity_y = dx * self.mouse_sensitivity * 0.3
            # self.rotation_x += (self.target_rotation_x - self.rotation_x) * 0.2
            # self.rotation_y += (self.target_rotation_y - self.rotation_y) * 0.2


        elif event.buttons() & Qt.MouseButton.RightButton:
            # self.zoom += dy * 0.01
            self.target_zoom += dy * 0.01
        
        self.last_pos = event.position()
        
    def update_rotation(self):
        self.target_zoom = max(-20.0, min(-1.0, self.target_zoom))
        # Smoothly move rotation toward target
        self.rotation_x += (self.target_rotation_x - self.rotation_x) * self.smoothing_factor
        self.rotation_y += (self.target_rotation_y - self.rotation_y) * self.smoothing_factor

        # Apply momentum
        self.rotation_x += self.velocity_x
        self.rotation_y += self.velocity_y
        self.velocity_x *= self.momentum_decay
        self.velocity_y *= self.momentum_decay

        # Smooth zoom
        self.zoom += (self.target_zoom - self.zoom) * self.smoothing_factor

        self.update()

    def wheelEvent(self, event):
        """Handle mouse wheel for zoom"""
        delta = event.angleDelta().y() / 120 
        self.target_zoom += delta * 0.15   
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