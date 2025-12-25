from imports import *

class CADViewer(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        
        self.main_layout = QVBoxLayout(self)
        self.main_layout.setContentsMargins(0, 0, 0, 0)
        
        # VTK components - NOT initialized yet
        self.vtk_widget = None
        self.renderer = None
        self.interactor = None
        self.initialized = False
        
        # Show a simple placeholder
        self.placeholder = QLabel("3D Viewer\nClick 'Initialize 3D View' to start")
        self.placeholder.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.placeholder.setStyleSheet("""
            QLabel {
                background-color: #f5f5f5;
                color: #666;
                font-size: 14px;
                border: 2px dashed #ccc;
                border-radius: 5px;
            }
        """)
        self.main_layout.addWidget(self.placeholder)
        
        self.setSizePolicy(
            QSizePolicy.Policy.Expanding,
            QSizePolicy.Policy.Expanding
        )

    def _setup_camera(self):
        """Setup camera after renderer is created"""
        if self.renderer:
            camera = self.renderer.GetActiveCamera()
            camera.SetPosition(1.5, 1.5, 1.5)
            camera.SetFocalPoint(0, 0, 0)
            camera.SetViewUp(0, 0, 1)
    
    def initialize_vtk(self):
        """Manually initialize VTK when needed"""
        if self.initialized:
            print("VTK already initialized")
            return True
            
        try:
            print("Initializing VTK...")
            
            # Remove placeholder
            if self.placeholder:
                self.main_layout.removeWidget(self.placeholder)
                self.placeholder.deleteLater()
                self.placeholder = None
            
            # Create VTK widget
            self.vtk_widget = QVTKRenderWindowInteractor(self)
            self.main_layout.addWidget(self.vtk_widget)
            
            # Create renderer
            self.renderer = vtk.vtkRenderer()
            self.renderer.SetBackground(0.95, 0.95, 0.95)
            
            # Setup render window
            render_window = self.vtk_widget.GetRenderWindow()
            render_window.AddRenderer(self.renderer)
            
            # Get interactor
            self.interactor = render_window.GetInteractor()
            
            # Setup camera
            self._setup_camera()
            
            # Initialize - THIS is where the X11 window is created
            self.interactor.Initialize()
            
            # Set interaction style
            style = vtk.vtkInteractorStyleTrackballCamera()
            self.interactor.SetInteractorStyle(style)
            
            # Force a render
            render_window.Render()
            
            self.initialized = True
            print("VTK initialized successfully")
            return True
            
        except Exception as e:
            print(f"Error initializing VTK: {e}")
            import traceback
            traceback.print_exc()
            self.initialized = False
            return False
        
    def load_glb(self, filepath):
        """Load a GLB file"""
        # Must initialize first
        if not self.initialized:
            if not self.initialize_vtk():
                print("Failed to initialize VTK")
                return []
            
        if not self.renderer or not self.vtk_widget:
            print("Error: VTK components not ready")
            return []
            
        if not os.path.exists(filepath):
            print(f"Error: File not found: {filepath}")
            return []
            
        try:
            print(f"Loading GLB file: {filepath}")
            reader = vtk.vtkGLTFReader()
            reader.SetFileName(filepath)
            reader.Update()

            actors = []
            num_outputs = reader.GetNumberOfOutputPorts()
            print(f"GLB file has {num_outputs} output ports")

            for i in range(num_outputs):
                mapper = vtk.vtkPolyDataMapper()
                mapper.SetInputConnection(reader.GetOutputPort(i))
                actor = vtk.vtkActor()
                actor.SetMapper(mapper)
                self.renderer.AddActor(actor)
                actors.append(actor)

            self.renderer.ResetCamera()
            self.vtk_widget.GetRenderWindow().Render()
            
            print(f"Successfully loaded {len(actors)} actors")
            return actors
        
        except Exception as e:
            print(f"Error loading GLB file: {e}")
            import traceback
            traceback.print_exc()
            return []
    
    def render(self):
        """Manually trigger a render"""
        if self.vtk_widget and self.initialized:
            try:
                self.vtk_widget.GetRenderWindow().Render()
            except Exception as e:
                print(f"Error rendering: {e}")
    
    def closeEvent(self, event):
        """Clean up VTK resources"""
        if self.vtk_widget and self.initialized:
            try:
                self.vtk_widget.Finalize()
            except:
                pass
        super().closeEvent(event)