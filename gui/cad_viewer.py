from imports import *

class CADViewer(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        
        self.layout = QVBoxLayout(self)
        self.layout.setContentsMargins(0, 0, 0, 0)
        
        self.vtk_widget = None
        self.renderer = None
        self.interactor = None
        self.initialized = False
        
        self.setSizePolicy(
            QSizePolicy.Policy.Expanding,
            QSizePolicy.Policy.Expanding
        )



    def _setup_camera(self):
        camera = self.renderer.GetActiveCamera()
        camera.SetPosition(1.5, 1.5, 1.5)
        camera.SetFocalPoint(0, 0, 0)
        camera.SetViewUp(0, 0, 1)
        
    def load_glb(self, filepath):
        if not os.path.exists(filepath):
            print(f"Error: File not found: {filepath}")
            return []
        try:
            reader = vtk.vtkGLTFReader()
            reader.SetFileName(filepath)
            reader.Update()

            actors = []

            for i in range(reader.GetNumberOfOutputPorts()):
                mapper = vtk.vtkPolyDataMapper()
                mapper.SetInputConnection(reader.GetOutputPort(i))
                actor = vtk.vtkActor()
                actor.SetMapper(mapper)

                self.renderer.AddActor(actor)
                actors.append(actor)

            self.renderer.ResetCamera()
            self.vtk_widget.GetRenderWindow().Render()

            return actors
        
        except Exception as e:
            print(f"Error loading GLB file: {e}")
            return []
    
    #Initialize VTK when widget is first shown
    def showEvent(self, event):
        super().showEvent(event)
        if not self.initialized:
            self._initialize_vtk()
            self.initialized = True
            
    """Clean up VTK resources"""
    
    def closeEvent(self, event):
        if hasattr(self, 'vtk_widget'):
            self.vtk_widget.Finalize()
        super().closeEvent(event)
        
    def _initialize_vtk(self):
        """Lazy initialization of VTK components"""
        self.vtk_widget = QVTKRenderWindowInteractor(self)
        self.layout.addWidget(self.vtk_widget)
        
        self.renderer = vtk.vtkRenderer()
        self.renderer.SetBackground(0.95, 0.95, 0.95)
        
        self.vtk_widget.GetRenderWindow().AddRenderer(self.renderer)
        self.interactor = self.vtk_widget.GetRenderWindow().GetInteractor()
        
        self._setup_camera()
        self.interactor.Initialize()
        
        style = vtk.vtkInteractorStyleTrackballCamera()
        self.interactor.SetInteractorStyle(style)