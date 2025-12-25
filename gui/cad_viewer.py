from imports import *

class CADViewer(QWidget):
    def __init__(self):
        super().__init__()

        self.view = Qt3DWindow()
        self.container = QWidget.createWindowContainer(self.view)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self.container)

        root = QEntity()
        self.view.setRootEntity(root)

        # Camera
        camera = self.view.camera()
        camera.setPosition(QVector3D(5, 5, 5))
        camera.setViewCenter(QVector3D(0, 0, 0))

        cam_controller = QOrbitCameraController(root)
        cam_controller.setCamera(camera)

        # Dummy mesh (replace with CAD later)
        mesh_entity = QEntity(root)
        mesh = QCuboidMesh()
        material = QPhongMaterial()
        mesh_entity.addComponent(mesh)
        mesh_entity.addComponent(material)