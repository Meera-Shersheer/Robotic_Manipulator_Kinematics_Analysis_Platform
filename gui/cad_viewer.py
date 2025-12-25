from imports import *
import base64

HTML_TEMPLATE = """
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <title>3D Model Viewer</title>
    <style>
        body {{ 
            margin: 0; 
            overflow: hidden;
        }}
        #loading {{
            position: absolute;
            top: 50%;
            left: 50%;
            transform: translate(-50%, -50%);
            color: white;
            font-family: Arial;
            font-size: 18px;
        }}
    </style>
</head>
<body>
<div id="loading">Loading model...</div>

<script type="importmap">
{{
  "imports": {{
    "three": "https://cdn.jsdelivr.net/npm/three@0.160.0/build/three.module.js",
    "three/addons/": "https://cdn.jsdelivr.net/npm/three@0.160.0/examples/jsm/"
  }}
}}
</script>

<script type="module">
import * as THREE from 'three';
import {{ OrbitControls }} from 'three/addons/controls/OrbitControls.js';
import {{ OBJLoader }} from 'three/addons/loaders/OBJLoader.js';
import {{ MTLLoader }} from 'three/addons/loaders/MTLLoader.js';

const loadingDiv = document.getElementById('loading');

// Scene setup
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x2a2a2a);

// Camera
const camera = new THREE.PerspectiveCamera(
    45, 
    window.innerWidth / window.innerHeight, 
    0.1, 
    1000
);
camera.position.set(3, 3, 5);

// Renderer
const renderer = new THREE.WebGLRenderer({{ antialias: true }});
renderer.setSize(window.innerWidth, window.innerHeight);
renderer.setPixelRatio(window.devicePixelRatio);
renderer.shadowMap.enabled = true;
document.body.appendChild(renderer.domElement);

// Orbit Controls
const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.dampingFactor = 0.05;
controls.minDistance = 1;
controls.maxDistance = 50;

// Lighting
const ambientLight = new THREE.AmbientLight(0x404040, 2);
scene.add(ambientLight);

const directionalLight1 = new THREE.DirectionalLight(0xffffff, 1.5);
directionalLight1.position.set(5, 5, 5);
directionalLight1.castShadow = true;
scene.add(directionalLight1);

const directionalLight2 = new THREE.DirectionalLight(0xffffff, 0.8);
directionalLight2.position.set(-5, 3, -5);
scene.add(directionalLight2);

const hemisphereLight = new THREE.HemisphereLight(0xffffff, 0x444444, 0.6);
scene.add(hemisphereLight);

// Grid helper
const gridHelper = new THREE.GridHelper(10, 10);
scene.add(gridHelper);

// Load model
const mtlLoader = new MTLLoader();
mtlLoader.setPath('');

mtlLoader.load(
    '{mtl_path}',
    function(materials) {{
        materials.preload();
        
        const objLoader = new OBJLoader();
        objLoader.setMaterials(materials);
        objLoader.setPath('');
        
        objLoader.load(
            '{obj_path}',
            function(object) {{
                // Center the model
                const box = new THREE.Box3().setFromObject(object);
                const center = box.getCenter(new THREE.Vector3());
                object.position.sub(center);
                
                // Scale to fit view
                const size = box.getSize(new THREE.Vector3());
                const maxDim = Math.max(size.x, size.y, size.z);
                const scale = 2 / maxDim;
                object.scale.multiplyScalar(scale);
                
                scene.add(object);
                loadingDiv.style.display = 'none';
                console.log('Model loaded successfully');
            }},
            function(xhr) {{
                const percent = (xhr.loaded / xhr.total * 100).toFixed(0);
                loadingDiv.textContent = `Loading model... ${{percent}}%`;
            }},
            function(error) {{
                console.error('Error loading OBJ:', error);
                loadingDiv.textContent = 'Error loading model. Check console.';
                loadingDiv.style.color = 'red';
            }}
        );
    }},
    function(xhr) {{
        console.log((xhr.loaded / xhr.total * 100) + '% loaded');
    }},
    function(error) {{
        console.error('Error loading MTL:', error);
        loadingDiv.textContent = 'Error loading materials. Check console.';
        loadingDiv.style.color = 'red';
    }}
);

// Animation loop
function animate() {{
    requestAnimationFrame(animate);
    controls.update();
    renderer.render(scene, camera);
}}
animate();

// Handle window resize
window.addEventListener('resize', () => {{
    camera.aspect = window.innerWidth / window.innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(window.innerWidth, window.innerHeight);
}});

</script>
</body>
</html>
"""

class CADViewer(QWidget):
    def __init__(self, obj_path, mtl_path=None, parent=None):
        super().__init__(parent)
        self.layout = QVBoxLayout(self)
        self.web = QWebEngineView()
        self.layout.addWidget(self.web)

        # Fill in HTML template
        html = HTML_TEMPLATE.format(
            obj_path=os.path.abspath(obj_path).replace("\\", "/"),
            mtl_path=os.path.abspath(mtl_path).replace("\\", "/") if mtl_path else ""
        )

        # Save temp HTML
        import tempfile
        temp_file = tempfile.NamedTemporaryFile(delete=False, suffix=".html")
        temp_file.write(html.encode("utf-8"))
        temp_file.close()
        self._temp_file_path = temp_file.name

        # Load in QWebEngineView
        self.web.load(QUrl.fromLocalFile(self._temp_file_path))

    def closeEvent(self, event):
        if hasattr(self, "_temp_file_path") and os.path.exists(self._temp_file_path):
            os.remove(self._temp_file_path)
        super().closeEvent(event)

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("CAD Viewer - Three.js")
        self.resize(1024, 768)

        widget = QWidget()
        layout = QVBoxLayout(widget)

        # Your OBJ/MTL files
        self.viewer = CADViewer(
            "/home/mshershe/robotics/Robotics_project/gui/ur5.obj",
            "/home/mshershe/robotics/Robotics_project/gui/ur5.mtl"
        )
        layout.addWidget(self.viewer)
        self.setCentralWidget(widget)

if __name__ == "__main__":

    os.environ["QTWEBENGINE_CHROMIUM_FLAGS"] = "--disable-gpu --disable-software-rasterizer"

    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())

