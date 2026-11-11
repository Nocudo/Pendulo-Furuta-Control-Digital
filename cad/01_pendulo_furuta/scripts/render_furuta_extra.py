import os
import math
import FreeCAD as App
import Part
import pivy.coin as coin
from PIL import Image

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
MODEL_DIR = os.path.dirname(SCRIPT_DIR)
RENDERS_DIR = os.path.join(MODEL_DIR, "renders")
os.makedirs(RENDERS_DIR, exist_ok=True)


def get_optional_shape(doc, *names):
    for name in names:
        obj = doc.getObject(name)
        if obj is not None and not obj.Shape.isNull():
            return obj.Shape
    return None

def render_scene(shapes_and_materials, output_path, cam_orientation=(1.1, 0.7), cam_zoom=1.0, bg_color=(0.12, 0.15, 0.20), width=1600, height=1200):
    root = coin.SoSeparator()
    cam = coin.SoPerspectiveCamera()
    rot1 = coin.SbRotation(coin.SbVec3f(1, 0, 0), cam_orientation[0])
    rot2 = coin.SbRotation(coin.SbVec3f(0, 0, 1), cam_orientation[1])
    cam.orientation.setValue(rot1 * rot2)
    root.addChild(cam)
    
    light1 = coin.SoDirectionalLight()
    light1.direction.setValue(1.0, -1.0, -1.5)
    light1.intensity.setValue(1.0)
    root.addChild(light1)
    
    light2 = coin.SoDirectionalLight()
    light2.direction.setValue(-1.2, 1.0, -0.8)
    light2.intensity.setValue(0.6)
    root.addChild(light2)

    light3 = coin.SoDirectionalLight()
    light3.direction.setValue(0.0, 0.0, 1.0)
    light3.intensity.setValue(0.4)
    root.addChild(light3)

    for shape, mat_props in shapes_and_materials:
        sep = coin.SoSeparator()
        mat = coin.SoMaterial()
        if "diffuse" in mat_props:
            mat.diffuseColor.setValue(*mat_props["diffuse"])
        if "specular" in mat_props:
            mat.specularColor.setValue(*mat_props["specular"])
        if "shininess" in mat_props:
            mat.shininess.setValue(mat_props["shininess"])
        sep.addChild(mat)
        
        iv_str = shape.writeInventor(2, 0.035)
        inp = coin.SoInput()
        inp.setBuffer(iv_str)
        scene = coin.SoDB.readAll(inp)
        if scene:
            sep.addChild(scene)
        root.addChild(sep)
        
    vp = coin.SbViewportRegion(width, height)
    cam.viewAll(root, vp)
    if cam_zoom != 1.0:
        cam.heightAngle.setValue(cam.heightAngle.getValue() / cam_zoom)
        
    renderer = coin.SoOffscreenRenderer(vp)
    renderer.setBackgroundColor(coin.SbColor(*bg_color))
    renderer.render(root)
    
    buf = renderer.getBuffer()
    img = Image.frombytes("RGB", (width, height), bytes(buf))
    img = img.transpose(Image.FLIP_TOP_BOTTOM)
    img.save(output_path)
    print(f"Render saved to: {output_path}")

doc_path = os.path.join(MODEL_DIR, "Pendulo_Furuta_Completo.FCStd")
doc = App.openDocument(doc_path)

mat_arm = {"diffuse": (0.12, 0.55, 0.90), "specular": (0.4, 0.6, 0.8), "shininess": 0.6}
mat_axle = {"diffuse": (0.85, 0.88, 0.92), "specular": (1.0, 1.0, 1.0), "shininess": 0.95}
mat_pendulum = {"diffuse": (0.95, 0.55, 0.15), "specular": (0.6, 0.5, 0.3), "shininess": 0.6}
mat_bearing = {"diffuse": (0.75, 0.75, 0.78), "specular": (0.9, 0.9, 0.95), "shininess": 0.9}
mat_pcb = {"diffuse": (0.15, 0.65, 0.35), "specular": (0.3, 0.5, 0.3), "shininess": 0.4}
mat_magnet = {"diffuse": (0.85, 0.15, 0.15), "specular": (0.8, 0.8, 0.9), "shininess": 0.85}
mat_screws = {"diffuse": (0.20, 0.22, 0.25), "specular": (0.7, 0.7, 0.8), "shininess": 0.85}

arm = doc.getObject("Brazo_Horizontal_L88").Shape
axle = doc.getObject("Eje_Portaiman_Pendulo").Shape
pendulum = doc.getObject("Barra_Pendulo_Furuta").Shape
bearing = doc.getObject("Rodamiento_608_Pendulo").Shape
pen_sensor = doc.getObject("Encoder_Pendulo_AS5600").Shape
pen_mag = doc.getObject("Iman_Diametral_Pendulo").Shape
pen_screw = get_optional_shape(doc, "Tornillo_Cubo_Pendulo_M3")
pcb_screws = doc.getObject("Tornillos_PCB_Pendulo_M3").Shape

# View of the head showing the sensor side (looking from -X towards the head)
joint_sensor_view = [
    (arm, mat_arm),
    (axle, mat_axle),
    (pendulum, mat_pendulum),
    (bearing, mat_bearing),
    (pen_sensor, mat_pcb),
    (pen_mag, mat_magnet),
    (pcb_screws, mat_screws)
]
if pen_screw is not None:
    joint_sensor_view.append((pen_screw, mat_screws))

out_head = os.path.join(RENDERS_DIR, "render_detalle_sensor_pendulo.png")
render_scene(joint_sensor_view, out_head, cam_orientation=(1.35, -1.8), cam_zoom=1.0)

# Section cutaway of head with Y > 0 removed
cut_box = Part.makeBox(120.0, 60.0, 100.0, App.Vector(50.0, 0.0, 114.0))
section_shapes = [
    (arm.cut(cut_box), mat_arm),
    (axle.cut(cut_box), mat_axle),
    (pendulum.cut(cut_box), mat_pendulum),
    (bearing.cut(cut_box), mat_bearing),
    (pen_sensor, mat_pcb),
    (pen_mag.cut(cut_box), mat_magnet)
]
out_sec = os.path.join(RENDERS_DIR, "render_corte_articulacion.png")
render_scene(section_shapes, out_sec, cam_orientation=(1.25, 0.15), cam_zoom=0.95)

print("Re-renders complete!")
