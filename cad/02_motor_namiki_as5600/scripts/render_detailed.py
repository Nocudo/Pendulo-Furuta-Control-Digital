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
    
    # Camera
    cam = coin.SoPerspectiveCamera()
    rot1 = coin.SbRotation(coin.SbVec3f(1, 0, 0), cam_orientation[0])
    rot2 = coin.SbRotation(coin.SbVec3f(0, 0, 1), cam_orientation[1])
    cam.orientation.setValue(rot1 * rot2)
    root.addChild(cam)
    
    # Lighting
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

doc_path = os.path.join(MODEL_DIR, "Motor_22CL_AS5600.FCStd")
doc = App.openDocument(doc_path)

gearbox = doc.getObject("Gearbox_3501PG").Shape
boss = doc.getObject("Centering_Boss").Shape
shaft = doc.getObject("Output_Shaft_D4mm").Shape
housing = doc.getObject("Motor_22CL_Housing").Shape
rear_shaft = doc.getObject("Rear_Shaft_D2mm").Shape
adapter = doc.getObject("Acoplador_Motor_AS5600").Shape
holder = doc.getObject("Portaiman_AS5600").Shape
magnet = doc.getObject("Iman_Diametral_AS5600").Shape
sensor = doc.getObject("Encoder_AS5600").Shape
clamp_screw = get_optional_shape(doc, "Tornillo_M3_Abrazadera")
clamp_nut = get_optional_shape(doc, "Tuerca_M3_Abrazadera")
pcb_screws = doc.getObject("Tornillos_Fijacion_PCB_AS5600").Shape

# Materials
mat_motor_housing = {"diffuse": (0.28, 0.30, 0.34), "specular": (0.6, 0.6, 0.7), "shininess": 0.8}
mat_gearbox = {"diffuse": (0.75, 0.76, 0.78), "specular": (0.9, 0.9, 0.95), "shininess": 0.9}
mat_steel_shaft = {"diffuse": (0.85, 0.88, 0.92), "specular": (1.0, 1.0, 1.0), "shininess": 0.95}
mat_printed_adapter = {"diffuse": (0.12, 0.55, 0.90), "specular": (0.4, 0.6, 0.8), "shininess": 0.5}
mat_printed_holder = {"diffuse": (0.95, 0.55, 0.15), "specular": (0.5, 0.4, 0.2), "shininess": 0.6}
mat_magnet = {"diffuse": (0.85, 0.15, 0.15), "specular": (0.8, 0.8, 0.9), "shininess": 0.85}
mat_pcb_sensor = {"diffuse": (0.15, 0.65, 0.35), "specular": (0.3, 0.5, 0.3), "shininess": 0.4}
mat_screws = {"diffuse": (0.20, 0.22, 0.25), "specular": (0.7, 0.7, 0.8), "shininess": 0.85}

# 1. Vista Inferior (Bottom view showing PCB, 4 screws, pin headers)
bottom_shapes = [
    (adapter, mat_printed_adapter),
    (sensor, mat_pcb_sensor),
    (pcb_screws, mat_screws),
    (housing, mat_motor_housing),
]
if clamp_screw is not None:
    bottom_shapes.append((clamp_screw, mat_screws))

out_bot = os.path.join(RENDERS_DIR, "render_vista_inferior_pcb.png")
render_scene(bottom_shapes, out_bot, cam_orientation=(2.3, 0.8), cam_zoom=1.2)

# 2. Portaiman Closeup
holder_shapes = [
    (holder, mat_printed_holder),
    (magnet, mat_magnet)
]
out_hol = os.path.join(RENDERS_DIR, "render_portaiman.png")
render_scene(holder_shapes, out_hol, cam_orientation=(1.1, 0.7), cam_zoom=1.3)

# 3. Exploded View (Vista en despiece / explosión)
adapter_exp = adapter.copy()
adapter_exp.translate(App.Vector(0, 0, -25))

holder_exp = holder.copy()
holder_exp.translate(App.Vector(0, 0, -50))

magnet_exp = magnet.copy()
magnet_exp.translate(App.Vector(0, 0, -50))

sensor_exp = sensor.copy()
sensor_exp.translate(App.Vector(0, 0, -75))

pcb_screws_exp = pcb_screws.copy()
pcb_screws_exp.translate(App.Vector(0, 0, -95))

exploded_shapes = [
    (gearbox, mat_gearbox),
    (boss, mat_gearbox),
    (shaft, mat_steel_shaft),
    (housing, mat_motor_housing),
    (rear_shaft, mat_steel_shaft),
    (adapter_exp, mat_printed_adapter),
    (holder_exp, mat_printed_holder),
    (magnet_exp, mat_magnet),
    (sensor_exp, mat_pcb_sensor),
    (pcb_screws_exp, mat_screws)
]
if clamp_screw is not None:
    cs_e = clamp_screw.copy()
    cs_e.translate(App.Vector(-20, 0, -25))
    exploded_shapes.append((cs_e, mat_screws))
if clamp_nut is not None:
    cn_e = clamp_nut.copy()
    cn_e.translate(App.Vector(20, 0, -25))
    exploded_shapes.append((cn_e, mat_screws))

out_exp = os.path.join(RENDERS_DIR, "render_despiece_explosivo.png")
render_scene(exploded_shapes, out_exp, cam_orientation=(1.25, 0.65), cam_zoom=0.85)

# 4. Detailed Cross Section (looking directly into cutaway)
cut_box = Part.makeBox(100.0, 100.0, 150.0, App.Vector(-50.0, -100.0, -100.0))
internal_shapes = [
    (housing.cut(cut_box), mat_motor_housing),
    (rear_shaft.cut(cut_box), mat_steel_shaft),
    (adapter.cut(cut_box), mat_printed_adapter),
    (holder.cut(cut_box), mat_printed_holder),
    (magnet.cut(cut_box), mat_magnet),
    (sensor.cut(cut_box), mat_pcb_sensor)
]
out_sec = os.path.join(RENDERS_DIR, "render_corte_interno.png")
render_scene(internal_shapes, out_sec, cam_orientation=(1.35, 0.25), cam_zoom=1.4)

print("Additional renders complete!")
