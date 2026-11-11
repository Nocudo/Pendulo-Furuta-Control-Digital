import os
import sys
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

doc_path = os.path.join(MODEL_DIR, "Pendulo_Furuta_Completo.FCStd")
if not os.path.exists(doc_path):
    print(f"Waiting for file: {doc_path}")
    sys.exit(0)

doc = App.openDocument(doc_path)

# Materials
mat_tower = {"diffuse": (0.15, 0.20, 0.28), "specular": (0.4, 0.5, 0.6), "shininess": 0.4}       # Dark Blue / Slate Grey
mat_arm = {"diffuse": (0.12, 0.55, 0.90), "specular": (0.4, 0.6, 0.8), "shininess": 0.6}         # Cyan / Tech Blue
mat_axle = {"diffuse": (0.85, 0.88, 0.92), "specular": (1.0, 1.0, 1.0), "shininess": 0.95}       # Silver
mat_pendulum = {"diffuse": (0.95, 0.55, 0.15), "specular": (0.6, 0.5, 0.3), "shininess": 0.6}    # High-vis Orange
mat_mass = {"diffuse": (0.85, 0.70, 0.20), "specular": (0.9, 0.8, 0.4), "shininess": 0.8}        # Brass / Gold
mat_bearing = {"diffuse": (0.75, 0.75, 0.78), "specular": (0.9, 0.9, 0.95), "shininess": 0.9}     # Bearing steel
mat_pcb = {"diffuse": (0.15, 0.65, 0.35), "specular": (0.3, 0.5, 0.3), "shininess": 0.4}         # FR4 Green
mat_magnet = {"diffuse": (0.85, 0.15, 0.15), "specular": (0.8, 0.8, 0.9), "shininess": 0.85}     # Red magnet
mat_screws = {"diffuse": (0.20, 0.22, 0.25), "specular": (0.7, 0.7, 0.8), "shininess": 0.85}     # Black oxide
mat_motor_housing = {"diffuse": (0.28, 0.30, 0.34), "specular": (0.6, 0.6, 0.7), "shininess": 0.8}
mat_gearbox = {"diffuse": (0.75, 0.76, 0.78), "specular": (0.9, 0.9, 0.95), "shininess": 0.9}

tower = doc.getObject("Base_Torre_Furuta").Shape
cap = get_optional_shape(doc, "Tapa_Motor_Torre_Furuta")
arm = doc.getObject("Brazo_Horizontal_L88").Shape
axle = doc.getObject("Eje_Portaiman_Pendulo").Shape
pendulum = doc.getObject("Barra_Pendulo_Furuta").Shape
tip_mass = doc.getObject("Masa_Ajustable_Pendulo").Shape
bearing = doc.getObject("Rodamiento_608_Pendulo").Shape
pen_sensor = doc.getObject("Encoder_Pendulo_AS5600").Shape
pen_mag = doc.getObject("Iman_Diametral_Pendulo").Shape

# Motor shapes
motor_objs = [
    (doc.getObject("Motor_Gearbox_3501PG").Shape, mat_gearbox),
    (doc.getObject("Motor_Centering_Boss").Shape, mat_gearbox),
    (doc.getObject("Motor_Output_Shaft_D4mm").Shape, mat_axle),
    (doc.getObject("Motor_Motor_22CL_Housing").Shape, mat_motor_housing),
    (doc.getObject("Motor_Acoplador_Motor_AS5600").Shape, mat_arm),
    (doc.getObject("Motor_Portaiman_AS5600").Shape, mat_pendulum),
    (doc.getObject("Motor_Iman_Diametral_AS5600").Shape, mat_magnet),
    (doc.getObject("Motor_Encoder_AS5600").Shape, mat_pcb)
]

# Fasteners
fastener_candidates = [
    ("Tornillos_Motor_M2_Avellanados", "Tornillos_Motor_M2"),
    ("Tornillos_Tapa_M3_Avellanados",),
    ("Tornillo_Cubo_Brazo_M3",),
    ("Tornillo_Cubo_Pendulo_M3",),
    ("Tornillo_Masa_M3",),
    ("Tornillos_PCB_Pendulo_M3",),
]
fasteners = []
for cands in fastener_candidates:
    shp = get_optional_shape(doc, *cands)
    if shp is not None:
        fasteners.append((shp, mat_screws))

full_scene = [
    (tower, mat_tower),
] + ([(cap, mat_arm)] if cap is not None else []) + [
    (arm, mat_arm),
    (axle, mat_axle),
    (pendulum, mat_pendulum),
    (tip_mass, mat_mass),
    (bearing, mat_bearing),
    (pen_sensor, mat_pcb),
    (pen_mag, mat_magnet)
] + motor_objs + fasteners

# 1. Full Furuta Assembly - Isometric Overview
out1 = os.path.join(RENDERS_DIR, "render_pendulo_furuta_completo.png")
render_scene(full_scene, out1, cam_orientation=(1.20, 0.75), cam_zoom=0.95)

# 2. Side View showing full pendulum swing and tower
out2 = os.path.join(RENDERS_DIR, "render_vista_lateral.png")
render_scene(full_scene, out2, cam_orientation=(0.2, -1.55), cam_zoom=0.95)

# 3. Close-up of Revolute Joint, 608 Bearing, Magnet, and second AS5600
joint_scene = [
    (arm, mat_arm),
    (axle, mat_axle),
    (pendulum, mat_pendulum),
    (bearing, mat_bearing),
    (pen_sensor, mat_pcb),
    (pen_mag, mat_magnet),
]
for cands in [("Tornillo_Cubo_Pendulo_M3",), ("Tornillos_PCB_Pendulo_M3",)]:
    shp = get_optional_shape(doc, *cands)
    if shp is not None:
        joint_scene.append((shp, mat_screws))
out3 = os.path.join(RENDERS_DIR, "render_detalle_articulacion.png")
render_scene(joint_scene, out3, cam_orientation=(1.15, 0.45), cam_zoom=1.4)

# 4. Exploded Assembly View (Vista en Explosión)
arm_exp = arm.copy()
arm_exp.translate(App.Vector(0, 0, 30))

pen_exp = pendulum.copy()
pen_exp.translate(App.Vector(40, 0, 30))

mass_exp = tip_mass.copy()
mass_exp.translate(App.Vector(40, 0, 30))

axle_exp = axle.copy()
axle_exp.translate(App.Vector(-20, 0, 30))

bearing_exp = bearing.copy()
bearing_exp.translate(App.Vector(15, 0, 30))

sensor_exp = pen_sensor.copy()
sensor_exp.translate(App.Vector(-45, 0, 30))

mag_exp = pen_mag.copy()
mag_exp.translate(App.Vector(-20, 0, 30))

exploded_scene = [
    (tower, mat_tower),
] + ([((cap_exp := cap.copy()), mat_arm)] if cap is not None else []) + [
    (arm_exp, mat_arm),
    (axle_exp, mat_axle),
    (pen_exp, mat_pendulum),
    (mass_exp, mat_mass),
    (bearing_exp, mat_bearing),
    (sensor_exp, mat_pcb),
    (mag_exp, mat_magnet)
] + motor_objs
if cap is not None:
    cap_exp.translate(App.Vector(0, 0, 12))

out4 = os.path.join(RENDERS_DIR, "render_despiece_furuta.png")
render_scene(exploded_scene, out4, cam_orientation=(1.22, 0.70), cam_zoom=0.8)

print("All Furuta renders completed successfully!")
