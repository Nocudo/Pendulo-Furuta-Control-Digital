import os
import sys
import math
import FreeCAD as App
import Part
import pivy.coin as coin
from PIL import Image

sys.stdout.reconfigure(encoding='utf-8')

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

def render_scene(shapes_and_materials, output_path, cam_orientation=(1.1, 0.7), cam_zoom=1.0, bg_color=(0.10, 0.13, 0.18), width=1600, height=1200):
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
    light2.intensity.setValue(0.65)
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

# Materials Palette (Colores de alto contraste para identificar inequívocamente cada pieza)
mat_tower = {"diffuse": (0.16, 0.18, 0.22), "specular": (0.3, 0.35, 0.4), "shininess": 0.3}          # Dark Charcoal Matte (Torre)
mat_cap = {"diffuse": (0.28, 0.42, 0.65), "specular": (0.4, 0.55, 0.8), "shininess": 0.5}           # Steel Blue (Tapa Motor)
mat_arm = {"diffuse": (0.00, 0.68, 0.88), "specular": (0.4, 0.8, 0.95), "shininess": 0.7}            # Cyan / Electric Blue (Brazo)
mat_axle = {"diffuse": (0.92, 0.94, 0.97), "specular": (1.0, 1.0, 1.0), "shininess": 0.98}           # Chrome Steel (Eje)
mat_pendulum = {"diffuse": (1.00, 0.35, 0.05), "specular": (0.8, 0.5, 0.3), "shininess": 0.6}       # Fiery Safety Orange (Péndulo)
mat_mass = {"diffuse": (0.65, 0.12, 0.78), "specular": (0.85, 0.4, 0.9), "shininess": 0.85}          # Anodized Purple (Masa Ajustable)
mat_bearing = {"diffuse": (0.50, 0.53, 0.58), "specular": (0.85, 0.9, 0.95), "shininess": 0.9}        # Titanium / Bearing Steel (Rodamiento)
mat_pcb = {"diffuse": (0.05, 0.78, 0.30), "specular": (0.4, 0.8, 0.4), "shininess": 0.5}             # Emerald Green FR4 (Sensor Péndulo)
mat_magnet = {"diffuse": (0.95, 0.05, 0.15), "specular": (0.9, 0.5, 0.5), "shininess": 0.85}         # Ruby Scarlet Red (Imanes)
mat_screws = {"diffuse": (0.24, 0.24, 0.26), "specular": (0.6, 0.6, 0.7), "shininess": 0.8}          # Gunmetal Black (Tornillería)
mat_motor_housing = {"diffuse": (0.10, 0.10, 0.12), "specular": (0.4, 0.4, 0.4), "shininess": 0.5}   # Deep Satin Black (Carcasa Motor)
mat_gearbox = {"diffuse": (0.82, 0.82, 0.80), "specular": (0.95, 0.95, 0.9), "shininess": 0.92}     # Champagne Aluminum (Reductor)
mat_motor_coupler = {"diffuse": (0.88, 0.92, 0.10), "specular": (0.8, 0.9, 0.3), "shininess": 0.7}   # Acid Lime Yellow (Acoplador Trasero)
mat_motor_mag_holder = {"diffuse": (0.95, 0.25, 0.55), "specular": (0.9, 0.5, 0.7), "shininess": 0.75} # Coral Pink (Portaimán Trasero)
mat_motor_pcb = {"diffuse": (0.08, 0.25, 0.75), "specular": (0.3, 0.5, 0.8), "shininess": 0.5}      # Royal Navy Blue FR4 (Sensor Motor)

tower = doc.getObject("Base_Torre_Furuta").Shape
cap = get_optional_shape(doc, "Tapa_Motor_Torre_Furuta")
arm = doc.getObject("Brazo_Horizontal_L88").Shape
axle = doc.getObject("Eje_Portaiman_Pendulo").Shape
pendulum = doc.getObject("Barra_Pendulo_Furuta").Shape
tip_mass = doc.getObject("Masa_Ajustable_Pendulo").Shape
bearing = doc.getObject("Rodamiento_608_Pendulo").Shape
pen_sensor = doc.getObject("Encoder_Pendulo_AS5600").Shape
pen_mag = doc.getObject("Iman_Diametral_Pendulo").Shape

motor_objs = [
    (doc.getObject("Motor_Gearbox_3501PG").Shape, mat_gearbox),
    (doc.getObject("Motor_Centering_Boss").Shape, mat_gearbox),
    (doc.getObject("Motor_Output_Shaft_D4mm").Shape, mat_axle),
    (doc.getObject("Motor_Motor_22CL_Housing").Shape, mat_motor_housing),
    (doc.getObject("Motor_Acoplador_Motor_AS5600").Shape, mat_motor_coupler),
    (doc.getObject("Motor_Portaiman_AS5600").Shape, mat_motor_mag_holder),
    (doc.getObject("Motor_Iman_Diametral_AS5600").Shape, mat_magnet),
    (doc.getObject("Motor_Encoder_AS5600").Shape, mat_motor_pcb)
]

m2_screws = get_optional_shape(doc, "Tornillos_Motor_M2_Avellanados", "Tornillos_Motor_M2")
cap_screws = get_optional_shape(doc, "Tornillos_Tapa_M3_Avellanados")
fasteners = []
for s in [
    m2_screws,
    cap_screws,
    get_optional_shape(doc, "Tornillo_Masa_M3"),
    get_optional_shape(doc, "Tornillos_PCB_Pendulo_M3"),
]:
    if s is not None:
        fasteners.append((s, mat_screws))

full_scene = [
    (tower, mat_tower),
] + ([(cap, mat_cap)] if cap is not None else []) + [
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
render_scene(full_scene, out1, cam_orientation=(1.20, 0.75), cam_zoom=0.88)

# 2. Side View showing 3cm clearance from pendulum tip to tabletop
out2 = os.path.join(RENDERS_DIR, "render_vista_lateral.png")
render_scene(full_scene, out2, cam_orientation=(0.15, -1.57), cam_zoom=0.88)

# 3. Close-up of Form-Fitting Motor Hub, Detachable Cap & D-Shaft Coupling
hub_view = [
    (doc.getObject("Motor_Gearbox_3501PG").Shape, mat_gearbox),
    (doc.getObject("Motor_Centering_Boss").Shape, mat_gearbox),
    (doc.getObject("Motor_Output_Shaft_D4mm").Shape, mat_axle),
    (arm, mat_arm),
    (tower, mat_tower),
]
if cap is not None:
    hub_view.append((cap, mat_cap))
if m2_screws is not None:
    hub_view.append((m2_screws, mat_screws))
if cap_screws is not None:
    hub_view.append((cap_screws, mat_screws))

out_hub = os.path.join(RENDERS_DIR, "render_detalle_acople_motor.png")
render_scene(hub_view, out_hub, cam_orientation=(1.05, 0.45), cam_zoom=2.0)

# 4. Close-up of Revolute Joint with Form-Fitting Pendulum Hub (NO CLAMPS)
joint_scene = [
    (arm, mat_arm),
    (axle, mat_axle),
    (pendulum, mat_pendulum),
    (bearing, mat_bearing),
    (pen_sensor, mat_pcb),
    (pen_mag, mat_magnet),
    (doc.getObject("Tornillos_PCB_Pendulo_M3").Shape, mat_screws)
]
out3 = os.path.join(RENDERS_DIR, "render_detalle_articulacion.png")
render_scene(joint_scene, out3, cam_orientation=(1.15, 0.45), cam_zoom=1.4)

# 5. Exploded Assembly View (Vista en Explosion)
arm_exp = arm.copy()
arm_exp.translate(App.Vector(0, 0, 28))

pen_exp = pendulum.copy()
pen_exp.translate(App.Vector(35, 0, 28))

mass_exp = tip_mass.copy()
mass_exp.translate(App.Vector(35, 0, 28))

axle_exp = axle.copy()
axle_exp.translate(App.Vector(-20, 0, 28))

bearing_exp = bearing.copy()
bearing_exp.translate(App.Vector(15, 0, 28))

sensor_exp = pen_sensor.copy()
sensor_exp.translate(App.Vector(-45, 0, 28))

mag_exp = pen_mag.copy()
mag_exp.translate(App.Vector(-20, 0, 28))

exploded_scene = [
    (tower, mat_tower),
] + ([((cap_exp := cap.copy()), mat_cap)] if cap is not None else []) + [
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
render_scene(exploded_scene, out4, cam_orientation=(1.22, 0.70), cam_zoom=0.78)

# 6. Cutaway Section through Head (Y >= 0 cut away)
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
render_scene(section_shapes, out_sec, cam_orientation=(1.25, 0.15), cam_zoom=1.0)

# 7. Cutaway Section of Base Tower, Detachable Cap & Motor Housing (X >= 0 cut away)
cut_tower_box = Part.makeBox(120.0, 140.0, 185.0, App.Vector(0.0, -70.0, -5.0))
tower_section = [
    (tower.cut(cut_tower_box), mat_tower),
] + ([(cap.cut(cut_tower_box), mat_cap)] if cap is not None else []) + [
    (doc.getObject("Motor_Gearbox_3501PG").Shape.cut(cut_tower_box), mat_gearbox),
    (doc.getObject("Motor_Centering_Boss").Shape.cut(cut_tower_box), mat_gearbox),
    (doc.getObject("Motor_Output_Shaft_D4mm").Shape.cut(cut_tower_box), mat_axle),
    (doc.getObject("Motor_Motor_22CL_Housing").Shape.cut(cut_tower_box), mat_motor_housing),
    (doc.getObject("Motor_Acoplador_Motor_AS5600").Shape.cut(cut_tower_box), mat_motor_coupler),
    (doc.getObject("Motor_Portaiman_AS5600").Shape.cut(cut_tower_box), mat_motor_mag_holder),
    (doc.getObject("Motor_Iman_Diametral_AS5600").Shape.cut(cut_tower_box), mat_magnet),
    (doc.getObject("Motor_Encoder_AS5600").Shape.cut(cut_tower_box), mat_motor_pcb),
    (arm.cut(cut_tower_box), mat_arm)
]
out_tower_cut = os.path.join(RENDERS_DIR, "render_corte_torre_motor.png")
render_scene(tower_section, out_tower_cut, cam_orientation=(1.10, 0.60), cam_zoom=0.95)

print("All Coin3D renders completed successfully!")
