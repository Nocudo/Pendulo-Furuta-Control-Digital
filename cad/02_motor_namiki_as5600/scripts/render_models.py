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
    
    # Lights (Key + Fill + Backlight)
    light1 = coin.SoDirectionalLight()
    light1.direction.setValue(1.0, -1.0, -1.5)
    light1.intensity.setValue(1.0)
    root.addChild(light1)
    
    light2 = coin.SoDirectionalLight()
    light2.direction.setValue(-1.2, 1.0, -0.8)
    light2.intensity.setValue(0.55)
    root.addChild(light2)

    light3 = coin.SoDirectionalLight()
    light3.direction.setValue(0.0, 0.0, 1.0)
    light3.intensity.setValue(0.35)
    root.addChild(light3)

    for shape, mat_props in shapes_and_materials:
        sep = coin.SoSeparator()
        
        # Material
        mat = coin.SoMaterial()
        if "diffuse" in mat_props:
            mat.diffuseColor.setValue(*mat_props["diffuse"])
        if "specular" in mat_props:
            mat.specularColor.setValue(*mat_props["specular"])
        if "shininess" in mat_props:
            mat.shininess.setValue(mat_props["shininess"])
        if "transparency" in mat_props:
            mat.transparency.setValue(mat_props["transparency"])
        sep.addChild(mat)
        
        # Geometry
        iv_str = shape.writeInventor(2, 0.04)
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

# Load models from project
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

# Material palettes
mat_motor_housing = {"diffuse": (0.28, 0.30, 0.34), "specular": (0.6, 0.6, 0.7), "shininess": 0.8} # Dark brushed aluminum
mat_gearbox = {"diffuse": (0.75, 0.76, 0.78), "specular": (0.9, 0.9, 0.95), "shininess": 0.9}      # Nickel/steel gearbox
mat_steel_shaft = {"diffuse": (0.85, 0.88, 0.92), "specular": (1.0, 1.0, 1.0), "shininess": 0.95}   # Ground stainless steel
mat_printed_adapter = {"diffuse": (0.12, 0.55, 0.90), "specular": (0.4, 0.6, 0.8), "shininess": 0.5} # Technical Blue PETG
mat_printed_holder = {"diffuse": (0.95, 0.55, 0.15), "specular": (0.5, 0.4, 0.2), "shininess": 0.6}  # High-vis Orange PETG
mat_magnet = {"diffuse": (0.80, 0.20, 0.20), "specular": (0.8, 0.8, 0.9), "shininess": 0.85}         # Red diametric magnet
mat_pcb_sensor = {"diffuse": (0.15, 0.65, 0.35), "specular": (0.3, 0.5, 0.3), "shininess": 0.4}      # FR4 Green PCB
mat_screws = {"diffuse": (0.20, 0.22, 0.25), "specular": (0.7, 0.7, 0.8), "shininess": 0.85}          # Black oxide steel screws

# 1. Full Assembly View (Rear Isometric - showing adapter and sensor)
full_assembly_shapes = [
    (gearbox, mat_gearbox),
    (boss, mat_gearbox),
    (shaft, mat_steel_shaft),
    (housing, mat_motor_housing),
    (rear_shaft, mat_steel_shaft),
    (adapter, mat_printed_adapter),
    (holder, mat_printed_holder),
    (magnet, mat_magnet),
    (sensor, mat_pcb_sensor),
    (pcb_screws, mat_screws)
]
if clamp_screw is not None:
    full_assembly_shapes.append((clamp_screw, mat_screws))
if clamp_nut is not None:
    full_assembly_shapes.append((clamp_nut, mat_screws))

out1 = os.path.join(RENDERS_DIR, "render_ensamble_completo.png")
render_scene(full_assembly_shapes, out1, cam_orientation=(1.25, 0.65), cam_zoom=1.0)

# 2. Opposite Isometric View (showing the motor front and coupler)
out2 = os.path.join(RENDERS_DIR, "render_vista_frontal_lateral.png")
render_scene(full_assembly_shapes, out2, cam_orientation=(0.6, -2.4), cam_zoom=1.0)

# 3. 3D-Printed Adapter Alone
adapter_alone_shapes = [
    (adapter, mat_printed_adapter)
]
out3 = os.path.join(RENDERS_DIR, "render_acoplador_3d.png")
render_scene(adapter_alone_shapes, out3, cam_orientation=(1.2, 0.7), cam_zoom=1.1)

# 4. Cross-Section Cutaway View (Corte Transversal)
cut_box = Part.makeBox(100.0, 100.0, 150.0, App.Vector(-50.0, 0.0, -100.0))
cut_shapes = [
    (gearbox.cut(cut_box), mat_gearbox),
    (boss.cut(cut_box), mat_gearbox),
    (shaft.cut(cut_box), mat_steel_shaft),
    (housing.cut(cut_box), mat_motor_housing),
    (rear_shaft.cut(cut_box), mat_steel_shaft),
    (adapter.cut(cut_box), mat_printed_adapter),
    (holder.cut(cut_box), mat_printed_holder),
    (magnet.cut(cut_box), mat_magnet),
    (pcb_screws.cut(cut_box), mat_screws)
]
if clamp_screw is not None:
    cut_shapes.append((clamp_screw.cut(cut_box), mat_screws))
if clamp_nut is not None:
    cut_shapes.append((clamp_nut.cut(cut_box), mat_screws))
out4 = os.path.join(RENDERS_DIR, "render_corte_transversal.png")
render_scene(cut_shapes, out4, cam_orientation=(1.1, 0.4), cam_zoom=1.2)

print("All photorealistic renders generated successfully!")
