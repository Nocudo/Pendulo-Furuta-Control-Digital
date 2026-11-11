"""
=============================================================================
PROYECTO FINAL - CONTROL DIGITAL: RENDERIZADOR COIN3D FOTORREALISTA 1080P
Genera 6 Vistas Técnicas de Alta Definición para la Caja del Driver L298N
=============================================================================
"""

import os
import sys


def _ensure_freecad_path():
    try:
        import FreeCAD  # noqa: F401
        return
    except ImportError:
        candidates = []
        env_bin = os.environ.get("FREECAD_BIN")
        if env_bin:
            candidates.append(env_bin)
        local_app = os.environ.get("LOCALAPPDATA", "")
        prog_files = os.environ.get("ProgramFiles", r"C:\Program Files")
        for ver in ("FreeCAD 1.1", "FreeCAD 1.0", "FreeCAD 0.21"):
            if local_app:
                candidates.append(os.path.join(local_app, "Programs", ver, "bin"))
            if prog_files:
                candidates.append(os.path.join(prog_files, ver, "bin"))
        candidates.extend([
            "/usr/lib/freecad-python3/lib",
            "/usr/lib/freecad/lib",
            "/Applications/FreeCAD.app/Contents/Resources/lib",
        ])
        for path in candidates:
            if path and os.path.isdir(path):
                if path not in sys.path:
                    sys.path.append(path)
                lib_dir = os.path.join(os.path.dirname(path), "lib")
                if os.path.isdir(lib_dir) and lib_dir not in sys.path:
                    sys.path.append(lib_dir)
                try:
                    import FreeCAD  # noqa: F401
                    return
                except ImportError:
                    continue
        raise RuntimeError(
            "FreeCAD no encontrado. Ejecuta este script con el Python de FreeCAD "
            "o define la variable de entorno FREECAD_BIN."
        )


_ensure_freecad_path()

import FreeCAD as App
import Part
import pivy.coin as coin
from PIL import Image

print("=== INICIANDO RENDERIZADO COIN3D PARA CAJA DRIVER L298N ===")

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
CAJA_L298N_DIR = os.path.dirname(SCRIPT_DIR)
RENDERS_DIR = os.path.join(CAJA_L298N_DIR, "renders")
os.makedirs(RENDERS_DIR, exist_ok=True)

def render_scene(shapes_and_materials, output_path, cam_orientation=(1.15, -0.85), cam_zoom=1.0, bg_color=(0.11, 0.13, 0.17), width=1920, height=1080):
    root = coin.SoSeparator()
    
    # Cámara de perspectiva
    cam = coin.SoPerspectiveCamera()
    rot1 = coin.SbRotation(coin.SbVec3f(1, 0, 0), cam_orientation[0])
    rot2 = coin.SbRotation(coin.SbVec3f(0, 0, 1), cam_orientation[1])
    cam.orientation.setValue(rot1 * rot2)
    root.addChild(cam)
    
    # Iluminación de Estudio Profesional de 4 Puntos
    light1 = coin.SoDirectionalLight()
    light1.direction.setValue(-1.5, -1.2, -1.3)
    light1.intensity.setValue(1.30)
    root.addChild(light1)
    
    light2 = coin.SoDirectionalLight()
    light2.direction.setValue(1.4, 1.3, -0.8)
    light2.intensity.setValue(0.75)
    root.addChild(light2)

    light3 = coin.SoDirectionalLight()
    light3.direction.setValue(0.0, 0.0, 1.4)
    light3.intensity.setValue(0.60)
    root.addChild(light3)

    light4 = coin.SoDirectionalLight()
    light4.direction.setValue(-0.8, 0.0, 0.9)
    light4.intensity.setValue(0.80)
    root.addChild(light4)

    for shape, mat_props in shapes_and_materials:
        sep = coin.SoSeparator()
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
        
        iv_str = shape.writeInventor(2, 0.03)
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
    print(f"[OK] Render guardado: {output_path}")

# Abrir documento FreeCAD con el ensamblaje completo oficial
doc_path = os.path.join(CAJA_L298N_DIR, "Caja_L298N.FCStd")
doc = App.openDocument(doc_path)

# Extraer formas del documento
base = doc.getObject("Caja_Base_L298N").Shape
lid = doc.getObject("Tapa_Caja_L298N").Shape
lid_screws = doc.getObject("Tornillos_Tapa_M3").Shape

board_body = doc.getObject("L298N_Cuerpo_Principal").Shape
mot_a_sc = doc.getObject("L298N_Tornillos_Motor_A").Shape
mot_b_sc = doc.getObject("L298N_Tornillos_Motor_B").Shape
pwr_sc = doc.getObject("L298N_Tornillos_Power").Shape
chip_pins = doc.getObject("L298N_Pines_Chip").Shape
logic_hdr = doc.getObject("L298N_Header_Control_Logico").Shape
cap_reg = doc.getObject("L298N_Capacitor_Regulador").Shape
pcb_screws = doc.getObject("Tornillos_Fijacion_PCB_M3").Shape

# Materiales Fotorrealistas PBR
mat_case_base = {"diffuse": (0.18, 0.20, 0.25), "specular": (0.40, 0.45, 0.50), "shininess": 0.45} # Gris pizarra técnico
mat_case_lid = {"diffuse": (0.95, 0.42, 0.12), "specular": (0.50, 0.70, 0.85), "shininess": 0.60}  # Naranja técnico
mat_l298n_body = {"diffuse": (0.12, 0.45, 0.22), "specular": (0.35, 0.40, 0.35), "shininess": 0.50}# PCB FR4 Verde
mat_steel = {"diffuse": (0.86, 0.87, 0.90), "specular": (1.0, 1.0, 1.0), "shininess": 0.95}       # Tornillos bornera acero
mat_gold = {"diffuse": (0.88, 0.75, 0.30), "specular": (1.0, 0.9, 0.4), "shininess": 0.90}        # Pines dorados
mat_nylon_black = {"diffuse": (0.10, 0.10, 0.10), "specular": (0.3, 0.3, 0.3), "shininess": 0.40}  # Cabezales nylon negro
mat_screws = {"diffuse": (0.78, 0.78, 0.82), "specular": (0.9, 0.9, 0.95), "shininess": 0.85}     # Tornillos acero A2

driver_all = [
    (board_body, mat_l298n_body),
    (mot_a_sc, mat_steel),
    (mot_b_sc, mat_steel),
    (pwr_sc, mat_steel),
    (chip_pins, mat_gold),
    (logic_hdr, mat_nylon_black),
    (cap_reg, mat_nylon_black),
    (pcb_screws, mat_screws)
]

# 1. Ensamble Cerrado - Vista Isométrica mostrando Chimenea y Ventanas de Borneras
scene_closed = [
    (base, mat_case_base),
    (lid, mat_case_lid),
    (lid_screws, mat_screws),
    (mot_a_sc, mat_steel),
    (mot_b_sc, mat_steel),
    (pwr_sc, mat_steel),
    (logic_hdr, mat_nylon_black)
]
out1 = os.path.join(RENDERS_DIR, "render_caja_l298n_cerrada.png")
render_scene(scene_closed, out1, cam_orientation=(1.15, -0.85), cam_zoom=0.92)

# 2. Vista Interior sin Tapa (Driver L298N Asentado en Separadores H=5mm)
scene_open = [(base, mat_case_base)] + driver_all
out2 = os.path.join(RENDERS_DIR, "render_caja_l298n_interna.png")
render_scene(scene_open, out2, cam_orientation=(1.12, -0.80), cam_zoom=0.95)

# 3. Vista en Despiece Explosivo
def translate_items(items, dz):
    res = []
    for s, m in items:
        sc = s.copy()
        sc.translate(App.Vector(0,0,dz))
        res.append((sc, m))
    return res

scene_exploded = (
    [(base, mat_case_base)] +
    translate_items(driver_all[:-1], 25.0) +
    translate_items([(pcb_screws, mat_screws)], 35.0) +
    translate_items([(lid, mat_case_lid)], 65.0) +
    translate_items([(lid_screws, mat_screws)], 85.0)
)
out3 = os.path.join(RENDERS_DIR, "render_caja_l298n_despiece.png")
render_scene(scene_exploded, out3, cam_orientation=(1.22, -0.80), cam_zoom=0.74)

# 4. Corte Transversal (Cutaway View mostrando espacio inferior y flujo de aire)
cut_box = Part.makeBox(120.0, 120.0, 60.0, App.Vector(-60.0, -120.0, -5.0))
scene_cut = [
    (base.cut(cut_box), mat_case_base),
    (lid.cut(cut_box), mat_case_lid),
    (lid_screws.cut(cut_box), mat_screws)
]
for s, m in driver_all:
    c = s.cut(cut_box)
    if c.Volume > 0.001:
        scene_cut.append((c, m))
        
out4 = os.path.join(RENDERS_DIR, "render_caja_l298n_corte.png")
render_scene(scene_cut, out4, cam_orientation=(1.05, 0.35), cam_zoom=1.05)

# 5. Vista Inferior (Rejilla de 6 Ranuras y Cajeras de Patas)
scene_bottom = [
    (base, mat_case_base)
]
out5 = os.path.join(RENDERS_DIR, "render_caja_l298n_inferior.png")
render_scene(scene_bottom, out5, cam_orientation=(2.30, 0.80), cam_zoom=1.0)

# 6. Detalle Macro de Conexiones (Borneras y Header Lógico)
scene_detail = [(base, mat_case_base)] + driver_all
out6 = os.path.join(RENDERS_DIR, "render_caja_l298n_detalle_conexiones.png")
render_scene(scene_detail, out6, cam_orientation=(1.18, -0.75), cam_zoom=1.65)

print("\n¡Los 6 Renders Coin3D 1080p de la Caja L298N fueron generados exitosamente!")
