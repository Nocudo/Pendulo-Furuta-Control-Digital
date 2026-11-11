import os
import math
import FreeCAD as App
import Part
import pivy.coin as coin
from PIL import Image

print("=== INICIANDO RENDERIZADO COIN3D PARA CAJA ESP32 (PUERTOS LADO A LADO Y AGUJEROS COMPLETOS) ===")

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
MODEL_DIR = os.path.dirname(SCRIPT_DIR)
RENDERS_DIR = os.path.join(MODEL_DIR, "renders")
os.makedirs(RENDERS_DIR, exist_ok=True)

def render_scene(shapes_and_materials, output_path, cam_orientation=(1.15, -0.85), cam_zoom=1.0, bg_color=(0.10, 0.12, 0.16), width=1920, height=1080):
    root = coin.SoSeparator()
    
    # Camera
    cam = coin.SoPerspectiveCamera()
    rot1 = coin.SbRotation(coin.SbVec3f(1, 0, 0), cam_orientation[0])
    rot2 = coin.SbRotation(coin.SbVec3f(0, 0, 1), cam_orientation[1])
    cam.orientation.setValue(rot1 * rot2)
    root.addChild(cam)
    
    # 4-Point Studio Lighting
    # Key light on front connectors
    light1 = coin.SoDirectionalLight()
    light1.direction.setValue(-1.4, -1.2, -1.5)
    light1.intensity.setValue(1.15)
    root.addChild(light1)
    
    # Fill light on side and top
    light2 = coin.SoDirectionalLight()
    light2.direction.setValue(1.5, 1.2, -0.8)
    light2.intensity.setValue(0.65)
    root.addChild(light2)

    # Top ambient rim light
    light3 = coin.SoDirectionalLight()
    light3.direction.setValue(0.0, 0.0, 1.2)
    light3.intensity.setValue(0.45)
    root.addChild(light3)

    # Bottom uplight for underside and interior recesses
    light4 = coin.SoDirectionalLight()
    light4.direction.setValue(-0.5, -0.5, 1.5)
    light4.intensity.setValue(0.75)
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
        
        iv_str = shape.writeInventor(2, 0.030)
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

# Abrir documento FreeCAD con el ensamblaje completo
doc_path = os.path.join(MODEL_DIR, "Caja_ESP32_Expansion.FCStd")
doc = App.openDocument(doc_path)

base = doc.getObject("Caja_Base_ESP32").Shape
lid = doc.getObject("Tapa_Caja_ESP32").Shape
exp_pcb = doc.getObject("Placa_Expansion_PCB").Shape
term_blocks = doc.getObject("Borneras_Tornillo_Verdes").Shape
term_screws = doc.getObject("Tornillos_Borneras").Shape
headers = doc.getObject("Headers_Hembra_Zocalo").Shape
dc_jack = doc.getObject("Conector_Alimentacion_DC_Jack").Shape
exp_elec = doc.getObject("Componentes_Expansion").Shape
pcb_screws = doc.getObject("Tornillos_Fijacion_PCB_M3").Shape
lid_screws = doc.getObject("Tornillos_Tapa_M3").Shape

# Componentes del ESP32 DevKit V1 Real
esp_pcb = doc.getObject("ESP32_DevKit_PCB").Shape
esp_pins = doc.getObject("ESP32_DevKit_Pines").Shape
esp_shield = doc.getObject("ESP32_Blindaje_WROOM32").Shape
esp_ics = doc.getObject("ESP32_Chips_CP2102_SMD").Shape
esp_buttons = doc.getObject("ESP32_Pulsadores_EN_BOOT").Shape
esp_usb = doc.getObject("ESP32_Conector_MicroUSB").Shape

# Paleta Cromática y Materiales PBR
mat_case_base = {"diffuse": (0.16, 0.18, 0.22), "specular": (0.35, 0.40, 0.45), "shininess": 0.40} # Gris antracita mate
mat_case_lid = {"diffuse": (0.00, 0.65, 0.88), "specular": (0.45, 0.85, 0.95), "shininess": 0.65}  # Cian tecnológico
mat_exp_pcb = {"diffuse": (0.06, 0.42, 0.16), "specular": (0.25, 0.50, 0.25), "shininess": 0.50}   # Verde esmeralda FR4
mat_tb = {"diffuse": (0.12, 0.68, 0.22), "specular": (0.35, 0.60, 0.30), "shininess": 0.55}        # Verde bornera
mat_tb_sc = {"diffuse": (0.85, 0.75, 0.35), "specular": (0.9, 0.85, 0.5), "shininess": 0.80}       # Tornillos latón dorado
mat_hdr = {"diffuse": (0.08, 0.08, 0.10), "specular": (0.2, 0.2, 0.2), "shininess": 0.30}          # Nylon negro headers
mat_dc = {"diffuse": (0.10, 0.10, 0.12), "specular": (0.3, 0.3, 0.3), "shininess": 0.40}           # Jack DC
mat_screws = {"diffuse": (0.75, 0.75, 0.80), "specular": (0.9, 0.9, 0.95), "shininess": 0.85}      # Acero inoxidable

# Materiales para el ESP32 DevKit V1 Real
mat_esp_pcb = {"diffuse": (0.06, 0.06, 0.08), "specular": (0.3, 0.3, 0.3), "shininess": 0.45}      # PCB negro mate
mat_esp_pins = {"diffuse": (0.88, 0.74, 0.25), "specular": (1.0, 0.9, 0.4), "shininess": 0.90}      # Pines dorados
mat_esp_shield = {"diffuse": (0.88, 0.90, 0.94), "specular": (1.0, 1.0, 1.0), "shininess": 0.95}    # Blindaje metálico RF
mat_esp_ics = {"diffuse": (0.12, 0.12, 0.14), "specular": (0.4, 0.4, 0.4), "shininess": 0.60}       # Chips silicio CP2102
mat_esp_btn = {"diffuse": (0.72, 0.72, 0.76), "specular": (0.8, 0.8, 0.85), "shininess": 0.75}      # Pulsadores EN/BOOT
mat_esp_usb = {"diffuse": (0.82, 0.84, 0.88), "specular": (1.0, 1.0, 1.0), "shininess": 0.95}       # Micro-USB niquelado

# 1. Ensamble Cerrado - Vista Isométrica mostrando Puertos USB y Jack DC Lado a Lado
scene_closed = [
    (base, mat_case_base),
    (lid, mat_case_lid),
    (lid_screws, mat_screws)
]
out1 = os.path.join(RENDERS_DIR, "render_caja_esp32_cerrada.png")
render_scene(scene_closed, out1, cam_orientation=(1.15, -0.85), cam_zoom=0.92)

# 2. Vista Interior sin Tapa (Módulo ESP32 Real y Expansión con Puertos Frontales Lado a Lado)
scene_open = [
    (base, mat_case_base),
    (exp_pcb, mat_exp_pcb),
    (term_blocks, mat_tb),
    (term_screws, mat_tb_sc),
    (headers, mat_hdr),
    (dc_jack, mat_dc),
    (exp_elec, mat_esp_ics),
    (pcb_screws, mat_screws),
    (esp_pcb, mat_esp_pcb),
    (esp_pins, mat_esp_pins),
    (esp_shield, mat_esp_shield),
    (esp_ics, mat_esp_ics),
    (esp_buttons, mat_esp_btn),
    (esp_usb, mat_esp_usb)
]
out2 = os.path.join(RENDERS_DIR, "render_caja_esp32_interna.png")
render_scene(scene_open, out2, cam_orientation=(1.12, -0.80), cam_zoom=0.95)

# 3. Vista en Despiece Explosivo (Exploded View alineada verticalmente)
exp_pcb_e = exp_pcb.copy(); exp_pcb_e.translate(App.Vector(0,0,18))
tb_e = term_blocks.copy(); tb_e.translate(App.Vector(0,0,18))
tb_sc_e = term_screws.copy(); tb_sc_e.translate(App.Vector(0,0,18))
hdr_e = headers.copy(); hdr_e.translate(App.Vector(0,0,18))
dc_e = dc_jack.copy(); dc_e.translate(App.Vector(0,0,18))
exp_el_e = exp_elec.copy(); exp_el_e.translate(App.Vector(0,0,18))
pcb_sc_e = pcb_screws.copy(); pcb_sc_e.translate(App.Vector(0,0,24))

esp_pcb_e = esp_pcb.copy(); esp_pcb_e.translate(App.Vector(0,0,42))
esp_pins_e = esp_pins.copy(); esp_pins_e.translate(App.Vector(0,0,42))
esp_shield_e = esp_shield.copy(); esp_shield_e.translate(App.Vector(0,0,42))
esp_ics_e = esp_ics.copy(); esp_ics_e.translate(App.Vector(0,0,42))
esp_btn_e = esp_buttons.copy(); esp_btn_e.translate(App.Vector(0,0,42))
esp_usb_e = esp_usb.copy(); esp_usb_e.translate(App.Vector(0,0,42))

lid_e = lid.copy(); lid_e.translate(App.Vector(0,0,72))
lid_sc_e = lid_screws.copy(); lid_sc_e.translate(App.Vector(0,0,92))

scene_exploded = [
    (base, mat_case_base),
    (exp_pcb_e, mat_exp_pcb),
    (tb_e, mat_tb),
    (tb_sc_e, mat_tb_sc),
    (hdr_e, mat_hdr),
    (dc_e, mat_dc),
    (exp_el_e, mat_esp_ics),
    (pcb_sc_e, mat_screws),
    (esp_pcb_e, mat_esp_pcb),
    (esp_pins_e, mat_esp_pins),
    (esp_shield_e, mat_esp_shield),
    (esp_ics_e, mat_esp_ics),
    (esp_btn_e, mat_esp_btn),
    (esp_usb_e, mat_esp_usb),
    (lid_e, mat_case_lid),
    (lid_sc_e, mat_screws)
]
out3 = os.path.join(RENDERS_DIR, "render_caja_esp32_despiece.png")
render_scene(scene_exploded, out3, cam_orientation=(1.22, -0.80), cam_zoom=0.76)

# 4. Corte Transversal (Cutaway View mostrando encastre y agujeros pasantes completos)
# Cortamos la mitad frontal para ver el interior en seccion
cut_box = Part.makeBox(140.0, 100.0, 80.0, App.Vector(-70.0, -100.0, -10.0))
scene_cut = [
    (base.cut(cut_box), mat_case_base),
    (lid.cut(cut_box), mat_case_lid),
    (exp_pcb.cut(cut_box), mat_exp_pcb),
    (term_blocks.cut(cut_box), mat_tb),
    (term_screws.cut(cut_box), mat_tb_sc),
    (headers.cut(cut_box), mat_hdr),
    (dc_jack.cut(cut_box), mat_dc),
    (pcb_screws.cut(cut_box), mat_screws),
    (esp_pcb.cut(cut_box), mat_esp_pcb),
    (esp_pins.cut(cut_box), mat_esp_pins),
    (esp_shield.cut(cut_box), mat_esp_shield),
    (esp_ics.cut(cut_box), mat_esp_ics),
    (esp_buttons.cut(cut_box), mat_esp_btn),
    (esp_usb.cut(cut_box), mat_esp_usb),
    (lid_screws.cut(cut_box), mat_screws)
]
out4 = os.path.join(RENDERS_DIR, "render_caja_esp32_corte.png")
render_scene(scene_cut, out4, cam_orientation=(1.05, 0.35), cam_zoom=1.05)

# 5. Vista Inferior (Rejillas Convectivas Pasantes y Alojamientos de Patas)
scene_bottom = [
    (base, mat_case_base)
]
out5 = os.path.join(RENDERS_DIR, "render_caja_esp32_inferior.png")
render_scene(scene_bottom, out5, cam_orientation=(2.30, 0.80), cam_zoom=1.0)

# 6. Detalle Macro Primer Plano de los Puertos USB-C y Jack DC Lado a Lado
scene_detail = [
    (base, mat_case_base),
    (exp_pcb, mat_exp_pcb),
    (term_blocks, mat_tb),
    (term_screws, mat_tb_sc),
    (headers, mat_hdr),
    (dc_jack, mat_dc),
    (exp_elec, mat_esp_ics),
    (esp_pcb, mat_esp_pcb),
    (esp_pins, mat_esp_pins),
    (esp_shield, mat_esp_shield),
    (esp_ics, mat_esp_ics),
    (esp_buttons, mat_esp_btn),
    (esp_usb, mat_esp_usb)
]
out6 = os.path.join(RENDERS_DIR, "render_caja_esp32_detalle_real.png")
render_scene(scene_detail, out6, cam_orientation=(1.18, -0.75), cam_zoom=1.65)

print("=== TODOS LOS RENDERS FOTORREALISTAS GENERADOS EN renders/ ===")
