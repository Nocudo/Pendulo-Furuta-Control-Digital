import sys
import os
import math


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

print(f"=== FreeCAD {App.Version()[0]}.{App.Version()[1]}.{App.Version()[2]} iniciado ===")

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
MODEL_DIR = os.path.dirname(SCRIPT_DIR)
COMPONENTS_DIR = os.path.join(MODEL_DIR, "components")
os.makedirs(MODEL_DIR, exist_ok=True)

# Paths a los modelos STEP reales descargados
ESP32_STEP_PATH = os.path.join(COMPONENTS_DIR, "esp32_dev_kit_v1_asm.stp")
BREAKOUT_STEP_PATH = os.path.join(COMPONENTS_DIR, "3D_ESP32_Breakout_V2.step")
HEADER_STEP_PATH = os.path.join(COMPONENTS_DIR, "15-Pin Header - Female - 2.54mm.step")
BARREL_JACK_STEP_PATH = os.path.join(COMPONENTS_DIR, "BarrelJack.step")

def make_fillet_box(length, width, height, radius, pos=App.Vector(0,0,0)):
    b = Part.makeBox(length - 2*radius, width, height, pos + App.Vector(radius, 0, 0))
    b2 = Part.makeBox(length, width - 2*radius, height, pos + App.Vector(0, radius, 0))
    fused = b.fuse(b2)
    c1 = Part.makeCylinder(radius, height, pos + App.Vector(radius, radius, 0))
    c2 = Part.makeCylinder(radius, height, pos + App.Vector(length - radius, radius, 0))
    c3 = Part.makeCylinder(radius, height, pos + App.Vector(length - radius, width - radius, 0))
    c4 = Part.makeCylinder(radius, height, pos + App.Vector(radius, width - radius, 0))
    return fused.fuse(c1).fuse(c2).fuse(c3).fuse(c4)

def build_base_box():
    # 1. Cuerpo exterior: 84.0 x 70.0 x 27.0 mm con esquinas redondeadas R=4.0 mm
    outer = make_fillet_box(84.0, 70.0, 27.0, 4.0, App.Vector(-42.0, -35.0, 0.0))
    
    # 2. Cavidad interna: 74.0 x 60.0 x 24.4 mm (Paredes de 3.0 mm, fondo de 2.6 mm)
    inner = make_fillet_box(74.0, 60.0, 26.0, 2.0, App.Vector(-37.0, -30.0, 2.6))
    base = outer.cut(inner)
    
    # 3. 4 Columnas / Torretas de esquina para tornillos M3 de la tapa
    cols = []
    col_coords = [(-35.0, -28.0), (35.0, -28.0), (35.0, 28.0), (-35.0, 28.0)]
    for cx, cy in col_coords:
        col = Part.makeCylinder(4.5, 24.4, App.Vector(cx, cy, 2.6), App.Vector(0,0,1))
        hole = Part.makeCylinder(1.4, 25.0, App.Vector(cx, cy, 5.0), App.Vector(0,0,1))
        cols.append(col.cut(hole))
    base = base.fuse(Part.Compound(cols))
    
    # 4. 4 Separadores / Standoffs para la placa de expansión (elevación 5.0 mm)
    standoffs = []
    for sx in [-30.0, 30.0]:
        for sy in [-24.0, 24.0]:
            st = Part.makeCylinder(3.25, 5.0, App.Vector(sx, sy, 2.6), App.Vector(0,0,1))
            st_extra = Part.makeBox(6.5, 3.0, 5.0, App.Vector(sx - 3.25, sy - 1.5, 2.6))
            st = st.fuse(st_extra)
            sh = Part.makeCylinder(1.35, 6.0, App.Vector(sx, sy, 2.0), App.Vector(0,0,1))
            standoffs.append(st.cut(sh))
    base = base.fuse(Part.Compound(standoffs))
    
    # 5. Ventana para conector USB (Cara -X, centrada): 14.0 mm ancho x 15.0 mm alto
    usb_cut = Part.makeBox(8.0, 14.0, 15.0, App.Vector(-43.0, -7.0, 13.0))
    
    # 6. Ranuras laterales para salida de cables de las borneras (Caras +Y y -Y):
    side_cut_pos = Part.makeBox(48.0, 8.0, 7.5, App.Vector(-24.0, 31.0, 9.0))
    side_cut_neg = Part.makeBox(48.0, 8.0, 7.5, App.Vector(-24.0, -39.0, 9.0))
    
    # 7. Ventana para alimentación Jack DC (Cara +X, Y=14.0 mm): Diámetro 11 mm
    dc_cut = Part.makeCylinder(5.5, 10.0, App.Vector(39.0, 14.0, 14.7), App.Vector(1,0,0))
    
    # 8. Rejilla de ventilación inferior (6 ranuras bajo el módulo ESP32)
    vents = []
    for vy in [-15.0, -9.0, -3.0, 3.0, 9.0, 15.0]:
        v = Part.makeBox(22.0, 2.4, 4.0, App.Vector(-11.0, vy, -0.5))
        vents.append(v)
    vents_compound = Part.Compound(vents)
    
    # 9. 4x Alojamientos circulares inferiores para patas de goma antideslizantes
    feet = []
    for fx in [-34.0, 34.0]:
        for fy in [-26.0, 26.0]:
            ft = Part.makeCylinder(4.25, 1.3, App.Vector(fx, fy, -0.1), App.Vector(0,0,1))
            feet.append(ft)
    feet_compound = Part.Compound(feet)
    
    base = base.cut(usb_cut).cut(side_cut_pos).cut(side_cut_neg).cut(dc_cut).cut(vents_compound).cut(feet_compound)
    return base

def build_top_lid():
    # 1. Placa superior: 84.0 x 70.0 x 2.5 mm
    lid_plate = make_fillet_box(84.0, 70.0, 2.5, 4.0, App.Vector(-42.0, -35.0, 27.0))
    
    # 2. Labio perimetral de encaje interno: 73.5 x 59.5 x 1.8 mm
    inner_lip = make_fillet_box(73.5, 59.5, 1.8, 1.8, App.Vector(-36.75, -29.75, 25.2))
    lip_pocket = make_fillet_box(68.5, 54.5, 2.0, 1.5, App.Vector(-34.25, -27.25, 25.0))
    inner_lip = inner_lip.cut(lip_pocket)
    lid = lid_plate.fuse(inner_lip)
    
    # 3. 4 Agujeros avellanados para tornillos M3 en esquinas
    col_coords = [(-35.0, -28.0), (35.0, -28.0), (35.0, 28.0), (-35.0, 28.0)]
    holes = []
    for cx, cy in col_coords:
        cyl = Part.makeCylinder(1.65, 5.0, App.Vector(cx, cy, 24.5), App.Vector(0,0,1))
        cone = Part.makeCone(3.2, 1.65, 1.6, App.Vector(cx, cy, 27.9), App.Vector(0,0,1))
        holes.append(cyl.fuse(cone))
    
    # 4. Rejillas superiores de ventilación tipo persiana
    top_vents = []
    for vy in [-21.0, -15.0, -9.0, -3.0, 3.0, 9.0, 15.0, 21.0]:
        tv = Part.makeBox(38.0, 2.2, 5.0, App.Vector(-19.0, vy, 26.0))
        top_vents.append(tv)
    
    lid = lid.cut(Part.Compound(holes)).cut(Part.Compound(top_vents))
    return lid

def build_expansion_board_parts():
    """Construye la placa de expansión utilizando los modelos CAD STEP reales de borneras, headers y Jack DC."""
    z_pcb_bot = 7.6
    z_pcb_top = 9.2
    
    # 1. PCB de la placa de expansión: 68.0 x 54.0 x 1.6 mm con agujeros M3
    pcb_raw = make_fillet_box(68.0, 54.0, 1.6, 2.5, App.Vector(-34.0, -27.0, z_pcb_bot))
    pcb_holes = []
    for hx in [-30.0, 30.0]:
        for hy in [-24.0, 24.0]:
            h = Part.makeCylinder(1.6, 2.0, App.Vector(hx, hy, z_pcb_bot - 0.2), App.Vector(0,0,1))
            pcb_holes.append(h)
    exp_pcb = pcb_raw.cut(Part.Compound(pcb_holes))
    
    # 2. MODELO STEP REAL: Headers Hembra de 15 Pines (2 filas a Y = ±12.7 mm)
    print(f"Cargando Headers Hembra reales desde: {HEADER_STEP_PATH}")
    hdr_raw = Part.read(HEADER_STEP_PATH)
    hdr_rot = hdr_raw.copy()
    hdr_rot.rotate(App.Vector(0,0,0), App.Vector(1,0,0), 90)
    
    hdr_left = hdr_rot.copy()
    hdr_left.translate(App.Vector(-2.25 - 17.78, 12.7, z_pcb_top))
    
    hdr_right = hdr_rot.copy()
    hdr_right.translate(App.Vector(-2.25 - 17.78, -12.7, z_pcb_top))
    headers = hdr_left.fuse(hdr_right)
    
    # 3. MODELO STEP REAL: Conector Jack DC (BarrelJack.step)
    print(f"Cargando Jack DC real desde: {BARREL_JACK_STEP_PATH}")
    dc_raw = Part.read(BARREL_JACK_STEP_PATH)
    dc_rot = dc_raw.copy()
    dc_rot.rotate(App.Vector(0,0,0), App.Vector(0,0,1), -90)
    dc_rot.translate(App.Vector(19.0, 14.0, z_pcb_top))
    
    # 4. MODELO STEP REAL: Borneras de Conexión de Tornillo Verdes (3D_ESP32_Breakout_V2.step)
    print(f"Cargando Borneras de tornillo reales desde: {BREAKOUT_STEP_PATH}")
    s_breakout = Part.read(BREAKOUT_STEP_PATH)
    # Extraer 3 bloques de 5 bornes = 15 bornes (45 sólidos: cuerpos + tornillos + bornes)
    tb_15_solids = s_breakout.Solids[16:61]
    
    tb_bodies_raw = [sol for sol in tb_15_solids if sol.Volume > 100]
    tb_metals_raw = [sol for sol in tb_15_solids if sol.Volume <= 100]
    
    comp_bodies = Part.Compound(tb_bodies_raw)
    comp_metals = Part.Compound(tb_metals_raw)
    
    # Fila positiva (+Y): wire holes hacia +Y
    tb_pos_body = comp_bodies.copy()
    tb_pos_body.rotate(App.Vector(0,0,0), App.Vector(0,0,1), -90)
    tb_pos_body.translate(App.Vector(-7.85, 4.72, z_pcb_top))
    
    tb_pos_metal = comp_metals.copy()
    tb_pos_metal.rotate(App.Vector(0,0,0), App.Vector(0,0,1), -90)
    tb_pos_metal.translate(App.Vector(-7.85, 4.72, z_pcb_top))
    
    # Fila negativa (-Y): wire holes hacia -Y
    tb_neg_body = comp_bodies.copy()
    tb_neg_body.rotate(App.Vector(0,0,0), App.Vector(0,0,1), 90)
    tb_neg_body.translate(App.Vector(3.35, -4.72, z_pcb_top))
    
    tb_neg_metal = comp_metals.copy()
    tb_neg_metal.rotate(App.Vector(0,0,0), App.Vector(0,0,1), 90)
    tb_neg_metal.translate(App.Vector(3.35, -4.72, z_pcb_top))
    
    tb_bodies = tb_pos_body.fuse(tb_neg_body)
    tb_metals = Part.Compound([tb_pos_metal, tb_neg_metal])
    
    # 5. Componentes SMD adicionales en la placa (Regulador SOT-223, LEDs, Capacitores)
    reg_body = Part.makeBox(6.5, 3.5, 1.8, App.Vector(-5.0, 8.0, z_pcb_top))
    reg_tab = Part.makeBox(3.5, 1.5, 0.5, App.Vector(-3.5, 11.5, z_pcb_top))
    reg = reg_body.fuse(reg_tab)
    cap1 = Part.makeCylinder(3.15, 6.0, App.Vector(12.0, 14.0, z_pcb_top), App.Vector(0,0,1))
    led_pwr = Part.makeBox(1.6, 0.8, 0.8, App.Vector(6.0, 8.0, z_pcb_top))
    exp_electronics = reg.fuse(cap1).fuse(led_pwr)
    
    # 6. Tornillos M3 de fijación de PCB a standoffs
    pcb_screws = []
    for hx in [-30.0, 30.0]:
        for hy in [-24.0, 24.0]:
            sc_head = Part.makeCylinder(2.75, 1.8, App.Vector(hx, hy, z_pcb_top), App.Vector(0,0,1))
            pcb_screws.append(sc_head)
            
    # 7. Tornillos M3 avellanados de la tapa
    lid_screws = []
    for cx, cy in [(-35.0, -28.0), (35.0, -28.0), (35.0, 28.0), (-35.0, 28.0)]:
        ls = Part.makeCone(3.1, 1.6, 1.6, App.Vector(cx, cy, 27.9), App.Vector(0,0,1))
        ls_slot = Part.makeBox(6.0, 0.6, 1.0, App.Vector(cx - 3.0, cy - 0.3, 28.7))
        lid_screws.append(ls.cut(ls_slot))
        
    return exp_pcb, tb_bodies, tb_metals, headers, dc_rot, exp_electronics, Part.Compound(pcb_screws), Part.Compound(lid_screws)

def load_and_transform_esp32():
    """Carga el modelo STEP real del ESP32 DevKit V1 y lo orienta/posiciona con precisión milimétrica."""
    print(f"Cargando modelo CAD real de ESP32 desde: {ESP32_STEP_PATH}")
    shape_raw = Part.read(ESP32_STEP_PATH)
    
    s_rot = shape_raw.copy()
    s_rot.rotate(App.Vector(0,0,0), App.Vector(0,0,1), -90)
    s_rot.translate(App.Vector(-28.0, 0.0, 17.7))
    
    solids = s_rot.Solids
    print(f"ESP32 procesado con éxito: {len(solids)} cuerpos sólidos independientes.")
    
    esp32_pcb = solids[0]
    esp32_pins = Part.Compound(solids[1:31])
    esp32_shield = solids[31]
    esp32_ics = Part.Compound(solids[32:50])
    esp32_buttons = Part.Compound(solids[50:54])
    esp32_usb = solids[54]
    
    return s_rot, esp32_pcb, esp32_pins, esp32_shield, esp32_ics, esp32_buttons, esp32_usb

def main():
    doc_name = "Caja_ESP32_Expansion"
    doc = App.newDocument(doc_name)
    
    print("\n--- Modelando Carcasa Paramétrica en FreeCAD 1.1 ---")
    base_shape = build_base_box()
    lid_shape = build_top_lid()
    
    print(f"Base Caja: Volumen = {base_shape.Volume:.2f} mm^3 | Es Válido: {base_shape.isValid()}")
    print(f"Tapa Caja: Volumen = {lid_shape.Volume:.2f} mm^3 | Es Válido: {lid_shape.isValid()}")
    
    print("\n--- Modelando Placa de Expansión con Modelos CAD Reales ---")
    exp_pcb, tb_bodies, tb_metals, headers, dc_jack, exp_elec, pcb_screws, lid_screws = build_expansion_board_parts()
    
    print("\n--- Cargando y Posicionando Módulo ESP32 DevKit V1 Real ---")
    esp_full, esp_pcb, esp_pins, esp_shield, esp_ics, esp_buttons, esp_usb = load_and_transform_esp32()
    
    def set_color(obj, color):
        if hasattr(obj, 'ViewObject') and obj.ViewObject is not None:
            obj.ViewObject.ShapeColor = color

    # 1. Base de la Caja
    obj_base = doc.addObject("Part::Feature", "Caja_Base_ESP32")
    obj_base.Shape = base_shape
    obj_base.Label = "Caja Base (Chasis Inferior con Torretas y Ranuras)"
    set_color(obj_base, (0.16, 0.18, 0.22))
    
    # 2. Tapa de la Caja
    obj_lid = doc.addObject("Part::Feature", "Tapa_Caja_ESP32")
    obj_lid.Shape = lid_shape
    obj_lid.Label = "Tapa Superior Desmontable (con Rejillas Convectivas)"
    set_color(obj_lid, (0.22, 0.24, 0.28))
    
    # 3. PCB de la Placa de Expansión
    obj_exp_pcb = doc.addObject("Part::Feature", "Placa_Expansion_PCB")
    obj_exp_pcb.Shape = exp_pcb
    obj_exp_pcb.Label = "Placa de Expansion PCB (Verde Esmeralda FR4)"
    set_color(obj_exp_pcb, (0.06, 0.38, 0.14))
    
    # 4. Borneras de Conexión de Tornillo Reales - Cuerpos
    obj_tb = doc.addObject("Part::Feature", "Borneras_Tornillo_Verdes")
    obj_tb.Shape = tb_bodies
    obj_tb.Label = "Borneras de Tornillo CAD Real (Cuerpo Verde Industrial)"
    set_color(obj_tb, (0.10, 0.68, 0.20))
    
    # 5. Borneras de Tornillo Reales - Tornillos y Metales
    obj_tb_sc = doc.addObject("Part::Feature", "Tornillos_Borneras")
    obj_tb_sc.Shape = tb_metals
    obj_tb_sc.Label = "Tornillos de Borneras CAD Real (Latón/Acero Cincado)"
    set_color(obj_tb_sc, (0.80, 0.75, 0.45))
    
    # 6. Tiras de Headers Hembra Reales
    obj_hdr = doc.addObject("Part::Feature", "Headers_Hembra_Zocalo")
    obj_hdr.Shape = headers
    obj_hdr.Label = "Zócalos Hembra 15 Pines CAD Real para ESP32"
    set_color(obj_hdr, (0.08, 0.08, 0.08))
    
    # 7. Conector Jack DC Real
    obj_dc = doc.addObject("Part::Feature", "Conector_Alimentacion_DC_Jack")
    obj_dc.Shape = dc_jack
    obj_dc.Label = "Conector de Alimentación Jack DC 5.5x2.1mm CAD Real"
    set_color(obj_dc, (0.12, 0.12, 0.12))
    
    # 8. Electrónica de la Placa de Expansión (Regulador, LED, Capacitores)
    obj_exp_el = doc.addObject("Part::Feature", "Componentes_Expansion")
    obj_exp_el.Shape = exp_elec
    obj_exp_el.Label = "Componentes SMD Placa Expansión (Regulador, LED)"
    set_color(obj_exp_el, (0.25, 0.25, 0.25))
    
    # 9. Tornillos de Fijación PCB M3
    obj_sc_pcb = doc.addObject("Part::Feature", "Tornillos_Fijacion_PCB_M3")
    obj_sc_pcb.Shape = pcb_screws
    obj_sc_pcb.Label = "Tornillos M3 Fijación Placa a Torretas"
    set_color(obj_sc_pcb, (0.75, 0.75, 0.78))
    
    # 10. Tornillos de la Tapa M3
    obj_sc_lid = doc.addObject("Part::Feature", "Tornillos_Tapa_M3")
    obj_sc_lid.Shape = lid_screws
    obj_sc_lid.Label = "Tornillos M3 Avellanados de Tapa"
    set_color(obj_sc_lid, (0.78, 0.78, 0.82))
    
    # 11. ESP32 DevKit V1 - PCB Negro Real
    obj_esp_pcb = doc.addObject("Part::Feature", "ESP32_DevKit_PCB")
    obj_esp_pcb.Shape = esp_pcb
    obj_esp_pcb.Label = "ESP32 DevKit V1 CAD Real - PCB Negro Mate"
    set_color(obj_esp_pcb, (0.08, 0.08, 0.10))
    
    # 12. ESP32 DevKit V1 - Pines Macho Dorados
    obj_esp_pins = doc.addObject("Part::Feature", "ESP32_DevKit_Pines")
    obj_esp_pins.Shape = esp_pins
    obj_esp_pins.Label = "ESP32 DevKit V1 CAD Real - Pines Macho Dorados"
    set_color(obj_esp_pins, (0.85, 0.72, 0.28))
    
    # 13. ESP32 DevKit V1 - Blindaje Metálico ESP-WROOM-32
    obj_esp_sh = doc.addObject("Part::Feature", "ESP32_Blindaje_WROOM32")
    obj_esp_sh.Shape = esp_shield
    obj_esp_sh.Label = "ESP32 DevKit V1 CAD Real - Blindaje Metálico WROOM-32"
    set_color(obj_esp_sh, (0.88, 0.90, 0.93))
    
    # 14. ESP32 DevKit V1 - Chips Silicio CP2102 y Pasivos
    obj_esp_ics = doc.addObject("Part::Feature", "ESP32_Chips_CP2102_SMD")
    obj_esp_ics.Shape = esp_ics
    obj_esp_ics.Label = "ESP32 DevKit V1 CAD Real - Chip USB-UART CP2102"
    set_color(obj_esp_ics, (0.15, 0.15, 0.15))
    
    # 15. ESP32 DevKit V1 - Micro Pulsadores EN y BOOT
    obj_esp_btn = doc.addObject("Part::Feature", "ESP32_Pulsadores_EN_BOOT")
    obj_esp_btn.Shape = esp_buttons
    obj_esp_btn.Label = "ESP32 DevKit V1 CAD Real - Botones EN y BOOT"
    set_color(obj_esp_btn, (0.70, 0.70, 0.75))
    
    # 16. ESP32 DevKit V1 - Conector Micro-USB Acero
    obj_esp_usb = doc.addObject("Part::Feature", "ESP32_Conector_MicroUSB")
    obj_esp_usb.Shape = esp_usb
    obj_esp_usb.Label = "ESP32 DevKit V1 CAD Real - Puerto Micro-USB Acero"
    set_color(obj_esp_usb, (0.82, 0.84, 0.88))
    
    doc.recompute()
    
    # EXPORTACIÓN DE ARCHIVOS CAD (FCStd, STEP, STL)
    fcstd_path = os.path.join(MODEL_DIR, "Caja_ESP32_Expansion.FCStd")
    doc.saveAs(fcstd_path)
    print(f"\n[OK] Guardado FreeCAD: {fcstd_path}")
    
    # Ensamblaje Completo Master
    all_objects = [obj_base, obj_lid, obj_exp_pcb, obj_tb, obj_tb_sc, obj_hdr, obj_dc, obj_exp_el, obj_sc_pcb, obj_sc_lid,
                   obj_esp_pcb, obj_esp_pins, obj_esp_sh, obj_esp_ics, obj_esp_btn, obj_esp_usb]
    
    asm_step_path = os.path.join(MODEL_DIR, "Caja_ESP32_Completa_Assembly.step")
    Part.export(all_objects, asm_step_path)
    print(f"[OK] Exportado STEP Ensamblaje Completo: {asm_step_path}")
    
    # Piezas Imprimibles individuales
    base_stl_path = os.path.join(MODEL_DIR, "Caja_Base_ESP32.stl")
    base_shape.exportStl(base_stl_path)
    base_step_path = os.path.join(MODEL_DIR, "Caja_Base_ESP32.step")
    Part.export([obj_base], base_step_path)
    print(f"[OK] Exportado Base STL y STEP")
    
    lid_stl_path = os.path.join(MODEL_DIR, "Tapa_Caja_ESP32.stl")
    lid_shape.exportStl(lid_stl_path)
    lid_step_path = os.path.join(MODEL_DIR, "Tapa_Caja_ESP32.step")
    Part.export([obj_lid], lid_step_path)
    print(f"[OK] Exportado Tapa STL y STEP")
    
    # Ensamblaje Electrónico Real (Placa Expansión Real + ESP32 DevKit V1 Real)
    elec_objects = [obj_exp_pcb, obj_tb, obj_tb_sc, obj_hdr, obj_dc, obj_exp_el, obj_sc_pcb,
                    obj_esp_pcb, obj_esp_pins, obj_esp_sh, obj_esp_ics, obj_esp_btn, obj_esp_usb]
    elec_step_path = os.path.join(MODEL_DIR, "Placa_ESP32_Expansion_Real.step")
    Part.export(elec_objects, elec_step_path)
    print(f"[OK] Exportado Placa Expansión Real + ESP32 Real (STEP): {elec_step_path}")

    # Los STL de ensamblaje completo superan 500 MB y estan ignorados en Git;
    # solo se regeneran bajo demanda pasando --with-huge-stl.
    if "--with-huge-stl" in sys.argv:
        asm_compound = Part.Compound([o.Shape for o in all_objects])
        asm_stl_path = os.path.join(MODEL_DIR, "Caja_ESP32_Completa_Assembly.stl")
        asm_compound.exportStl(asm_stl_path)
        print(f"[OK] Exportado STL Ensamblaje Completo: {asm_stl_path}")

        elec_stl_path = os.path.join(MODEL_DIR, "Placa_ESP32_Expansion_Real.stl")
        Part.Compound([o.Shape for o in elec_objects]).exportStl(elec_stl_path)
        print(f"[OK] Exportado Placa Expansión Real STL: {elec_stl_path}")
            
    print("\n¡Construcción y ensamblaje con modelos CAD 100% reales completados con éxito!")

if __name__ == "__main__":
    main()
