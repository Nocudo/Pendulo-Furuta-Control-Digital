"""
=============================================================================
PROYECTO FINAL - CONTROL DIGITAL: CARCASA PROFESIONAL PARA DRIVER L298N
Script de Modelado Paramétrico y Ensamblaje en FreeCAD 1.1 con Modelo CAD Oficial
=============================================================================
Integra:
1. Modelo CAD STEP Oficial del Driver L298N (User Library-L298n-4.STEP).
2. Chasis Base paramétrico con:
   - 4 separadores cilíndricos H=5.0mm para tornillos M3 en (+-21.50, +-16.50 mm).
   - Ventanas pasantes para borneras de Motor A (-Y), Motor B (+Y) y Alimentación (+X).
   - Rejilla inferior de 6 ranuras de admisión convectiva bajo el disipador.
   - 3 ranuras laterales de ventilación cruzada en la pared del disipador (-X).
   - 4 cajeras cilíndricas para patas antideslizantes.
   - 4 torretas de esquina con orificios para tornillos de tapa M3.
3. Tapa superior ventilada con:
   - Chimenea convectiva con 6 louvers sobre el disipador de aluminio.
   - Ventana de acceso directo a los pines de control lógico (ENA, IN1..IN4, ENB) y jumper 5V.
   - 3 ventanas de acceso vertical con destornillador para los tornillos de las borneras.
   - Pestaña de encaje perimetral macho para cierre firme.
   - 4 avellanados M3 para tornillos de cabeza avellanada (DIN 7991).
4. Exportación completa a FCStd, STEP y STL con verificación manifold.
"""

import os
import sys
import shutil


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

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
CAJA_L298N_DIR = os.path.dirname(SCRIPT_DIR)
WORKSPACE_ROOT = os.path.dirname(os.path.dirname(CAJA_L298N_DIR))
IMPRESION_DIR = os.path.join(WORKSPACE_ROOT, "IMPRESION_3D", "03_Caja_L298N")
STEP_SOURCE_PATH = os.path.join(CAJA_L298N_DIR, "User Library-L298n-4.STEP")

os.makedirs(CAJA_L298N_DIR, exist_ok=True)
os.makedirs(IMPRESION_DIR, exist_ok=True)

def make_fillet_box(length, width, height, radius, pos=App.Vector(0,0,0)):
    """Crea una caja prismática con esquinas redondeadas en el plano horizontal."""
    b1 = Part.makeBox(length - 2*radius, width, height, pos + App.Vector(radius, 0, 0))
    b2 = Part.makeBox(length, width - 2*radius, height, pos + App.Vector(0, radius, 0))
    fused = b1.fuse(b2)
    c1 = Part.makeCylinder(radius, height, pos + App.Vector(radius, radius, 0))
    c2 = Part.makeCylinder(radius, height, pos + App.Vector(length - radius, radius, 0))
    c3 = Part.makeCylinder(radius, height, pos + App.Vector(length - radius, width - radius, 0))
    c4 = Part.makeCylinder(radius, height, pos + App.Vector(radius, width - radius, 0))
    return fused.fuse(c1).fuse(c2).fuse(c3).fuse(c4)

def build_base_box():
    """
    Construye el chasis base paramétrico para el driver L298N.
    Dimensiones exteriores: 65.0 x 55.0 x 26.0 mm.
    Espesor de pared: 2.5 mm, espesor de suelo: 2.0 mm.
    """
    # 1. Cuerpo exterior con esquinas redondeadas R=3.5 mm
    outer = make_fillet_box(65.0, 55.0, 26.0, 3.5, App.Vector(-32.5, -27.5, 0.0))
    
    # 2. Cavidad interior: 60.0 x 50.0 x 25.0 mm desde Z=2.0 mm
    inner = make_fillet_box(60.0, 50.0, 25.0, 2.0, App.Vector(-30.0, -25.0, 2.0))
    base = outer.cut(inner)
    
    # 3. 4 Torretas en esquinas para tornillos M3 de la tapa (radio 3.5 mm en +-28.5, +-23.5)
    cols = []
    col_coords = [(-28.5, -23.5), (28.5, -23.5), (28.5, 23.5), (-28.5, 23.5)]
    for cx, cy in col_coords:
        col = Part.makeCylinder(3.5, 24.0, App.Vector(cx, cy, 2.0), App.Vector(0,0,1))
        hole = Part.makeCylinder(1.4, 25.0, App.Vector(cx, cy, 3.5), App.Vector(0,0,1))
        cols.append(col.cut(hole))
    base = base.fuse(Part.Compound(cols))
    
    # 4. 4 Separadores para la PCB del L298N:
    # Altura H = 5.0 mm (soporte a Z=7.0 mm, protegiendo pines inferiores que llegan a Z=4.39 mm)
    # Coordenadas exactas extraídas del modelo STEP: (+-21.50 mm, +-16.50 mm)
    standoffs = []
    pcb_holes = [(-21.5, -16.5), (-21.5, 16.5), (21.5, -16.5), (21.5, 16.5)]
    for sx, sy in pcb_holes:
        st = Part.makeCylinder(3.2, 5.0, App.Vector(sx, sy, 2.0), App.Vector(0,0,1))
        sh = Part.makeCylinder(1.35, 6.0, App.Vector(sx, sy, 1.5), App.Vector(0,0,1))
        standoffs.append(st.cut(sh))
    base = base.fuse(Part.Compound(standoffs))
    
    # 5. Ventana Frontal Pasante (-Y): Salida cables Motor A
    # Centro en X = 0.0 mm, ancho 16.0 mm, altura desde Z=7.0 mm hasta la parte superior
    cut_mot_a = Part.makeBox(16.0, 10.0, 20.0, App.Vector(-8.0, -33.0, 7.0))
    
    # 6. Ventana Trasera Pasante (+Y): Salida cables Motor B
    # Centro en X = 0.0 mm, ancho 16.0 mm, altura desde Z=7.0 mm hasta la parte superior
    cut_mot_b = Part.makeBox(16.0, 10.0, 20.0, App.Vector(-8.0, 23.0, 7.0))
    
    # 7. Ventana Lateral Pasante (+X): Entrada de Alimentación (12V, GND, 5V)
    # Centro en Y = 0.0 mm, ancho 20.0 mm (Y in [-10, 10]), altura desde Z=7.0 mm hasta parte superior
    cut_pwr = Part.makeBox(10.0, 20.0, 20.0, App.Vector(25.0, -10.0, 7.0))
    
    # 8. Ranuras Laterales de Ventilación Cruzada (-X): Directas al Disipador
    # 3 ranuras horizontales de 26.0 x 2.2 mm en la pared izquierda
    lat_vents = []
    for vz in [10.0, 15.0, 20.0]:
        lv = Part.makeBox(10.0, 26.0, 2.2, App.Vector(-35.0, -13.0, vz))
        lat_vents.append(lv)
        
    # 9. Rejilla Inferior de Ventilación Convectiva (6 ranuras en el suelo bajo el disipador)
    bot_vents = []
    for vy in [-12.5, -7.5, -2.5, 2.5, 7.5, 12.5]:
        bv = Part.makeBox(16.0, 2.5, 4.0, App.Vector(-24.0, vy, -1.0))
        bot_vents.append(bv)
        
    # 10. 4 Cajeras Cilíndricas en el fondo para Patas de Goma Antideslizantes
    feet = []
    for fx, fy in [(-26.0, -21.0), (26.0, -21.0), (26.0, 21.0), (-26.0, 21.0)]:
        f_rec = Part.makeCylinder(4.25, 0.7, App.Vector(fx, fy, -0.1), App.Vector(0,0,1))
        feet.append(f_rec)
        
    base_final = base.cut(cut_mot_a).cut(cut_mot_b).cut(cut_pwr).cut(Part.Compound(lat_vents)).cut(Part.Compound(bot_vents)).cut(Part.Compound(feet))
    return base_final

def build_top_lid():
    """
    Construye la tapa superior con chimenea de escape térmico, ventanas de acceso
    a borneras y conector lógico de control, y avellanados M3.
    """
    col_coords = [(-28.5, -23.5), (28.5, -23.5), (28.5, 23.5), (-28.5, 23.5)]
    
    # Placa superior: 65.0 x 55.0 x 3.0 mm (Z de 26.0 a 29.0 mm)
    lid_plate = make_fillet_box(65.0, 55.0, 3.0, 3.5, App.Vector(-32.5, -27.5, 26.0))
    # Pestaña de encaje perimetral macho (labio inferior de 1.8 mm, Z de 24.2 a 26.0 mm)
    lid_rim = make_fillet_box(59.2, 49.2, 1.8, 1.6, App.Vector(-29.6, -24.6, 24.2))
    lid = lid_plate.fuse(lid_rim)
    
    # 4 Avellanados M3 para tornillos de cabeza avellanada (DIN 7991)
    csk_holes = []
    for cx, cy in col_coords:
        csk = Part.makeCone(3.2, 1.6, 1.7, App.Vector(cx, cy, 27.4), App.Vector(0,0,1))
        shk = Part.makeCylinder(1.6, 6.0, App.Vector(cx, cy, 23.5), App.Vector(0,0,1))
        csk_holes.append(csk.fuse(shk))
        
    # Chimenea convectiva superior sobre el disipador (cámara de alivio + 6 louvers)
    chimney_pocket = Part.makeBox(18.0, 28.0, 2.0, App.Vector(-25.0, -14.0, 24.0))
    top_louvers = []
    for ly in [-12.5, -7.5, -2.5, 2.5, 7.5, 12.5]:
        tl = Part.makeBox(17.0, 2.8, 8.0, App.Vector(-24.5, ly, 23.0))
        top_louvers.append(tl)
        
    # Ventana de acceso directo a pines lógicos (ENA, IN1..IN4, ENB) y jumper 5V
    log_win = Part.makeBox(16.0, 16.0, 8.0, App.Vector(7.0, -8.0, 23.0))
    
    # Ventanas de acceso vertical para destornillador a las borneras
    port_mot_a = Part.makeBox(12.0, 6.0, 8.0, App.Vector(-6.0, -18.5, 23.0))
    port_mot_b = Part.makeBox(12.0, 6.0, 8.0, App.Vector(-6.0, 12.5, 23.0))
    port_pwr = Part.makeBox(6.0, 16.0, 8.0, App.Vector(18.0, -8.0, 23.0))
    
    lid_final = lid.cut(Part.Compound(csk_holes)).cut(chimney_pocket).cut(Part.Compound(top_louvers)).cut(log_win).cut(port_mot_a).cut(port_mot_b).cut(port_pwr)
    return lid_final

def load_and_transform_driver():
    """
    Carga el modelo CAD STEP oficial del driver L298N (15 sólidos),
    lo orienta horizontalmente (PCB paralelo a XY con componentes hacia +Z),
    lo centra en (0,0) y lo eleva para asentar el PCB a Z = 7.0 mm sobre los separadores.
    """
    print(f"Cargando Modelo Oficial L298N desde: {STEP_SOURCE_PATH}")
    shape = Part.read(STEP_SOURCE_PATH)
    s = shape.copy()
    s.rotate(App.Vector(0,0,0), App.Vector(1,0,0), 90)
    s.translate(App.Vector(2.75, 3.76, 9.09))
    solids = s.Solids
    print(f"Driver L298N cargado exitosamente: {len(solids)} cuerpos sólidos.")
    
    # Clasificación por componentes
    mot_a_screws = Part.Compound([solids[0], solids[1]])
    mot_b_screws = Part.Compound([solids[2], solids[3]])
    pwr_screws = Part.Compound([solids[4], solids[5], solids[6]])
    main_board = solids[7]
    chip_pins = Part.Compound([solids[i] for i in range(8, 13)])
    logic_header = solids[13]
    cap_reg = solids[14]
    
    # 4 Tornillos M3 de fijación del PCB a los separadores (cabeza a Z=8.5mm, vástago a Z=4.0mm)
    pcb_screws = []
    pcb_holes = [(-21.5, -16.5), (-21.5, 16.5), (21.5, -16.5), (21.5, 16.5)]
    for sx, sy in pcb_holes:
        head = Part.makeCylinder(2.7, 1.8, App.Vector(sx, sy, 8.2), App.Vector(0,0,1))
        shank = Part.makeCylinder(1.4, 5.0, App.Vector(sx, sy, 3.5), App.Vector(0,0,1))
        sc = head.fuse(shank)
        slot = Part.makeBox(5.4, 0.6, 1.0, App.Vector(sx - 2.7, sy - 0.3, 9.0))
        pcb_screws.append(sc.cut(slot))
    comp_pcb_screws = Part.Compound(pcb_screws)
    
    return mot_a_screws, mot_b_screws, pwr_screws, main_board, chip_pins, logic_header, cap_reg, comp_pcb_screws

def build_lid_screws():
    """Genera 4 tornillos avellanados M3 para el cierre de la tapa superior."""
    col_coords = [(-28.5, -23.5), (28.5, -23.5), (28.5, 23.5), (-28.5, 23.5)]
    screws = []
    for cx, cy in col_coords:
        head = Part.makeCone(3.0, 1.45, 1.5, App.Vector(cx, cy, 27.5), App.Vector(0,0,1))
        shank = Part.makeCylinder(1.45, 10.0, App.Vector(cx, cy, 17.5), App.Vector(0,0,1))
        sc = head.fuse(shank)
        slot = Part.makeBox(6.0, 0.6, 1.0, App.Vector(cx - 3.0, cy - 0.3, 28.2))
        screws.append(sc.cut(slot))
    return Part.Compound(screws)

def main():
    doc_name = "Caja_L298N"
    doc = App.newDocument(doc_name)
    
    print("\n--- 1. Modelando Carcasa Paramétrica para Driver L298N ---")
    base_shape = build_base_box()
    lid_shape = build_top_lid()
    lid_screws = build_lid_screws()
    
    print(f"Base Caja: Volumen = {base_shape.Volume:.2f} mm^3 | Válido: {base_shape.isValid()}")
    print(f"Tapa Caja: Volumen = {lid_shape.Volume:.2f} mm^3 | Válido: {lid_shape.isValid()}")
    
    print("\n--- 2. Cargando y Orientando Driver L298N Oficial ---")
    mot_a_sc, mot_b_sc, pwr_sc, board_body, chip_pins, logic_hdr, cap_reg, pcb_screws = load_and_transform_driver()
    
    def set_color(obj, color):
        if hasattr(obj, 'ViewObject') and obj.ViewObject is not None:
            obj.ViewObject.ShapeColor = color
            
    # Añadir a documento FreeCAD con colores PBR técnicos
    o_base = doc.addObject("Part::Feature", "Caja_Base_L298N")
    o_base.Shape = base_shape
    o_base.Label = "Caja Base (Chasis con Rejillas Térmicas y Ventanas Pasantes)"
    set_color(o_base, (0.18, 0.20, 0.25))
    
    o_lid = doc.addObject("Part::Feature", "Tapa_Caja_L298N")
    o_lid.Shape = lid_shape
    o_lid.Label = "Tapa Superior Ventilada (Chimenea Louvers y Ventana de Control)"
    set_color(o_lid, (0.95, 0.42, 0.12)) # Naranja industrial de alto contraste
    
    o_lscrews = doc.addObject("Part::Feature", "Tornillos_Tapa_M3")
    o_lscrews.Shape = lid_screws
    o_lscrews.Label = "Tornillos M3 Avellanados de Tapa"
    set_color(o_lscrews, (0.78, 0.78, 0.82))
    
    # Objetos del Driver
    o_board = doc.addObject("Part::Feature", "L298N_Cuerpo_Principal")
    o_board.Shape = board_body
    o_board.Label = "L298N - PCB FR4 Verde, Disipador y Borneras"
    set_color(o_board, (0.12, 0.45, 0.22))
    
    o_mota = doc.addObject("Part::Feature", "L298N_Tornillos_Motor_A")
    o_mota.Shape = mot_a_sc
    o_mota.Label = "L298N - Tornillos Bornera Motor A (Salida -Y)"
    set_color(o_mota, (0.86, 0.87, 0.90))
    
    o_motb = doc.addObject("Part::Feature", "L298N_Tornillos_Motor_B")
    o_motb.Shape = mot_b_sc
    o_motb.Label = "L298N - Tornillos Bornera Motor B (Salida +Y)"
    set_color(o_motb, (0.86, 0.87, 0.90))
    
    o_pwr = doc.addObject("Part::Feature", "L298N_Tornillos_Power")
    o_pwr.Shape = pwr_sc
    o_pwr.Label = "L298N - Tornillos Bornera Alimentación (Entrada +X)"
    set_color(o_pwr, (0.86, 0.87, 0.90))
    
    o_cpins = doc.addObject("Part::Feature", "L298N_Pines_Chip")
    o_cpins.Shape = chip_pins
    o_cpins.Label = "L298N - Terminales Chip hacia Disipador"
    set_color(o_cpins, (0.88, 0.75, 0.30))
    
    o_lhdr = doc.addObject("Part::Feature", "L298N_Header_Control_Logico")
    o_lhdr.Shape = logic_hdr
    o_lhdr.Label = "L298N - Pines Control ENA, IN1..IN4, ENB"
    set_color(o_lhdr, (0.10, 0.10, 0.10))
    
    o_creg = doc.addObject("Part::Feature", "L298N_Capacitor_Regulador")
    o_creg.Shape = cap_reg
    o_creg.Label = "L298N - Condensador de Filtro y Regulador 78M05"
    set_color(o_creg, (0.15, 0.15, 0.15))
    
    o_pscrews = doc.addObject("Part::Feature", "Tornillos_Fijacion_PCB_M3")
    o_pscrews.Shape = pcb_screws
    o_pscrews.Label = "Tornillos M3 Fijación Placa a Separadores"
    set_color(o_pscrews, (0.75, 0.75, 0.80))
    
    doc.recompute()
    
    # EXPORTACIÓN DE ARCHIVOS CAD MAESTROS
    print("\n--- 3. Exportando Archivos CAD a cad/04_caja_driver_l298n e IMPRESION_3D ---")
    
    fcstd_path = os.path.join(CAJA_L298N_DIR, "Caja_L298N.FCStd")
    doc.saveAs(fcstd_path)
    print(f"[OK] FreeCAD Documento: {fcstd_path}")
    
    all_objects = [o_base, o_lid, o_lscrews, o_board, o_mota, o_motb, o_pwr, o_cpins, o_lhdr, o_creg, o_pscrews]
    asm_step_path = os.path.join(CAJA_L298N_DIR, "Caja_L298N_Completa_Assembly.step")
    Part.export(all_objects, asm_step_path)
    print(f"[OK] Ensamblaje Completo STEP: {asm_step_path}")
    
    # Piezas imprimibles 3D individuales
    base_stl_path = os.path.join(CAJA_L298N_DIR, "Caja_Base_L298N.stl")
    base_shape.exportStl(base_stl_path)
    base_step_path = os.path.join(CAJA_L298N_DIR, "Caja_Base_L298N.step")
    Part.export([o_base], base_step_path)
    print(f"[OK] Caja Base STL y STEP")
    
    lid_stl_path = os.path.join(CAJA_L298N_DIR, "Tapa_Caja_L298N.stl")
    lid_shape.exportStl(lid_stl_path)
    lid_step_path = os.path.join(CAJA_L298N_DIR, "Tapa_Caja_L298N.step")
    Part.export([o_lid], lid_step_path)
    print(f"[OK] Tapa Superior STL y STEP")
    
    # Subensamblaje del Driver L298N
    driver_objects = [o_board, o_mota, o_motb, o_pwr, o_cpins, o_lhdr, o_creg, o_pscrews]
    driver_step_path = os.path.join(CAJA_L298N_DIR, "Driver_L298N_Real.step")
    Part.export(driver_objects, driver_step_path)
    print(f"[OK] Driver L298N STEP")

    # Los STL de ensamblaje completo con PCB real superan los 250 MB y estan ignorados en Git;
    # solo se regeneran bajo demanda pasando --with-huge-stl.
    if "--with-huge-stl" in sys.argv:
        driver_stl_compound = Part.Compound([board_body, mot_a_sc, mot_b_sc, pwr_sc, logic_hdr, cap_reg])
        driver_stl_path = os.path.join(CAJA_L298N_DIR, "Driver_L298N_Real.stl")
        driver_stl_compound.exportStl(driver_stl_path)
        print(f"[OK] Driver L298N STL")

        asm_stl_compound = Part.Compound([base_shape, lid_shape, board_body, mot_a_sc, mot_b_sc, pwr_sc, logic_hdr, cap_reg, lid_screws])
        asm_stl_path = os.path.join(CAJA_L298N_DIR, "Caja_L298N_Completa_Assembly.stl")
        asm_stl_compound.exportStl(asm_stl_path)
        print(f"[OK] Ensamblaje Completo STL: {asm_stl_path}")
    
    # Sincronizar piezas imprimibles (.stl y .step) a IMPRESION_3D/03_Caja_L298N
    shutil.copy2(base_stl_path, os.path.join(IMPRESION_DIR, "Caja_Base_L298N.stl"))
    shutil.copy2(base_step_path, os.path.join(IMPRESION_DIR, "Caja_Base_L298N.step"))
    shutil.copy2(lid_stl_path, os.path.join(IMPRESION_DIR, "Tapa_Caja_L298N.stl"))
    shutil.copy2(lid_step_path, os.path.join(IMPRESION_DIR, "Tapa_Caja_L298N.step"))
    print(f"[OK] Copia sincronizada en IMPRESION_3D/03_Caja_L298N/")
    
    print("\n¡Construcción y ensamblaje de la Caja L298N completados exitosamente!")

if __name__ == "__main__":
    main()
