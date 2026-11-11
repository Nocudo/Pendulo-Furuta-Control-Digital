import os
import math
import shutil
import FreeCAD as App
import Part
import MeshPart

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
MODEL_DIR = os.path.dirname(SCRIPT_DIR)
ROOT_DIR = os.path.dirname(os.path.dirname(MODEL_DIR))
IMPRESION_FURUTA_DIR = os.path.join(ROOT_DIR, "IMPRESION_3D", "02_Pendulo_Furuta")

FCSTD_PATH = os.path.join(MODEL_DIR, "Motor_22CL_AS5600.FCStd")

# PCB alignment offsets from AS5600 STEP model
PCB_CX = 0.2642
PCB_CY = -0.1237

# Exact M3 mounting hole coordinates (from Encoder_AS5600 PCB)
M3_HOLES = [
    (-7.816, -8.254),
    (-7.816,  7.976),
    ( 8.344, -8.254),
    ( 8.344,  7.976),
]


def make_rounded_rect_wire(cx, cy, hx, hy, r, z):
    """Create a closed horizontal XY wire centered at (cx, cy) at height z with corner fillet radius r."""
    r = min(r, hx - 0.05, hy - 0.05)
    pts = [
        App.Vector(cx - hx + r, cy - hy, z),
        App.Vector(cx + hx - r, cy - hy, z),
        App.Vector(cx + hx, cy - hy + r, z),
        App.Vector(cx + hx, cy + hy - r, z),
        App.Vector(cx + hx - r, cy + hy, z),
        App.Vector(cx - hx + r, cy + hy, z),
        App.Vector(cx - hx, cy + hy - r, z),
        App.Vector(cx - hx, cy - hy + r, z),
    ]
    edges = []
    # Bottom edge
    edges.append(Part.makeLine(pts[0], pts[1]))
    # Bottom-right arc
    edges.append(Part.Arc(
        pts[1],
        App.Vector(cx + hx - r + r * 0.70710678, cy - hy + r - r * 0.70710678, z),
        pts[2],
    ).toShape())
    # Right edge
    edges.append(Part.makeLine(pts[2], pts[3]))
    # Top-right arc
    edges.append(Part.Arc(
        pts[3],
        App.Vector(cx + hx - r + r * 0.70710678, cy + hy - r + r * 0.70710678, z),
        pts[4],
    ).toShape())
    # Top edge
    edges.append(Part.makeLine(pts[4], pts[5]))
    # Top-left arc
    edges.append(Part.Arc(
        pts[5],
        App.Vector(cx - hx + r - r * 0.70710678, cy + hy - r + r * 0.70710678, z),
        pts[6],
    ).toShape())
    # Left edge
    edges.append(Part.makeLine(pts[6], pts[7]))
    # Bottom-left arc
    edges.append(Part.Arc(
        pts[7],
        App.Vector(cx - hx + r - r * 0.70710678, cy - hy + r - r * 0.70710678, z),
        pts[0],
    ).toShape())
    return Part.Wire(edges)


def build_acoplador_dfam():
    """
    DfAM zero-support AS5600 motor coupler (28 x 28 mm, Z = -64.60 .. -42.00):
    - Flat bottom at Z=-64.60 (PCB side down on build plate).
    - Replaced horizontal Z=-62.75 / Z=-58.50 overhangs with a single >=43 deg
      self-supporting conical/pyramidal taper up to Z=-52.50.
    - Replaced horizontal wire slot roofs with 45 deg gabled arches.
    - Blind M3 pilot holes terminate in 45 deg drill-point cones below Z=-52.00.
    """
    # 1. Outer 28x28 rounded square block (Z = -64.60 .. -52.00)
    w_outer = make_rounded_rect_wire(0.0, 0.0, 14.0, 14.0, 3.5, -64.60)
    block = Part.Face(w_outer).extrude(App.Vector(0, 0, 12.60))

    # 2. Upper motor-body collar (Z = -52.00 .. -42.00, OD 26.5mm)
    collar = Part.makeCylinder(13.25, 10.0, App.Vector(0, 0, -52.00), App.Vector(0, 0, 1))
    body = block.fuse(collar)

    # 3. Cutout: Motor housing slip bore (Ø22.16 mm, Z = -52.00 .. -42.00) + top lead-in chamfer
    mot_bore = Part.makeCylinder(11.08, 11.0, App.Vector(0, 0, -52.00), App.Vector(0, 0, 1))
    mot_chamfer = Part.makeCone(11.08, 11.68, 0.65, App.Vector(0, 0, -42.60), App.Vector(0, 0, 1))
    body = body.cut(mot_bore).cut(mot_chamfer)

    # 4. Cutout: PCB bottom square recess (23.5 x 23.5 mm, Z = -65.00 .. -62.75)
    # 5. Cutout: <=40-deg self-supporting tapered SMD + magnet chamber (Z = -62.75 .. -51.20)
    # Using exact coincident Z=-62.75 and ending above Z=-52.00 removes ALL horizontal ceiling faces!
    w_pcb = make_rounded_rect_wire(PCB_CX, PCB_CY, 11.75, 11.75, 2.00, -65.00)
    pcb_pocket = Part.Face(w_pcb).extrude(App.Vector(0, 0, 2.25))
    w_dome_bot = make_rounded_rect_wire(PCB_CX, PCB_CY, 11.75, 11.75, 2.00, -62.75)
    w_dome_top = make_rounded_rect_wire(0.0, 0.0, 5.80, 5.80, 5.70, -49.50)
    dome_loft = Part.makeLoft([w_dome_bot, w_dome_top], True, False, False)
    smd_cavity = pcb_pocket.fuse(dome_loft)
    body = body.cut(smd_cavity)

    # 6. Cutout: 2x 38-deg gabled wire exit slots (+Y and -Y)
    # Span X in [-6.24, +6.76] (width 13.0 mm centered at PCB_CX), apex Z = -52.80 (< 38 deg slopes)
    x0, x1, xm = -6.24, 6.76, 0.26
    z_bot, z_eave, z_peak = -65.00, -61.20, -52.80
    for y_start, y_span in [(-15.0, 7.63), (7.37, 7.63)]:
        slot = Part.Face(Part.makePolygon([
            App.Vector(x0, y_start, z_bot),
            App.Vector(x1, y_start, z_bot),
            App.Vector(x1, y_start, z_eave),
            App.Vector(xm, y_start, z_peak),
            App.Vector(x0, y_start, z_eave),
            App.Vector(x0, y_start, z_bot),
        ])).extrude(App.Vector(0, y_span, 0))
        body = body.cut(slot)

    # 7. Cutout: 4x M3 self-tapping pilot holes (R=1.30 mm) with sharp 35-deg pointed tip below Z=-52.00
    for hx, hy in M3_HOLES:
        cyl_m3 = Part.makeCylinder(1.30, 8.85, App.Vector(hx, hy, -63.00), App.Vector(0, 0, 1))
        tip_m3 = Part.makeCone(1.30, 0.0, 1.85, App.Vector(hx, hy, -54.15), App.Vector(0, 0, 1))
        body = body.cut(cyl_m3.fuse(tip_m3))

    return body.removeSplitter()


def build_portaiman_dfam():
    """Magnet holder Ø7.5 x 7.5 mm (Z = -60.50 .. -53.00), zero supports when printed magnet-side down."""
    cyl = Part.makeCylinder(3.75, 7.50, App.Vector(0, 0, -60.50), App.Vector(0, 0, 1))
    mag_pocket = Part.makeCylinder(2.25, 2.10, App.Vector(0, 0, -60.60), App.Vector(0, 0, 1))
    shaft_bore = Part.makeCylinder(1.03, 5.70, App.Vector(0, 0, -58.50), App.Vector(0, 0, 1))
    # Small 0.3mm chamfer at shaft entry Z=-53.0
    shaft_chamfer = Part.makeCone(1.03, 1.38, 0.35, App.Vector(0, 0, -53.30), App.Vector(0, 0, 1))
    return cyl.cut(mag_pocket).cut(shaft_bore).cut(shaft_chamfer).removeSplitter()


def main():
    # Load static motor/encoder reference parts from existing Motor_22CL_AS5600.FCStd
    ref_names = [
        "Gearbox_3501PG",
        "Centering_Boss",
        "Output_Shaft_D4mm",
        "Motor_22CL_Housing",
        "Rear_Shaft_D2mm",
        "Iman_Diametral_AS5600",
        "Encoder_AS5600",
        "Tornillos_Fijacion_PCB_AS5600",
    ]
    ref_doc = App.openDocument(FCSTD_PATH)
    ref_shapes = {}
    for rn in ref_names:
        obj = ref_doc.getObject(rn)
        if obj is None or obj.Shape.isNull():
            raise RuntimeError(f"Missing reference object {rn} in {FCSTD_PATH}")
        ref_shapes[rn] = (obj.Label, obj.Shape.copy())
    App.closeDocument(ref_doc.Name)

    acop_shape = build_acoplador_dfam()
    port_shape = build_portaiman_dfam()

    assert acop_shape.isValid(), "Acoplador shape is not valid!"
    assert port_shape.isValid(), "Portaiman shape is not valid!"

    # Automated interference / clash checks (non-thread bodies MUST be 0.0000 mm^3)
    clash_checks = [
        ("Acoplador vs Encoder_AS5600", acop_shape, ref_shapes["Encoder_AS5600"][1]),
        ("Acoplador vs Motor_22CL_Housing", acop_shape, ref_shapes["Motor_22CL_Housing"][1]),
        ("Acoplador vs Portaiman_AS5600", acop_shape, port_shape),
        ("Acoplador vs Rear_Shaft_D2mm", acop_shape, ref_shapes["Rear_Shaft_D2mm"][1]),
        ("Portaiman vs Rear_Shaft_D2mm", port_shape, ref_shapes["Rear_Shaft_D2mm"][1]),
        ("Portaiman vs Encoder_AS5600", port_shape, ref_shapes["Encoder_AS5600"][1]),
    ]
    print("=== Motor Coupler DfAM Clash Verification ===")
    for cname, s1, s2 in clash_checks:
        vol = s1.common(s2).Volume
        print(f"  {cname:36s}: {vol:.4f} mm^3")
        if vol > 1e-3:
            raise RuntimeError(f"Clash detected in {cname}: {vol:.4f} mm^3")

    doc = App.newDocument("Motor_22CL_AS5600")
    all_objs = []
    for rn in ref_names[:5]:
        lbl, shp = ref_shapes[rn]
        o = doc.addObject("Part::Feature", rn)
        o.Label = lbl
        o.Shape = shp
        all_objs.append(o)

    o_acop = doc.addObject("Part::Feature", "Acoplador_Motor_AS5600")
    o_acop.Label = "Acoplador Motor AS5600 (DfAM Sin Soportes)"
    o_acop.Shape = acop_shape
    all_objs.append(o_acop)

    o_port = doc.addObject("Part::Feature", "Portaiman_AS5600")
    o_port.Label = "Portaiman Eje Trasero 2mm (Sin Soportes)"
    o_port.Shape = port_shape
    all_objs.append(o_port)

    for rn in ref_names[5:]:
        lbl, shp = ref_shapes[rn]
        o = doc.addObject("Part::Feature", rn)
        o.Label = lbl
        o.Shape = shp
        all_objs.append(o)

    doc.recompute()

    # Save & export across cad/02_motor_namiki_as5600 and IMPRESION_3D/02_Pendulo_Furuta
    doc.saveAs(FCSTD_PATH)
    Part.export(all_objs, os.path.join(MODEL_DIR, "Motor_22CL_AS5600_Assembly.step"))
    MeshPart.meshFromShape(
        Shape=Part.Compound([o.Shape for o in all_objs]),
        LinearDeflection=0.05,
        AngularDeflection=0.1,
    ).write(os.path.join(MODEL_DIR, "Motor_22CL_AS5600_Assembly.stl"))

    os.makedirs(IMPRESION_FURUTA_DIR, exist_ok=True)
    for name, obj, shp in [
        ("Acoplador_Motor_AS5600", o_acop, acop_shape),
        ("Portaiman_AS5600", o_port, port_shape),
    ]:
        step_out = os.path.join(MODEL_DIR, f"{name}.step")
        stl_out = os.path.join(MODEL_DIR, f"{name}.stl")
        Part.export([obj], step_out)
        MeshPart.meshFromShape(Shape=shp, LinearDeflection=0.03, AngularDeflection=0.08).write(stl_out)
        shutil.copy2(step_out, os.path.join(IMPRESION_FURUTA_DIR, f"{name}.step"))
        shutil.copy2(stl_out, os.path.join(IMPRESION_FURUTA_DIR, f"{name}.stl"))
        print(f"Exported & synced {name} (Vol={shp.Volume:.2f} mm^3)")

    App.closeDocument(doc.Name)
    print("DONE: Motor coupler DfAM build verified & synced!")


if __name__ == "__main__":
    main()
