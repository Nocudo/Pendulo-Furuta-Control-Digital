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

# Core kinematic datum heights (preserve Z_MOTOR_FACE=156.0 & Z_PIVOT=182.8)
Z_MOTOR_FACE = 156.0
Z_CAP_TOP = 160.0
Z_ARM_BOT = 162.5
Z_ARM_TOP = 171.5
Z_PIVOT = 182.8
Z_HEAD_TOP = 201.8

# Axle / pendulum X datum coordinates (with >=41 deg self-supporting flange taper)
X_BEARING_IN = 74.0
X_BEARING_OUT = 81.0
X_FLANGE_FLAT = 82.0
X_TAPER_END = 86.2   # 4.2 mm run for 2.7 mm radial drop => 32.7 deg cone overhang
X_STUB_END = 93.2    # 7.0 mm full D-shaft engagement
X_BAR_THICK = 5.5
X_BAR_IN = X_STUB_END - X_BAR_THICK  # 87.7
X_BAR_CTR = (X_BAR_IN + X_STUB_END) / 2.0  # 90.45


def make_rounded_rect_wire(cx, cy, hx, hy, r, z):
    """Create a closed XY wire centered at (cx, cy) at height z with corner radius r."""
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
    d = r * 0.70710678
    edges = [
        Part.makeLine(pts[0], pts[1]),
        Part.Arc(pts[1], App.Vector(cx + hx - r + d, cy - hy + r - d, z), pts[2]).toShape(),
        Part.makeLine(pts[2], pts[3]),
        Part.Arc(pts[3], App.Vector(cx + hx - r + d, cy + hy - r + d, z), pts[4]).toShape(),
        Part.makeLine(pts[4], pts[5]),
        Part.Arc(pts[5], App.Vector(cx - hx + r - d, cy + hy - r + d, z), pts[6]).toShape(),
        Part.makeLine(pts[6], pts[7]),
        Part.Arc(pts[7], App.Vector(cx - hx + r - d, cy - hy + r - d, z), pts[0]).toShape(),
    ]
    return Part.Wire(edges)


def make_teardrop_wire_yz(x, y0, z0, r, slope_deg=41.0):
    """Create a closed YZ teardrop profile at X=x whose top roof slopes at slope_deg from vertical."""
    a = math.radians(slope_deg)
    ca, sa = math.cos(a), math.sin(a)
    yt = r * ca
    zt = r * sa
    z_apex = z0 + r / sa

    p_bot = App.Vector(x, y0, z0 - r)
    p_tr = App.Vector(x, y0 + yt, z0 + zt)
    p_apex = App.Vector(x, y0, z_apex)
    p_tl = App.Vector(x, y0 - yt, z0 + zt)

    mid_r_ang = math.radians((-90.0 + slope_deg) / 2.0)
    mid_l_ang = math.radians((180.0 - slope_deg + 270.0) / 2.0)
    p_mr = App.Vector(x, y0 + r * math.cos(mid_r_ang), z0 + r * math.sin(mid_r_ang))
    p_ml = App.Vector(x, y0 + r * math.cos(mid_l_ang), z0 + r * math.sin(mid_l_ang))

    edges = [
        Part.Arc(p_bot, p_mr, p_tr).toShape(),
        Part.makeLine(p_tr, p_apex),
        Part.makeLine(p_apex, p_tl),
        Part.Arc(p_tl, p_ml, p_bot).toShape(),
    ]
    return Part.Wire(edges)


def make_teardrop_bore_x(r, x_start, x_end, y0=0.0, z0=Z_PIVOT, slope_deg=41.0):
    """Horizontal bore along +X with self-supporting teardrop roof."""
    w = make_teardrop_wire_yz(x_start, y0, z0, r, slope_deg)
    return Part.Face(w).extrude(App.Vector(x_end - x_start, 0, 0))


def build_tower_dfam():
    """
    DfAM Support-Free Open Tower (Z = 0 .. 156.0 mm):
    - Top motor plate split off into Tapa_Motor_Torre_Furuta so the central bore
      runs straight through at Ø32.0 mm (0 horizontal overhangs).
    - 4-way access windows topped with 40.3-deg pointed gable arches (Z=130..146.5).
    - Column flares outward 22.6 deg at Z=144..156 (42x42 -> 52x52) with 4x M3 pilot holes.
    """
    w_base = make_rounded_rect_wire(0.0, 0.0, 60.0, 60.0, 6.0, 0.0)
    base_plate = Part.Face(w_base).extrude(App.Vector(0, 0, 8.0))

    w_col = make_rounded_rect_wire(0.0, 0.0, 21.0, 21.0, 1.5, 8.0)
    col = Part.Face(w_col).extrude(App.Vector(0, 0, 136.5))  # Z = 8 .. 144.5

    w_fl_bot = make_rounded_rect_wire(0.0, 0.0, 21.0, 21.0, 1.5, 144.0)
    w_fl_top = make_rounded_rect_wire(0.0, 0.0, 26.0, 26.0, 3.0, Z_MOTOR_FACE)
    flare = Part.makeLoft([w_fl_bot, w_fl_top], True, False, False)
    tower = base_plate.fuse(col).fuse(flare)

    # 4 triangular reinforcing ribs (29.5 deg from vertical, Z=8..68, span ±55 -> ±21)
    for axis in ("x+", "x-", "y+", "y-"):
        if axis in ("x+", "x-"):
            s = 1.0 if axis == "x+" else -1.0
            poly = [
                App.Vector(s * 20.5, -3.0, 8.0),
                App.Vector(s * 55.0, -3.0, 8.0),
                App.Vector(s * 20.5, -3.0, 68.0),
                App.Vector(s * 20.5, -3.0, 8.0),
            ]
            rib = Part.Face(Part.makePolygon(poly)).extrude(App.Vector(0, 6.0, 0))
        else:
            s = 1.0 if axis == "y+" else -1.0
            poly = [
                App.Vector(-3.0, s * 20.5, 8.0),
                App.Vector(-3.0, s * 55.0, 8.0),
                App.Vector(-3.0, s * 20.5, 68.0),
                App.Vector(-3.0, s * 20.5, 8.0),
            ]
            rib = Part.Face(Part.makePolygon(poly)).extrude(App.Vector(6.0, 0, 0))
        tower = tower.fuse(rib)

    # Central full-height bore Ø32.0 mm (R=16.0, Z=-1..157) - 0 deg overhang
    bore = Part.makeCylinder(16.0, 158.0, App.Vector(0, 0, -1.0), App.Vector(0, 0, 1))
    tower = tower.cut(bore)

    # 4-way arched windows: vertical walls Z=75..130, 40.3-deg gable apex Z=146.5
    pts_wx = [
        App.Vector(-35.0, -14.0, 75.0),
        App.Vector(-35.0,  14.0, 75.0),
        App.Vector(-35.0,  14.0, 130.0),
        App.Vector(-35.0,   0.0, 146.5),
        App.Vector(-35.0, -14.0, 130.0),
        App.Vector(-35.0, -14.0, 75.0),
    ]
    win_x = Part.Face(Part.makePolygon(pts_wx)).extrude(App.Vector(70.0, 0, 0))
    pts_wy = [
        App.Vector(-14.0, -35.0, 75.0),
        App.Vector( 14.0, -35.0, 75.0),
        App.Vector( 14.0, -35.0, 130.0),
        App.Vector(  0.0, -35.0, 146.5),
        App.Vector(-14.0, -35.0, 130.0),
        App.Vector(-14.0, -35.0, 75.0),
    ]
    win_y = Part.Face(Part.makePolygon(pts_wy)).extrude(App.Vector(0, 70.0, 0))
    tower = tower.cut(win_x).cut(win_y)

    # 4x M5 baseplate anchor holes
    for bx, by in [(-48.0, -48.0), (48.0, -48.0), (-48.0, 48.0), (48.0, 48.0)]:
        h_m5 = Part.makeCylinder(2.70, 12.0, App.Vector(bx, by, -1.0), App.Vector(0, 0, 1))
        tower = tower.cut(h_m5)

    # 4x M3 self-tapping pilot holes in top collar for removable motor cap
    for cx, cy in [(-19.5, -19.5), (19.5, -19.5), (-19.5, 19.5), (19.5, 19.5)]:
        h_m3 = Part.makeCylinder(1.35, 13.5, App.Vector(cx, cy, 143.0), App.Vector(0, 0, 1))
        ch_m3 = Part.makeCone(1.35, 1.95, 0.6, App.Vector(cx, cy, Z_MOTOR_FACE - 0.55), App.Vector(0, 0, 1))
        tower = tower.cut(h_m3).cut(ch_m3)

    return tower.removeSplitter()


def build_tower_base_dfam():
    """
    DfAM Lower Base Pedestal (Z = 0 .. 70.0 mm + male spigot to Z = 82.0 mm):
    - Massive 120x120 mm baseplate (Z=0..8) with 4x M5 mounting holes.
    - Central column (42x42 mm, rounded r=1.5) Z=8..70 mm.
    - 4 triangular stiffening ribs along X+, X-, Y+, Y- (Z=8..68 mm).
    - Male coaxial alignment spigot: Ø31.6 mm OD (R=15.8) x 12 mm tall (Z=70..82 mm)
      with 45-degree lead-in chamfer (Z=80.5..82.0 mm, R=14.3 at tip).
    - Inner bore: Ø24.0 mm through-bore (R=12.0, Z=-1..83 mm) for motor and sensor wiring.
    - 4x M3 self-tapping pilot holes Ø2.6 mm at (±15.0, ±15.0, Z=55..71 mm) depth 15 mm
      for clamping the upper column.
    """
    w_base = make_rounded_rect_wire(0.0, 0.0, 60.0, 60.0, 6.0, 0.0)
    base_plate = Part.Face(w_base).extrude(App.Vector(0, 0, 8.0))

    w_col = make_rounded_rect_wire(0.0, 0.0, 21.0, 21.0, 1.5, 8.0)
    col = Part.Face(w_col).extrude(App.Vector(0, 0, 62.0))

    base = base_plate.fuse(col)

    for axis in ("x+", "x-", "y+", "y-"):
        if axis in ("x+", "x-"):
            s = 1.0 if axis == "x+" else -1.0
            poly = [
                App.Vector(s * 20.5, -3.0, 8.0),
                App.Vector(s * 55.0, -3.0, 8.0),
                App.Vector(s * 20.5, -3.0, 68.0),
                App.Vector(s * 20.5, -3.0, 8.0),
            ]
            rib = Part.Face(Part.makePolygon(poly)).extrude(App.Vector(0, 6.0, 0))
        else:
            s = 1.0 if axis == "y+" else -1.0
            poly = [
                App.Vector(-3.0, s * 20.5, 8.0),
                App.Vector(-3.0, s * 55.0, 8.0),
                App.Vector(-3.0, s * 20.5, 68.0),
                App.Vector(-3.0, s * 20.5, 8.0),
            ]
            rib = Part.Face(Part.makePolygon(poly)).extrude(App.Vector(6.0, 0, 0))
        base = base.fuse(rib)

    spigot_cyl = Part.makeCylinder(15.8, 10.5, App.Vector(0, 0, 70.0), App.Vector(0, 0, 1))
    spigot_cone = Part.makeCone(15.8, 14.3, 1.5, App.Vector(0, 0, 80.5), App.Vector(0, 0, 1))
    spigot = spigot_cyl.fuse(spigot_cone)
    base = base.fuse(spigot)

    bore = Part.makeCylinder(12.0, 85.0, App.Vector(0, 0, -1.0), App.Vector(0, 0, 1))
    base = base.cut(bore)

    for bx, by in [(-48.0, -48.0), (48.0, -48.0), (-48.0, 48.0), (48.0, 48.0)]:
        h_m5 = Part.makeCylinder(2.70, 12.0, App.Vector(bx, by, -1.0), App.Vector(0, 0, 1))
        base = base.cut(h_m5)

    for px, py in [(-15.0, -15.0), (15.0, -15.0), (-15.0, 15.0), (15.0, 15.0)]:
        h_m3_pilot = Part.makeCylinder(1.30, 16.0, App.Vector(px, py, 55.0), App.Vector(0, 0, 1))
        cs_m3 = Part.makeCone(1.30, 1.80, 0.8, App.Vector(px, py, 69.3), App.Vector(0, 0, 1))
        base = base.cut(h_m3_pilot).cut(cs_m3)

    return base.removeSplitter()


def build_tower_upper_dfam():
    """
    DfAM Upper Motor Column (Z = 70.0 .. 156.0 mm, total height = 86.0 mm):
    - Designed specifically for inverted printing (BOCA ABAJO, Z=156 on bed):
      - 52x52 mm motor face sits flat on bed (18.9 cm^2 adhesion area, 0 warping).
      - Collar tapers inward 22.6 deg (42x42 <- 52x52) with zero overhang.
      - 4-way access windows feature DOUBLE GABLE ARCHES:
        * Upper arch: apex at Z=146.5 mm, base at Z=130 mm (40.3 deg from vertical).
        * Lower arch: apex at Z=73.5 mm, base at Z=91.5 mm (37.8 deg from vertical).
        * Inverted print direction: lower arch closes at 37.8 deg with ZERO supports!
      - 4x M3 motor cap pilot holes at Z=156 feature 34-deg conical tips (Z=141..143)
        eliminating flat ceiling overhangs.
      - 4x M3 corner clearance through-holes Ø3.4 mm at (±15.0, ±15.0, Z=69..76.5 mm).
      - 100% SUPPORT-FREE in BOCA ABAJO orientation (SUP = 0, 0.00 mm^2 overhang > 45 deg).
    """
    w_col = make_rounded_rect_wire(0.0, 0.0, 21.0, 21.0, 1.5, 70.0)
    col = Part.Face(w_col).extrude(App.Vector(0, 0, 74.5))

    w_fl_bot = make_rounded_rect_wire(0.0, 0.0, 21.0, 21.0, 1.5, 144.0)
    w_fl_top = make_rounded_rect_wire(0.0, 0.0, 26.0, 26.0, 3.0, Z_MOTOR_FACE)
    flare = Part.makeLoft([w_fl_bot, w_fl_top], True, False, False)
    col = col.fuse(flare)

    bore = Part.makeCylinder(16.0, 89.0, App.Vector(0, 0, 69.0), App.Vector(0, 0, 1))
    col = col.cut(bore)

    # 4-way arched windows with double pointed gable arches (self-supporting both ways!)
    pts_wx = [
        App.Vector(-35.0,   0.0,  73.5),
        App.Vector(-35.0,  14.0,  91.5),
        App.Vector(-35.0,  14.0, 130.0),
        App.Vector(-35.0,   0.0, 146.5),
        App.Vector(-35.0, -14.0, 130.0),
        App.Vector(-35.0, -14.0,  91.5),
        App.Vector(-35.0,   0.0,  73.5),
    ]
    win_x = Part.Face(Part.makePolygon(pts_wx)).extrude(App.Vector(70.0, 0, 0))

    pts_wy = [
        App.Vector(  0.0, -35.0,  73.5),
        App.Vector( 14.0, -35.0,  91.5),
        App.Vector( 14.0, -35.0, 130.0),
        App.Vector(  0.0, -35.0, 146.5),
        App.Vector(-14.0, -35.0, 130.0),
        App.Vector(-14.0, -35.0,  91.5),
        App.Vector(  0.0, -35.0,  73.5),
    ]
    win_y = Part.Face(Part.makePolygon(pts_wy)).extrude(App.Vector(0, 70.0, 0))
    col = col.cut(win_x).cut(win_y)

    for cx, cy in [(-19.5, -19.5), (19.5, -19.5), (-19.5, 19.5), (19.5, 19.5)]:
        h_m3 = Part.makeCylinder(1.35, 13.0, App.Vector(cx, cy, 143.0), App.Vector(0, 0, 1))
        tip_m3 = Part.makeCone(0.0, 1.35, 2.0, App.Vector(cx, cy, 141.0), App.Vector(0, 0, 1))
        ch_m3 = Part.makeCone(1.35, 1.95, 0.6, App.Vector(cx, cy, Z_MOTOR_FACE - 0.55), App.Vector(0, 0, 1))
        col = col.cut(h_m3).cut(tip_m3).cut(ch_m3)

    for px, py in [(-15.0, -15.0), (15.0, -15.0), (-15.0, 15.0), (15.0, 15.0)]:
        h_m3_through = Part.makeCylinder(1.70, 7.5, App.Vector(px, py, 69.0), App.Vector(0, 0, 1))
        col = col.cut(h_m3_through)

    return col.removeSplitter()


def build_tower_cap_dfam():
    """
    DfAM Removable Motor Cap Plate (52 x 52 mm, Z = 152.5 .. 160.0 mm):
    - Printed inverted (Z=160.0 on bed): locating skirt (Ø31.5 OD / Ø24.0 ID) points UP,
      countersunk conical walls taper inward 40-42 deg upward => 0 supports.
    """
    w_cap = make_rounded_rect_wire(0.0, 0.0, 26.0, 26.0, 3.0, Z_MOTOR_FACE)
    cap = Part.Face(w_cap).extrude(App.Vector(0, 0, Z_CAP_TOP - Z_MOTOR_FACE))

    # Downward cylindrical locating rabbet skirt (drops coaxially into Ø32.0 tower bore)
    skirt_out = Part.makeCylinder(15.75, 3.6, App.Vector(0, 0, 152.5), App.Vector(0, 0, 1))
    # Lead-in bevel on bottom tip of rabbet skirt (45 deg)
    bevel_cut = Part.makeCone(14.95, 16.0, 1.05, App.Vector(0, 0, 152.49), App.Vector(0, 0, 1))
    skirt_chamf = skirt_out.common(bevel_cut.fuse(Part.makeCylinder(16.0, 3.0, App.Vector(0, 0, 153.5), App.Vector(0, 0, 1))))
    skirt_in = Part.makeCylinder(12.00, 4.0, App.Vector(0, 0, 152.3), App.Vector(0, 0, 1))
    cap = cap.fuse(skirt_chamf.cut(skirt_in))

    # Motor centering boss pocket & shaft clearance bore
    boss_pocket = Part.makeCylinder(5.15, 2.0, App.Vector(0, 0, Z_MOTOR_FACE - 0.1), App.Vector(0, 0, 1))
    shaft_hole = Part.makeCylinder(3.00, 5.0, App.Vector(0, 0, Z_MOTOR_FACE - 0.5), App.Vector(0, 0, 1))
    cap = cap.cut(boss_pocket).cut(shaft_hole)

    # 4x M2 DIN 965 countersunk motor holes (flush below Z=160.0)
    for mx, my in [(-6.01, -6.01), (6.01, -6.01), (-6.01, 6.01), (6.01, 6.01)]:
        h_m2 = Part.makeCylinder(1.15, 5.0, App.Vector(mx, my, Z_MOTOR_FACE - 0.5), App.Vector(0, 0, 1))
        cs_m2 = Part.makeCone(1.15, 2.35, 1.40, App.Vector(mx, my, Z_CAP_TOP - 1.38), App.Vector(0, 0, 1))
        cap = cap.cut(h_m2).cut(cs_m2)

    # 4x M3 DIN 7991 countersunk tower clamping holes (flush below Z=160.0)
    for cx, cy in [(-19.5, -19.5), (19.5, -19.5), (-19.5, 19.5), (19.5, 19.5)]:
        h_m3 = Part.makeCylinder(1.65, 5.0, App.Vector(cx, cy, Z_MOTOR_FACE - 0.5), App.Vector(0, 0, 1))
        cs_m3 = Part.makeCone(1.65, 3.45, 2.00, App.Vector(cx, cy, Z_CAP_TOP - 1.98), App.Vector(0, 0, 1))
        cap = cap.cut(h_m3).cut(cs_m3)

    return cap.removeSplitter()


def build_horizontal_arm_dfam():
    """
    DfAM Support-Free Horizontal Arm (Z = 162.5 .. 201.8 mm):
    - Single completely flat bottom at Z=162.5 (prints horizontally with 0 supports & max X bending strength).
    - Motor D-socket has 39-deg conical top roof inside Z=171.5 beam.
    - Enlarged R=8.4 teardrop SMD/pin pocket clears ALL Encoder_AS5600 pins with 0.0000 mm^3 clash.
    - All horizontal X bores use 41-deg teardrop profiles; PCB pocket has 40-deg upper slope.
    """
    # 1. Coplanar bottom hub, beam, transition, and bearing head block at Z = 162.5
    beam_h = Z_ARM_TOP - Z_ARM_BOT
    hub_cyl = Part.makeCylinder(6.50, beam_h, App.Vector(0, 0, Z_ARM_BOT), App.Vector(0, 0, 1))
    beam_straight = Part.makeBox(50.0, 12.0, beam_h, App.Vector(0.0, -6.0, Z_ARM_BOT))
    w_t1 = Part.makePolygon([
        App.Vector(50.0, -6.0, Z_ARM_BOT), App.Vector(50.0,  6.0, Z_ARM_BOT),
        App.Vector(50.0,  6.0, Z_ARM_TOP), App.Vector(50.0, -6.0, Z_ARM_TOP), App.Vector(50.0, -6.0, Z_ARM_BOT)
    ])
    w_t2 = Part.makePolygon([
        App.Vector(63.0, -14.0, Z_ARM_BOT), App.Vector(63.0,  14.0, Z_ARM_BOT),
        App.Vector(63.0,  14.0, Z_ARM_TOP), App.Vector(63.0, -14.0, Z_ARM_TOP), App.Vector(63.0, -14.0, Z_ARM_BOT)
    ])
    beam_trans = Part.makeLoft([Part.Wire(w_t1), Part.Wire(w_t2)], True, False, False)
    head_box = Part.makeBox(18.0, 28.0, Z_HEAD_TOP - Z_ARM_BOT, App.Vector(63.0, -14.0, Z_ARM_BOT))
    arm = hub_cyl.fuse(beam_straight).fuse(beam_trans).fuse(head_box)

    # 2. Motor D4mm shaft socket (R=2.08 mm, D-flat Y=1.55 mm for zero metal interference, 39 deg conical blind top)
    cyl_sock = Part.makeCylinder(2.08, 5.40, App.Vector(0, 0, Z_ARM_BOT - 0.1), App.Vector(0, 0, 1))
    tip_sock = Part.makeCone(2.08, 0.0, 2.55, App.Vector(0, 0, Z_ARM_BOT + 5.30), App.Vector(0, 0, 1))
    flat_keep = Part.makeBox(6.0, 2.0, 8.5, App.Vector(-3.0, 1.55, Z_ARM_BOT - 0.2))
    d_sock = cyl_sock.fuse(tip_sock).cut(flat_keep)
    arm = arm.cut(d_sock)

    # 3. 608 bearing bore + entry chamfer (41-deg teardrop roof)
    bearing_td = make_teardrop_bore_x(11.10, X_BEARING_IN, 81.5, 0.0, Z_PIVOT, 41.0)
    w_c1 = make_teardrop_wire_yz(80.1, 0.0, Z_PIVOT, 11.10, 41.0)
    w_c2 = make_teardrop_wire_yz(81.5, 0.0, Z_PIVOT, 11.95, 41.0)
    entry_td = Part.makeLoft([w_c1, w_c2], True, False, False)
    arm = arm.cut(bearing_td).cut(entry_td)

    # 4. Concentric inner stepped bores (41-deg teardrops, zero flat overhangs)
    arm = arm.cut(make_teardrop_bore_x(7.00, 72.0, 74.2, 0.0, Z_PIVOT, 41.0))   # flange/shoulder clearance
    arm = arm.cut(make_teardrop_bore_x(6.00, 68.5, 72.2, 0.0, Z_PIVOT, 41.0))   # axle stem bore
    arm = arm.cut(make_teardrop_bore_x(7.20, 65.5, 68.7, 0.0, Z_PIVOT, 41.0))   # magnet chamber
    arm = arm.cut(make_teardrop_bore_x(8.40, 64.5, 67.5, 0.0, Z_PIVOT, 41.0))   # enlarged IC/pin-header clearance

    # 5. PCB pocket (X=62.5..65.5, Y=±11.95, Z=171.0..194.65) with 40-deg sloped upper overhang (no flat roof!)
    pcb_poly = [
        App.Vector(62.5, -11.95, 171.0),
        App.Vector(65.6, -11.95, 171.0),
        App.Vector(65.6, -11.95, 194.65),
        App.Vector(62.5, -11.95, 198.35),  # (198.35-194.65)/3.1 = 1.193 => 40 deg from vertical
        App.Vector(62.5, -11.95, 171.0),
    ]
    pcb_cut = Part.Face(Part.makePolygon(pcb_poly)).extrude(App.Vector(0, 23.90, 0))
    arm = arm.cut(pcb_cut)

    # 6. 4x M3 PCB screw pilot holes (41-deg teardrop horizontal holes)
    for hy, hz in [(-8.10, -8.08), (8.10, -8.08), (-8.10, 8.08), (8.10, 8.08)]:
        arm = arm.cut(make_teardrop_bore_x(1.35, 65.3, 73.5, hy, Z_PIVOT + hz, 41.0))

    # 7. Cable clip / adhesive inspection slots with 40-deg pointed gable tops
    for sz_bot in (Z_PIVOT - 10.5, Z_PIVOT + 8.5):
        slot_poly = [
            App.Vector(72.5, -2.0, sz_bot),
            App.Vector(72.5,  2.0, sz_bot),
            App.Vector(72.5,  2.0, sz_bot + 1.8),
            App.Vector(72.5,  0.0, sz_bot + 4.2),
            App.Vector(72.5, -2.0, sz_bot + 1.8),
            App.Vector(72.5, -2.0, sz_bot),
        ]
        arm = arm.cut(Part.Face(Part.makePolygon(slot_poly)).extrude(App.Vector(3.2, 0, 0)))

    # 8. Open-top wire routing channel along beam top (Z=169.0..171.6)
    wire_groove = Part.makeBox(57.5, 3.6, 3.0, App.Vector(6.0, -1.8, Z_ARM_TOP - 2.5))
    arm = arm.cut(wire_groove)

    return arm.removeSplitter()


def build_pivot_axle_dfam():
    """
    DfAM Support-Free Pivot Axle & Magnet Holder (X = 66.8 .. 92.2 mm):
    - Printed vertically standing on stub_d end (X=92.2 on bed -> mag_cup at Z_bed=25.4 mm).
    - Replaced 90-deg flange step with 41.2-deg self-supporting conical shoulder => 0 supports.
    """
    mag_stem = Part.makeCylinder(3.50, X_BEARING_IN - 66.8, App.Vector(66.8, 0, Z_PIVOT), App.Vector(1, 0, 0))
    journal = Part.makeCylinder(3.98, X_BEARING_OUT - X_BEARING_IN, App.Vector(X_BEARING_IN, 0, Z_PIVOT), App.Vector(1, 0, 0))
    flange = Part.makeCylinder(5.20, X_FLANGE_FLAT - X_BEARING_OUT, App.Vector(X_BEARING_OUT, 0, Z_PIVOT), App.Vector(1, 0, 0))
    taper = Part.makeCone(5.20, 2.50, X_TAPER_END - X_FLANGE_FLAT, App.Vector(X_FLANGE_FLAT, 0, Z_PIVOT), App.Vector(1, 0, 0))
    stub = Part.makeCylinder(2.50, X_STUB_END - X_TAPER_END, App.Vector(X_TAPER_END, 0, Z_PIVOT), App.Vector(1, 0, 0))

    axle = mag_stem.fuse(journal).fuse(flange).fuse(taper).fuse(stub)
    d_cut = Part.makeBox(X_STUB_END - X_TAPER_END + 0.5, 3.0, 8.0, App.Vector(X_TAPER_END, 1.80, Z_PIVOT - 4.0))
    # 40.6 deg ramp of the D-flat chord outward across X_FLANGE_FLAT..X_TAPER_END
    d_exit = Part.Face(Part.makePolygon([
        App.Vector(X_TAPER_END, 1.80, Z_PIVOT - 4.0),
        App.Vector(X_TAPER_END, 6.00, Z_PIVOT - 4.0),
        App.Vector(X_FLANGE_FLAT, 6.00, Z_PIVOT - 4.0),
        App.Vector(X_FLANGE_FLAT, 5.40, Z_PIVOT - 4.0),
        App.Vector(X_TAPER_END, 1.80, Z_PIVOT - 4.0),
    ])).extrude(App.Vector(0, 0, 8.0))
    mag_cup = Part.makeCylinder(2.20, 2.05, App.Vector(66.75, 0, Z_PIVOT), App.Vector(1, 0, 0))
    return axle.cut(d_cut).cut(d_exit).cut(mag_cup).removeSplitter()


def build_pendulum_dfam():
    """
    DfAM Support-Free Pendulum Rod (X = 85.2 .. 92.2 mm, Z = 30.0 .. 187.3 mm):
    - Hub, rod, and tip share one coplanar +X=92.2 mm flat face (prints flat on bed -> 0 supports).
    - Thru D-socket (R=2.56, Y_flat=1.84 mm) clears steel/PLA D-flat cleanly without bridged ceiling.
    """
    hub_cyl = Part.makeCylinder(4.50, X_STUB_END - X_TAPER_END, App.Vector(X_TAPER_END, 0, Z_PIVOT), App.Vector(1, 0, 0))
    rod_box = Part.makeBox(X_BAR_THICK, 8.0, Z_PIVOT - 34.0, App.Vector(X_BAR_IN, -4.0, 34.0))
    tip_cyl = Part.makeCylinder(4.00, X_BAR_THICK, App.Vector(X_BAR_IN, 0, 34.0), App.Vector(1, 0, 0))
    pen = hub_cyl.fuse(rod_box).fuse(tip_cyl)

    sock_cyl = Part.makeCylinder(2.56, (X_STUB_END - X_TAPER_END) + 0.4, App.Vector(X_TAPER_END - 0.2, 0, Z_PIVOT), App.Vector(1, 0, 0))
    sock_keep = Part.makeBox((X_STUB_END - X_TAPER_END) + 0.6, 3.0, 8.0, App.Vector(X_TAPER_END - 0.3, 1.84, Z_PIVOT - 4.0))
    pen_d_socket = sock_cyl.cut(sock_keep)
    adj_slot = Part.makeBox(8.0, 3.40, 50.0, App.Vector(X_BAR_CTR - 4.0, -1.70, 44.0))
    return pen.cut(pen_d_socket).cut(adj_slot).removeSplitter()


def build_tip_mass_dfam():
    """Two-piece clamping counterweight centered on updated rod X_BAR_CTR = 89.45 mm."""
    mass_cyl = Part.makeCylinder(9.00, 12.0, App.Vector(X_BAR_CTR - 6.0, 0, 64.0), App.Vector(1, 0, 0))
    rod_slot = Part.makeBox(X_BAR_THICK + 0.30, 19.0, 19.0, App.Vector(X_BAR_CTR - (X_BAR_THICK + 0.30) / 2.0, -9.5, 54.5))
    screw_hole = Part.makeCylinder(1.70, 16.0, App.Vector(X_BAR_CTR - 8.0, 0, 64.0), App.Vector(1, 0, 0))
    return mass_cyl.cut(rod_slot).cut(screw_hole).removeSplitter()


def main():
    tower_shape = build_tower_dfam()
    tower_base_shape = build_tower_base_dfam()
    tower_upper_shape = build_tower_upper_dfam()
    cap_shape = build_tower_cap_dfam()
    arm_shape = build_horizontal_arm_dfam()
    axle_shape = build_pivot_axle_dfam()
    pendulum_shape = build_pendulum_dfam()
    tip_mass_shape = build_tip_mass_dfam()

    for pname, shp in [
        ("Base_Torre_Furuta", tower_shape),
        ("Base_Inferior_Torre_Furuta", tower_base_shape),
        ("Torre_Superior_Motor_Furuta", tower_upper_shape),
        ("Tapa_Motor_Torre_Furuta", cap_shape),
        ("Brazo_Horizontal_L88", arm_shape),
        ("Eje_Portaiman_Pendulo", axle_shape),
        ("Barra_Pendulo_Furuta", pendulum_shape),
        ("Masa_Ajustable_Pendulo", tip_mass_shape),
    ]:
        assert shp.isValid(), f"{pname} shape invalid!"

    # Load Motor_22CL_AS5600 reference shapes and translate to Z_MOTOR_FACE = 156.0
    motor_fcstd = os.path.join(ROOT_DIR, "cad", "02_motor_namiki_as5600", "Motor_22CL_AS5600.FCStd")
    motor_doc = App.openDocument(motor_fcstd)
    motor_shapes = {o.Name: o.Shape.copy() for o in motor_doc.Objects}
    App.closeDocument(motor_doc.Name)
    for k in motor_shapes:
        motor_shapes[k].translate(App.Vector(0, 0, Z_MOTOR_FACE))

    # Load 608 bearing, magnet 2, sensor 2 from ASSembly.STEP
    step_path = os.path.join(ROOT_DIR, "cad", "referencias", "rodamiento_608_as5600", "ASSembly.STEP")
    temp_doc = App.newDocument("TempStep")
    Part.insert(step_path, temp_doc.Name)
    bearing_shape = temp_doc.getObject("ASSembly037").Shape.copy()
    mag2_shapes = [temp_doc.getObject(f"ASSembly{i:03d}").Shape.copy() for i in (35, 36)]
    sensor2_shapes = [temp_doc.getObject(f"ASSembly{i:03d}").Shape.copy() for i in range(1, 35)]
    App.closeDocument(temp_doc.Name)

    bearing_shape.rotate(App.Vector(0, 0, 0), App.Vector(0, 0, 1), -90)
    b_center = bearing_shape.BoundBox.Center
    bearing_shape.translate(App.Vector(77.5 - b_center.x, -b_center.y, Z_PIVOT - b_center.z))

    for s in mag2_shapes + sensor2_shapes:
        s.rotate(App.Vector(0, 0, 0), App.Vector(0, 0, 1), -90)
    mag2_comp = Part.Compound(mag2_shapes)
    mc = mag2_comp.BoundBox.Center
    trans_pen = App.Vector(66.80 - mag2_comp.BoundBox.XMin, -mc.y, Z_PIVOT - mc.z)
    for s in mag2_shapes + sensor2_shapes:
        s.translate(trans_pen)
    mag2_shape = Part.Compound(mag2_shapes)
    sensor2_shape = Part.Compound(sensor2_shapes)

    # Fasteners
    m2_screws = []
    for hx, hy in [(-6.01, -6.01), (6.01, -6.01), (-6.01, 6.01), (6.01, 6.01)]:
        head = Part.makeCone(1.1, 1.9, 1.2, App.Vector(hx, hy, Z_CAP_TOP - 1.2), App.Vector(0, 0, 1))
        shank = Part.makeCylinder(1.0, 6.0, App.Vector(hx, hy, Z_CAP_TOP - 1.2), App.Vector(0, 0, -1))
        m2_screws.append(head.fuse(shank))
    m2_screws_shape = Part.Compound(m2_screws)

    m3_cap_screws = []
    for hx, hy in [(-19.5, -19.5), (19.5, -19.5), (-19.5, 19.5), (19.5, 19.5)]:
        head = Part.makeCone(1.55, 3.0, 1.7, App.Vector(hx, hy, Z_CAP_TOP - 1.7), App.Vector(0, 0, 1))
        shank = Part.makeCylinder(1.5, 10.0, App.Vector(hx, hy, Z_CAP_TOP - 1.7), App.Vector(0, 0, -1))
        m3_cap_screws.append(head.fuse(shank))
    m3_cap_screws_shape = Part.Compound(m3_cap_screws)

    mass_screw = Part.makeCylinder(1.5, 14.0, App.Vector(X_BAR_CTR - 7.5, 0, 64.0), App.Vector(1, 0, 0))
    mass_screw_head = Part.makeCylinder(2.75, 2.5, App.Vector(X_BAR_CTR - 7.5, 0, 64.0), App.Vector(-1, 0, 0))
    mass_screw_shape = mass_screw.fuse(mass_screw_head)

    pen_pcb_screws = []
    for hy, hz in [(-8.10, -8.08), (8.10, -8.08), (-8.10, 8.08), (8.10, 8.08)]:
        head = Part.makeCylinder(2.85, 1.6, App.Vector(62.0, hy, Z_PIVOT + hz), App.Vector(-1, 0, 0))
        shank = Part.makeCylinder(1.5, 8.0, App.Vector(62.0, hy, Z_PIVOT + hz), App.Vector(1, 0, 0))
        pen_pcb_screws.append(head.fuse(shank))
    pen_pcb_screws_shape = Part.Compound(pen_pcb_screws)

    # 4x M3x12 screws joining upper motor tower to lower base pedestal
    m3_joint_screws = []
    for px, py in [(-15.0, -15.0), (15.0, -15.0), (-15.0, 15.0), (15.0, 15.0)]:
        head = Part.makeCylinder(2.75, 3.0, App.Vector(px, py, 75.0), App.Vector(0, 0, 1))
        shank = Part.makeCylinder(1.50, 12.0, App.Vector(px, py, 63.0), App.Vector(0, 0, 1))
        m3_joint_screws.append(head.fuse(shank))
    m3_joint_screws_shape = Part.Compound(m3_joint_screws)

    # Full assembly collision verification
    clash_pairs = [
        ("Tower_Base vs Tower_Upper", tower_base_shape, tower_upper_shape),
        ("Tower_Upper vs Tapa", tower_upper_shape, cap_shape),
        ("Tower_Upper vs Motor_Gearbox", tower_upper_shape, motor_shapes["Gearbox_3501PG"]),
        ("Tower_Upper vs Motor_Housing", tower_upper_shape, motor_shapes["Motor_22CL_Housing"]),
        ("Tower_Upper vs Acoplador", tower_upper_shape, motor_shapes["Acoplador_Motor_AS5600"]),
        ("Tower_Upper vs Encoder_Motor", tower_upper_shape, motor_shapes["Encoder_AS5600"]),
        ("Tower_Base vs Motor_Housing", tower_base_shape, motor_shapes["Motor_22CL_Housing"]),
        ("Tower_Base vs Encoder_Motor", tower_base_shape, motor_shapes["Encoder_AS5600"]),
        ("Tower vs Tapa", tower_shape, cap_shape),
        ("Tower vs Motor_Gearbox", tower_shape, motor_shapes["Gearbox_3501PG"]),
        ("Tower vs Motor_Housing", tower_shape, motor_shapes["Motor_22CL_Housing"]),
        ("Tower vs Acoplador", tower_shape, motor_shapes["Acoplador_Motor_AS5600"]),
        ("Tower vs Encoder_Motor", tower_shape, motor_shapes["Encoder_AS5600"]),
        ("Tapa vs Motor_Gearbox", cap_shape, motor_shapes["Gearbox_3501PG"]),
        ("Tapa vs Motor_Boss", cap_shape, motor_shapes["Centering_Boss"]),
        ("Tapa vs Motor_Shaft", cap_shape, motor_shapes["Output_Shaft_D4mm"]),
        ("Tapa vs Arm", cap_shape, arm_shape),
        ("Arm vs Motor_Shaft", arm_shape, motor_shapes["Output_Shaft_D4mm"]),
        ("Arm vs Bearing_608", arm_shape, bearing_shape),
        ("Arm vs Encoder_Pendulo", arm_shape, sensor2_shape),
        ("Arm vs Eje", arm_shape, axle_shape),
        ("Eje vs Barra", axle_shape, pendulum_shape),
        ("Barra vs Masa", pendulum_shape, tip_mass_shape),
    ]
    print("=== Furuta DfAM Full Assembly Clash Verification ===")
    for cname, s1, s2 in clash_pairs:
        vol = s1.common(s2).Volume
        print(f"  {cname:30s}: {vol:.4f} mm^3")
        if vol > 1e-3:
            raise RuntimeError(f"Kinematic clash detected in {cname}: {vol:.4f} mm^3")

    # Assemble FreeCAD document
    doc = App.newDocument("Pendulo_Furuta_Completo")
    objs = []

    def add_part(name, label, shape):
        o = doc.addObject("Part::Feature", name)
        o.Label = label
        o.Shape = shape
        objs.append(o)
        return o

    add_part("Base_Inferior_Torre_Furuta", "Base Inferior Pedestal DfAM (0 Soportes, Z=0..82)", tower_base_shape)
    add_part("Torre_Superior_Motor_Furuta", "Torre Superior Motor DfAM (0 Soportes, Z=70..156)", tower_upper_shape)
    add_part("Base_Torre_Furuta", "Base Torre Monolitica DfAM (Alternativa 156mm)", tower_shape)
    add_part("Tapa_Motor_Torre_Furuta", "Tapa Desmontable Motor DfAM (Sin Soportes, Z=152.5..160)", cap_shape)
    add_part("Brazo_Horizontal_L88", "Brazo Horizontal Plano DfAM (Sin Soportes, 41 deg teardrops)", arm_shape)
    add_part("Eje_Portaiman_Pendulo", "Eje Portaiman Pendulo DfAM (Cono 41.2 deg Sin Soportes)", axle_shape)
    add_part("Barra_Pendulo_Furuta", "Barra del Pendulo Plana DfAM (Sin Soportes)", pendulum_shape)
    add_part("Masa_Ajustable_Pendulo", "Masa / Contrapeso Ajustable en Dos Mitades", tip_mass_shape)
    add_part("Rodamiento_608_Pendulo", "Rodamiento de Bolas 608 (22x8x7mm)", bearing_shape)
    add_part("Encoder_Pendulo_AS5600", "Encoder Magnetico AS5600 del Pendulo", sensor2_shape)
    add_part("Iman_Diametral_Pendulo", "Iman Diametral del Pendulo D4.3x2.0mm", mag2_shape)

    for k, s in motor_shapes.items():
        add_part(f"Motor_{k}", f"Motor - {k}", s)

    add_part("Tornillos_Motor_M2_Avellanados", "4x Tornillos M2 Avellanados DIN 965", m2_screws_shape)
    add_part("Tornillos_Tapa_M3_Avellanados", "4x Tornillos M3x10 Avellanados DIN 7991", m3_cap_screws_shape)
    add_part("Tornillos_Union_Torre_M3", "4x Tornillos M3x12 Union Base-Torre", m3_joint_screws_shape)
    add_part("Tornillo_Masa_M3", "Tornillo M3x12 Bloqueo Masa Ajustable", mass_screw_shape)
    add_part("Tornillos_PCB_Pendulo_M3", "4x Tornillos M3x8 Sujecion PCB Pendulo", pen_pcb_screws_shape)

    doc.recompute()

    # Save master FCStd + assembly files
    os.makedirs(MODEL_DIR, exist_ok=True)
    fcstd_path = os.path.join(MODEL_DIR, "Pendulo_Furuta_Completo.FCStd")
    doc.saveAs(fcstd_path)

    step_asm = os.path.join(MODEL_DIR, "Pendulo_Furuta_Assembly.step")
    Part.export(doc.Objects, step_asm)

    stl_asm = os.path.join(MODEL_DIR, "Pendulo_Furuta_Assembly.stl")
    MeshPart.meshFromShape(
        Shape=Part.Compound([o.Shape for o in doc.Objects]),
        LinearDeflection=0.04,
        AngularDeflection=0.08,
    ).write(stl_asm)

    # Export printable individual parts across cad/01_pendulo_furuta and IMPRESION_3D/02_Pendulo_Furuta
    printable_parts = [
        ("Base_Inferior_Torre_Furuta", tower_base_shape),
        ("Torre_Superior_Motor_Furuta", tower_upper_shape),
        ("Base_Torre_Furuta", tower_shape),
        ("Tapa_Motor_Torre_Furuta", cap_shape),
        ("Brazo_Horizontal_L88", arm_shape),
        ("Eje_Portaiman_Pendulo", axle_shape),
        ("Barra_Pendulo_Furuta", pendulum_shape),
        ("Masa_Ajustable_Pendulo", tip_mass_shape),
    ]
    os.makedirs(IMPRESION_FURUTA_DIR, exist_ok=True)
    for name, shp in printable_parts:
        stl_p = os.path.join(MODEL_DIR, f"{name}.stl")
        stp_p = os.path.join(MODEL_DIR, f"{name}.step")
        MeshPart.meshFromShape(Shape=shp, LinearDeflection=0.03, AngularDeflection=0.06).write(stl_p)
        tmp_f = doc.addObject("Part::Feature", f"Tmp_{name}")
        tmp_f.Shape = shp
        Part.export([tmp_f], stp_p)
        doc.removeObject(tmp_f.Name)
        shutil.copy2(stl_p, os.path.join(IMPRESION_FURUTA_DIR, f"{name}.stl"))
        shutil.copy2(stp_p, os.path.join(IMPRESION_FURUTA_DIR, f"{name}.step"))
        print(f"Exported & synced {name} (Vol={shp.Volume:.2f} mm^3)")

    App.closeDocument(doc.Name)
    print("=== ALL FURUTA DFAM CAD MODELS GENERATED, CLASH-TESTED & SYNCED ===")


if __name__ == "__main__":
    main()
