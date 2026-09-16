"""
Fly-brain drone frame generator.

Parametric X quad frame, fully 3D printed. Two presets:

  pico35   3.5-inch props, 180 mm wheelbase, Raspberry Pi Pico 2 as the onboard brain.
           Small body, 20x20 or 30.5x30.5 flight-controller stack, camera on the nose,
           optical-flow sensor on the tail, battery underneath between four legs.

  pi5_5in  5-inch props, 250 mm wheelbase, Raspberry Pi 5 on the top deck.
           Same construction scaled up, for running the whole FlyWire brain onboard.

Run:   uv run python frame/gen_frame.py            (pico35)
       uv run python frame/gen_frame.py pi5_5in
STLs land in frame/stl/<preset>/ with a preview.png next to them.

Coordinate system (assembly): x = forward (nose), y = left, z = up.
z = 0 is the TOP surface of the bottom plate.

Construction: each arm has a thin tab that is sandwiched between the bottom and
top plates and held by two M3 bolts. The arm steps up to full height exactly where
the plate "ear" ends, so the step locates the arm. The same two bolts continue
down into a nut trap in a leg under each corner. Print arms lying flat.
"""
from __future__ import annotations

import math
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import manifold3d as m3d
from manifold3d import CrossSection, JoinType, Manifold
import trimesh

M3 = 3.3   # clearance hole diameters
M25 = 2.8
M2 = 2.3


@dataclass
class Preset:
    name: str
    wheelbase: float          # motor-to-motor diagonal
    prop_dia: float           # for the clearance check / preview only
    plate_half: float         # centre square is 2*plate_half wide
    ear_w: float              # width of the diagonal ears the arms bolt to
    ear_r: float              # ears extend to this radius
    bot_t: float
    top_t: float
    outline_round: float
    arm_w: float
    arm_h: float
    tab_r0: float             # tab starts at this radius (must clear FC bolts)
    arm_bolt_r: tuple[float, float]
    motor_pad_r: float
    motor_center_hole: float
    batt_slot_x: float        # strap slots at +-x
    batt_slot_len: float
    tail_len: float           # bottom-plate tail for the optical flow sensor
    tail_half_w: float
    leg_h: float
    leg_l: float
    leg_w: float
    pi5: bool                 # add Pi 5 standoff holes on the top plate
    cam_upright_h: float
    batt_box: tuple[float, float, float]   # preview only
    brain_box: tuple[float, float, float]  # preview only (Pico or Pi)
    brain_z: float

    @property
    def tab_t(self) -> float:
        return self.bot_t

    @property
    def tab_r1(self) -> float:
        return self.ear_r + 0.4

    @property
    def motor_r(self) -> float:
        return self.wheelbase / 2.0


PRESETS = {
    "pico35": Preset(
        name="pico35", wheelbase=180.0, prop_dia=89.0,
        plate_half=27.0, ear_w=16.0, ear_r=47.0, bot_t=3.5, top_t=2.5, outline_round=2.5,
        arm_w=12.0, arm_h=7.0, tab_r0=25.0, arm_bolt_r=(30.0, 41.0),
        motor_pad_r=15.0, motor_center_hole=6.0,
        batt_slot_x=14.0, batt_slot_len=22.0, tail_len=22.0, tail_half_w=13.0,
        leg_h=32.0, leg_l=22.0, leg_w=12.0, pi5=False, cam_upright_h=18.0,
        batt_box=(60.0, 30.0, 26.0), brain_box=(21.0, 51.0, 8.0), brain_z=20.0,
    ),
    "pi5_5in": Preset(
        name="pi5_5in", wheelbase=250.0, prop_dia=127.0,
        plate_half=42.0, ear_w=20.0, ear_r=68.0, bot_t=4.0, top_t=3.0, outline_round=3.0,
        arm_w=14.0, arm_h=8.0, tab_r0=38.0, arm_bolt_r=(47.0, 60.0),
        motor_pad_r=15.0, motor_center_hole=8.0,
        batt_slot_x=20.0, batt_slot_len=30.0, tail_len=24.0, tail_half_w=15.0,
        leg_h=45.0, leg_l=30.0, leg_w=14.0, pi5=True, cam_upright_h=24.0,
        batt_box=(72.0, 35.0, 32.0), brain_box=(85.0, 56.0, 18.0), brain_z=30.0,
    ),
}

# shared hardware patterns
FC_PITCH_BIG = 30.5     # M3
FC_PITCH_SMALL = 20.0   # M2
PICO_HOLES = (11.4, 47.0)   # Pico / Pico 2 mounted ACROSS the body (long axis = y)
PI_PITCH = (58.0, 49.0)
PI_OFFSET_X = -10.0     # centres the 85 mm board over the frame
FLOW_VIEW_HOLE = 10.0
FLOW_HOLE_PITCH = 15.0
CAM_SLOT_Y = 10.5       # camera upright slots at +-y (fits Pi Camera Module 3, 21 mm)
CAM_TILT = 15.0
NUT_AF = 5.7
NUT_T = 2.8
NUT_DEPTH = 5.0
DIAG = (45.0, 135.0, 225.0, 315.0)

m3d.set_circular_segments(48)


def polar(r: float, deg: float) -> tuple[float, float]:
    a = math.radians(deg)
    return (r * math.cos(a), r * math.sin(a))


def cyl(d: float, h: float, x: float = 0.0, y: float = 0.0, z: float = 0.0) -> Manifold:
    return Manifold.cylinder(h, d / 2.0).translate([x, y, z])


def slot(x: float, y: float, sx: float, sy: float, h: float, z: float = 0.0) -> Manifold:
    r = min(sx, sy) / 2.0
    cs = CrossSection.square([sx - 2 * r, sy - 2 * r], center=True).offset(r, JoinType.Round)
    return cs.extrude(h).translate([x, y, z])


def union(parts: list[Manifold]) -> Manifold:
    out = parts[0]
    for p in parts[1:]:
        out = out + p
    return out


def plate_outline(p: Preset, with_tail: bool) -> CrossSection:
    cs = CrossSection.square([2 * p.plate_half, 2 * p.plate_half], center=True)
    for a in DIAG:
        cs = cs + CrossSection.square([p.ear_r, p.ear_w]).translate([0.0, -p.ear_w / 2.0]).rotate(a)
    if with_tail:
        cs = cs + CrossSection.square([p.plate_half + p.tail_len, 2 * p.tail_half_w]).translate(
            [-(p.plate_half + p.tail_len), -p.tail_half_w]
        )
    return cs.offset(-p.outline_round, JoinType.Round).offset(p.outline_round, JoinType.Round)


def arm_bolt_holes(p: Preset, h: float, z: float) -> list[Manifold]:
    return [cyl(M3, h, *polar(r, a), z) for a in DIAG for r in p.arm_bolt_r]


def fc_holes(h: float, z: float) -> list[Manifold]:
    out = []
    for dx in (-1, 1):
        for dy in (-1, 1):
            out.append(cyl(M3, h, dx * FC_PITCH_BIG / 2, dy * FC_PITCH_BIG / 2, z))
            out.append(cyl(M2, h, dx * FC_PITCH_SMALL / 2, dy * FC_PITCH_SMALL / 2, z))
    return out


def bottom_plate(p: Preset) -> Manifold:
    body = plate_outline(p, with_tail=True).extrude(p.bot_t).translate([0, 0, -p.bot_t])
    h, z = p.bot_t + 2, -p.bot_t - 1
    cut = arm_bolt_holes(p, h, z) + fc_holes(h, z)
    for sx in (-p.batt_slot_x, p.batt_slot_x):
        cut.append(slot(sx, 0.0, 4.0, p.batt_slot_len, h, z))
    fx = -(p.plate_half + p.tail_len / 2.0)
    cut.append(cyl(FLOW_VIEW_HOLE, h, fx, 0.0, z))
    for dx in (-1, 1):
        for dy in (-1, 1):
            cut.append(cyl(M2, h, fx + dx * FLOW_HOLE_PITCH / 2, dy * FLOW_HOLE_PITCH / 2, z))
    wy = p.plate_half - 6.0
    for sy in (-wy, wy):
        cut.append(slot(0.0, sy, 12.0, 4.0, h, z))
    return body - union(cut)


def top_plate(p: Preset) -> Manifold:
    z0 = p.tab_t
    body = plate_outline(p, with_tail=False).extrude(p.top_t).translate([0, 0, z0])
    h, z = p.top_t + 2, z0 - 1
    cut = arm_bolt_holes(p, h, z) + fc_holes(h, z)
    for dx in (-1, 1):
        for dy in (-1, 1):
            cut.append(cyl(M2, h, dx * PICO_HOLES[0] / 2, dy * PICO_HOLES[1] / 2, z))
            if p.pi5:
                cut.append(cyl(M25, h, PI_OFFSET_X + dx * PI_PITCH[0] / 2, dy * PI_PITCH[1] / 2, z))
    for dy in (-9.0, 9.0):                       # camera bracket bolts, base sits on the nose edge
        cut.append(cyl(M3, h, p.plate_half - 5.0, dy, z))
    wy = p.plate_half - 10.0
    for sy in (-wy, wy):                          # wire pass-throughs, either side of the stack
        cut.append(slot(0.0, sy, 12.0, 4.0, h, z))
    if p.pi5:
        for sx in (-30.0, 30.0):                  # lightening slots between FC and Pi standoffs
            cut.append(slot(sx, 0.0, 6.0, 26.0, h, z))
    return body - union(cut)


def arm_local(p: Preset) -> Manifold:
    """Arm in its own frame: axis along +x from the body centre, lying flat, z from 0."""
    tab = Manifold.cube([p.tab_r1 - p.tab_r0, p.arm_w, p.tab_t]).translate([p.tab_r0, -p.arm_w / 2, 0])
    body_len = p.motor_r - p.tab_r1          # body ends at the pad centre; the pad disc finishes the arm
    body = Manifold.cube([body_len, p.arm_w, p.arm_h]).translate([p.tab_r1, -p.arm_w / 2, 0])
    pad = cyl(2 * p.motor_pad_r, p.arm_h, p.motor_r, 0.0, 0.0)
    cut = [cyl(M3, p.tab_t + 2, r, 0.0, -1) for r in p.arm_bolt_r]
    for dx in (-1, 1):
        for dy in (-1, 1):
            # 16x16 M3 pattern, square to the arm
            cut.append(cyl(M3, p.arm_h + 2, p.motor_r + dx * 8.0, dy * 8.0, -1))
    for ang in (0.0, 90.0, 180.0, 270.0):
        # 12x12 M2 pattern rotated 45 deg so it does not merge with the M3 holes
        x, y = polar(6.0 * math.sqrt(2), ang)
        cut.append(cyl(M2, p.arm_h + 2, p.motor_r + x, y, -1))
    cut.append(cyl(p.motor_center_hole, p.arm_h + 2, p.motor_r, 0.0, -1))
    return (tab + body + pad) - union(cut)


def leg_local(p: Preset) -> Manifold:
    """Leg standing on its foot, z from 0 to leg_h. Long axis along +x (= the diagonal)."""
    top = Manifold.cube([p.leg_l, p.leg_w, p.leg_h - 3.0]).translate([-p.leg_l / 2, -p.leg_w / 2, 3.0])
    foot = Manifold.cube([p.leg_l - 4.0, p.leg_w - 4.0, 0.2]).translate([-(p.leg_l - 4) / 2, -(p.leg_w - 4) / 2, 0])
    leg = (top + foot).hull()
    span = p.arm_bolt_r[1] - p.arm_bolt_r[0]
    hex_r = (NUT_AF / 2.0) / math.cos(math.radians(30))
    cut = []
    for sx in (-span / 2, span / 2):
        z_nut = p.leg_h - NUT_DEPTH - NUT_T
        cut.append(cyl(M3, NUT_DEPTH + NUT_T + 4, sx, 0.0, z_nut - 2))
        cut.append(Manifold.cylinder(NUT_T, hex_r, hex_r, 6).translate([sx, 0.0, z_nut]))
        cut.append(Manifold.cube([NUT_AF, p.leg_w / 2 + 1, NUT_T]).translate([sx - NUT_AF / 2, 0.0, z_nut]))
    return leg - union(cut)


def camera_bracket_local(p: Preset) -> Manifold:
    """Local frame: plate front edge at x = 0, base extends to -x over the plate, z = 0 is the plate top.
    A lip hooks over the plate edge so two bolts are enough."""
    base = Manifold.cube([10.0, 30.0, 3.0]).translate([-10.0, -15.0, 0])
    lip = Manifold.cube([2.5, 30.0, p.top_t + 3.0]).translate([0.0, -15.0, -p.top_t])
    up_t, up_w, up_h = 3.0, 32.0, p.cam_upright_h
    upright = Manifold.cube([up_t, up_w, up_h]).translate([-up_t, -up_w / 2, 0])
    cut = []
    for sy in (-CAM_SLOT_Y, CAM_SLOT_Y):          # vertical slots: any hole spacing 6..16 mm tall
        cut.append(Manifold.cube([up_t + 2, 2.4, 10.0]).translate([-up_t - 1, sy - 1.2, up_h / 2 - 5.0]))
    cut.append(Manifold.cylinder(up_t + 2, 4.5).rotate([0, 90, 0]).translate([-up_t - 1, 0, up_h / 2]))
    for sy in (-13.0, 13.0):                     # zip-tie slots near the outer edges
        cut.append(Manifold.cube([up_t + 2, 1.6, 8.0]).translate([-up_t - 1, sy - 0.8, up_h / 2 - 4.0]))
    upright = (upright - union(cut)).rotate([0, -CAM_TILT, 0]).translate([0.0, 0, 3.0])
    gusset = Manifold.cube([7.0, 3.0, 8.0]).translate([-7.0 - up_t, -1.5, 3.0])
    part = base + lip + upright + gusset
    bolts = [cyl(M3, 5.0, -5.0, dy, -1) for dy in (-9.0, 9.0)]
    return part - union(bolts)


def save(part: Manifold, out: Path, name: str) -> trimesh.Trimesh:
    mesh = part.to_mesh()
    tm = trimesh.Trimesh(vertices=np.asarray(mesh.vert_properties)[:, :3], faces=np.asarray(mesh.tri_verts))
    tm.merge_vertices()
    out.mkdir(parents=True, exist_ok=True)
    tm.export(out / name)
    lo, hi = tm.bounds
    print(
        f"  {name:22s} {hi[0]-lo[0]:6.1f} x {hi[1]-lo[1]:6.1f} x {hi[2]-lo[2]:5.1f} mm "
        f"{part.volume()/1000:6.1f} cm3  ~{part.volume()/1000*1.27:5.1f} g PETG  watertight={tm.is_watertight}"
    )
    return tm


def assembly(p: Preset, parts: dict[str, Manifold]) -> Manifold:
    pieces = [parts["bottom"], parts["top"]]
    for a in DIAG:
        pieces.append(parts["arm"].rotate([0, 0, a]))
        lx, ly = polar(sum(p.arm_bolt_r) / 2, a)
        pieces.append(parts["leg"].rotate([0, 0, a]).translate([lx, ly, -p.bot_t - p.leg_h]))
    pieces.append(parts["camera"].translate([p.plate_half, 0, p.tab_t + p.top_t]))
    return union(pieces)


def ghosts(p: Preset) -> Manifold:
    """Motors, prop discs, FC stack, brain board, battery: preview only."""
    g = []
    for a in DIAG:
        mx, my = polar(p.motor_r, a)
        g.append(cyl(0.22 * p.prop_dia, 18.0, mx, my, p.arm_h))
        g.append(cyl(p.prop_dia, 1.0, mx, my, p.arm_h + 18.0))
    fc = 36.0 if p.pi5 else 30.0
    g.append(Manifold.cube([fc, fc, 16]).translate([-fc / 2, -fc / 2, p.tab_t + p.top_t + 4]))
    bx, by, bz = p.brain_box
    g.append(Manifold.cube([bx, by, bz]).translate([-bx / 2, -by / 2, p.tab_t + p.top_t + p.brain_z]))
    sx, sy, sz = p.batt_box
    g.append(Manifold.cube([sx, sy, sz]).translate([-sx / 2, -sy / 2, -p.bot_t - sz - 1]))
    return union(g)


def clearance_report(p: Preset) -> None:
    mx, my = polar(p.motor_r, 45.0)
    bx, by, _ = p.brain_box
    d_brain = math.hypot(mx - bx / 2, my - by / 2) - p.prop_dia / 2
    cam = (p.plate_half + 3.0 + 3.0, 15.0)   # upright + camera module thickness, half width
    d_cam = math.hypot(mx - cam[0], my - cam[1]) - p.prop_dia / 2
    side = p.wheelbase / math.sqrt(2)
    print(f"  prop disc to brain board corner:  {d_brain:5.1f} mm")
    print(f"  prop disc to camera corner:       {d_cam:5.1f} mm")
    print(f"  prop tip to prop tip:             {side - p.prop_dia:5.1f} mm")
    print(f"  arm print length:                 {p.motor_r + p.motor_pad_r - p.tab_r0:5.1f} mm")
    print(f"  height, feet to top of brain box: {p.leg_h + p.bot_t + p.tab_t + p.top_t + p.brain_z + p.brain_box[2]:5.1f} mm")


def render_preview(asm: trimesh.Trimesh, ghost: trimesh.Trimesh, path: Path, lim: float) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    fig = plt.figure(figsize=(16, 7))
    for i, (title, elev, azim) in enumerate([("Top view", 90, -90), ("Isometric", 28, -50)], 1):
        ax = fig.add_subplot(1, 2, i, projection="3d")
        for mesh, color, alpha in ((asm, "#3a7bd5", 0.95), (ghost, "#888888", 0.22)):
            ax.add_collection3d(Poly3DCollection(mesh.vertices[mesh.faces], alpha=alpha, facecolor=color, edgecolor="none"))
        ax.set_xlim(-lim, lim); ax.set_ylim(-lim, lim); ax.set_zlim(-lim * 0.66, lim * 0.66)
        ax.set_box_aspect((1, 1, 2 / 3))
        ax.view_init(elev=elev, azim=azim)
        ax.set_title(title); ax.set_xlabel("x fwd"); ax.set_ylabel("y")
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    print(f"  preview -> {path}")


def build(p: Preset) -> None:
    out = Path(__file__).parent / "stl" / p.name
    print(f"[{p.name}]  wheelbase {p.wheelbase:.0f} mm, props {p.prop_dia:.0f} mm")
    parts = {
        "bottom": bottom_plate(p),
        "top": top_plate(p),
        "arm": arm_local(p),
        "leg": leg_local(p),
        "camera": camera_bracket_local(p),
    }
    save(parts["bottom"], out, "bottom_plate.stl")
    save(parts["top"], out, "top_plate.stl")
    save(parts["arm"], out, "arm_x4.stl")
    save(parts["leg"], out, "leg_x4.stl")
    save(parts["camera"].rotate([90, 0, 0]), out, "camera_bracket.stl")   # on its side: no overhangs
    asm_tm = save(assembly(p, parts), out, "assembly_preview.stl")
    ghost_tm = save(ghosts(p), out, "ghost_components.stl")
    clearance_report(p)
    render_preview(asm_tm, ghost_tm, out / "preview.png", lim=p.motor_r + p.prop_dia / 2 + 10)


if __name__ == "__main__":
    names = sys.argv[1:] or ["pico35"]
    for n in names:
        build(PRESETS[n])
