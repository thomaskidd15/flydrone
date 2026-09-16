"""Flat sections of every part, for eyeballing hole placement. Writes stl/<preset>/sections.png"""
import sys
from pathlib import Path
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import trimesh

preset = sys.argv[1] if len(sys.argv) > 1 else "pico35"
d = Path(__file__).parent / "stl" / preset
specs = [  # file, plane origin z (fraction of height), title
    ("bottom_plate.stl", 0.5, "bottom plate (from above)"),
    ("top_plate.stl", 0.5, "top plate (from above)"),
    ("arm_x4.stl", 0.25, "arm, section through the tab"),
    ("arm_x4.stl", 0.8, "arm, section through the body"),
    ("leg_x4.stl", 0.85, "leg, section through the nut traps"),
    ("camera_bracket.stl", 0.5, "camera bracket (as printed, on its side)"),
]
fig, axes = plt.subplots(2, 3, figsize=(18, 11))
for ax, (f, frac, title) in zip(axes.flat, specs):
    m = trimesh.load(d / f)
    lo, hi = m.bounds
    z = lo[2] + frac * (hi[2] - lo[2])
    sec = m.section(plane_origin=[0, 0, z], plane_normal=[0, 0, 1])
    if sec is None:
        ax.set_title(title + " (no section)"); continue
    p2, _ = sec.to_planar(normal=[0, 0, 1])
    for ent in p2.entities:
        pts = p2.vertices[ent.points]
        ax.plot(pts[:, 0], pts[:, 1], "k-", lw=0.8)
    ax.set_aspect("equal"); ax.grid(True, alpha=0.3); ax.set_title(f"{title}\nz={z:.1f}")
fig.suptitle(preset)
fig.tight_layout()
fig.savefig(d / "sections.png", dpi=90)
print("wrote", d / "sections.png")
