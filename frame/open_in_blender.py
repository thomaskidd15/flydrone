"""
Open the generated frame in Blender for a look.

    "C:\\Program Files\\Blender Foundation\\Blender 5.2\\blender.exe" --python frame/open_in_blender.py -- pico35

Loads assembly_preview.stl (solid blue) and ghost_components.stl (transparent grey:
motors, prop discs, FC stack, brain board, battery) into a fresh scene in millimetres
and sets a 3/4 view. The import is deferred with a timer because the STL importer
needs a window context that does not exist yet while Blender is still starting.
"""
import sys
from math import radians
from pathlib import Path

import bpy
from mathutils import Euler, Vector

preset = "pico35"
if "--" in sys.argv:
    args = sys.argv[sys.argv.index("--") + 1:]
    if args:
        preset = args[0]

STL_DIR = Path(__file__).resolve().parent / "stl" / preset


def material(name, rgba, alpha=1.0):
    mat = bpy.data.materials.new(name)
    mat.use_nodes = True
    bsdf = next(n for n in mat.node_tree.nodes if n.type == "BSDF_PRINCIPLED")
    bsdf.inputs["Base Color"].default_value = rgba
    bsdf.inputs["Alpha"].default_value = alpha
    mat.diffuse_color = (rgba[0], rgba[1], rgba[2], alpha)
    if alpha < 1.0:
        try:
            mat.surface_render_method = "BLENDED"
        except Exception:
            pass
    return mat


def load():
    wm = bpy.context.window_manager
    if not wm.windows:
        return 0.3  # window not up yet, try again
    win = wm.windows[0]
    area = next((a for a in win.screen.areas if a.type == "VIEW_3D"), None)
    if area is None:
        return 0.3
    region = next(r for r in area.regions if r.type == "WINDOW")

    scene = bpy.context.scene
    scene.unit_settings.system = "METRIC"
    scene.unit_settings.scale_length = 0.001
    scene.unit_settings.length_unit = "MILLIMETERS"
    for o in list(bpy.data.objects):
        bpy.data.objects.remove(o, do_unlink=True)

    def import_stl(name, path):
        with bpy.context.temp_override(window=win, area=area, region=region):
            bpy.ops.wm.stl_import(filepath=str(path))
        obj = bpy.context.selected_objects[0]
        obj.name = name
        obj.select_set(False)
        return obj

    frame = import_stl("Frame", STL_DIR / "assembly_preview.stl")
    frame.data.materials.append(material("FramePETG", (0.10, 0.35, 0.85, 1.0)))
    ghost = import_stl("Components_ghost", STL_DIR / "ghost_components.stl")
    ghost.data.materials.append(material("Ghost", (0.6, 0.6, 0.6, 1.0), alpha=0.25))

    space = area.spaces.active
    space.shading.type = "MATERIAL"
    space.clip_end = 100000
    space.lens = 50
    rv3d = space.region_3d
    bb = [frame.matrix_world @ Vector(c) for c in frame.bound_box]
    rv3d.view_perspective = "PERSP"
    rv3d.view_rotation = Euler((radians(58), 0.0, radians(35)), "XYZ").to_quaternion()
    rv3d.view_location = sum(bb, Vector()) / 8.0
    rv3d.view_distance = 2.0 * max(frame.dimensions)
    area.tag_redraw()
    print(f"loaded {preset} from {STL_DIR}: frame {[round(d, 1) for d in frame.dimensions]} mm")
    return None


bpy.app.timers.register(load, first_interval=0.5)
