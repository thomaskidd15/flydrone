"""
Slice the pico35 print plates for a Creality K1 (0.4 nozzle) with OrcaSlicer's CLI.

    uv run python frame/k1/slice.py

What it does:
1. Flattens OrcaSlicer's built-in Creality K1 profiles (they use "inherits" chains) into
   standalone JSON files in frame/k1/profiles/, then applies our overrides:
     process_plates.json  4 walls, 40% gyroid, 150 mm/s outer walls   (plates, legs, bracket)
     process_arms.json    6 walls, 100% infill                        (arms)
     filament_petg.json   245 C / 75 C bed, part fan 30-50%, aux fan OFF
     machine_k1.json      Creality K1 (0.4 nozzle), Klipper START_PRINT / END_PRINT macros
2. Runs orca-slicer on plate_A and plate_B -> frame/k1/gcode/*.gcode
3. Prints time, filament, temps and bed extents parsed from each G-code file.

OrcaSlicer is looked for in the portable folder below and on PATH.
"""
from __future__ import annotations

import json
import os
import re
import shutil
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
STL_DIR = HERE.parent / "stl" / "pico35"
PROFILES = HERE / "profiles"
GCODE = HERE / "gcode"

ORCA_CANDIDATES = [
    Path(os.environ.get("LOCALAPPDATA", "")) / "Programs" / "OrcaSlicer-portable" / "orca-slicer.exe",
    Path(r"C:\Program Files\OrcaSlicer\orca-slicer.exe"),
]

MACHINE = "Creality K1 (0.4 nozzle)"
PROCESS = "0.20mm Standard @Creality K1 (0.4 nozzle)"
FILAMENT = "Creality Generic PETG"

STRIP = {"inherits", "from", "setting_id", "instantiation", "compatible_printers", "compatible_printers_condition"}

PROCESS_COMMON = {
    "layer_height": "0.2",
    "initial_layer_print_height": "0.2",
    "outer_wall_speed": "150",
    "inner_wall_speed": "200",
    "sparse_infill_speed": "200",
    "internal_solid_infill_speed": "200",
    "top_surface_speed": "150",
    "initial_layer_speed": "50",
    "initial_layer_infill_speed": "60",
    "top_shell_layers": "5",
    "bottom_shell_layers": "4",
    "enable_prime_tower": "0",
    "skirt_loops": "1",
    "skirt_distance": "3",
    "seam_position": "aligned",
    "enable_support": "0",
    "gcode_label_objects": "1",
    "exclude_object": "1",
}
# Fixed small brim on the plates job. auto_brim sized a ~20 mm brim off the 32 mm legs around the
# whole combined object and ran off the bed.
PROCESS_PLATES = {**PROCESS_COMMON, "wall_loops": "4", "sparse_infill_density": "40%", "sparse_infill_pattern": "gyroid",
                  "brim_type": "outer_only", "brim_width": "4", "brim_object_gap": "0.1"}
PROCESS_ARMS = {**PROCESS_COMMON, "wall_loops": "6", "sparse_infill_density": "100%", "sparse_infill_pattern": "rectilinear",
                "brim_type": "no_brim"}

FILAMENT_OVERRIDES = {
    "nozzle_temperature": ["245"],
    "nozzle_temperature_initial_layer": ["250"],
    "cool_plate_temp": ["75"], "cool_plate_temp_initial_layer": ["78"],
    "eng_plate_temp": ["75"], "eng_plate_temp_initial_layer": ["78"],
    "hot_plate_temp": ["75"], "hot_plate_temp_initial_layer": ["78"],
    "textured_plate_temp": ["75"], "textured_plate_temp_initial_layer": ["78"],
    "fan_min_speed": ["30"],
    "fan_max_speed": ["50"],
    "overhang_fan_speed": ["80"],
    "additional_cooling_fan_speed": ["0"],      # K1 auxiliary side fan: off for PETG
    "close_fan_the_first_x_layers": ["2"],
    "slow_down_layer_time": ["12"],
    "filament_max_volumetric_speed": ["9"],
}


def find_orca() -> Path:
    for c in ORCA_CANDIDATES:
        if c.is_file():
            return c
    on_path = shutil.which("orca-slicer")
    if on_path:
        return Path(on_path)
    sys.exit("orca-slicer.exe not found; install OrcaSlicer (portable zip into %LOCALAPPDATA%\\Programs\\OrcaSlicer-portable)")


def load_vendor(orca: Path) -> dict[str, Path]:
    """name -> json path for every Creality profile shipped with Orca."""
    vendor = orca.parent / "resources" / "profiles" / "Creality"
    index: dict[str, Path] = {}
    for f in vendor.rglob("*.json"):
        try:
            d = json.loads(f.read_text(encoding="utf-8"))
        except Exception:
            continue
        if isinstance(d, dict) and "name" in d:
            index[d["name"]] = f
    return index


def flatten(name: str, index: dict[str, Path]) -> dict:
    d = json.loads(index[name].read_text(encoding="utf-8"))
    chain = [name]
    merged: dict = {}
    parent = d.get("inherits")
    if parent:
        merged = flatten(parent, index)
        chain = merged.pop("_chain") + chain
    merged.update({k: v for k, v in d.items() if k not in STRIP})
    merged["_chain"] = chain
    return merged


def write_profile(path: Path, prof: dict, overrides: dict, name: str) -> None:
    prof = dict(prof)
    chain = prof.pop("_chain")
    prof.update(overrides)
    prof["name"] = name
    # Orca's CLI compat check compares the process's compatible_printers with the printer's
    # *system* name, which for from=="system" is simply its name (for "User" it would be the
    # empty "inherits" of our flattened file). So present these as system presets.
    prof["from"] = "system"
    prof["version"] = "2.4.2.0"
    path.write_text(json.dumps(prof, indent=2), encoding="utf-8")
    print(f"  {path.name:22s} <- {' -> '.join(chain)}")


def build_profiles(orca: Path) -> None:
    PROFILES.mkdir(parents=True, exist_ok=True)
    idx = load_vendor(orca)
    print("profiles:")
    write_profile(PROFILES / "machine_k1.json", flatten(MACHINE, idx), {}, MACHINE)
    compat = {"compatible_printers": [MACHINE]}   # the CLI refuses a process/filament that does not list the printer
    proc = flatten(PROCESS, idx)
    write_profile(PROFILES / "process_plates.json", proc, {**PROCESS_PLATES, **compat}, "flydrone plates @K1 0.4")
    write_profile(PROFILES / "process_arms.json", proc, {**PROCESS_ARMS, **compat}, "flydrone arms @K1 0.4")
    write_profile(PROFILES / "filament_petg.json", flatten(FILAMENT, idx), {**FILAMENT_OVERRIDES, **compat}, "flydrone PETG @K1")


def _newest_gcode(d: Path) -> Path | None:
    files = sorted(d.glob("*.gcode"), key=lambda f: f.stat().st_mtime, reverse=True)
    return files[0] if files else None


def run_slice(orca: Path, stl: Path, process: Path, out: Path) -> None:
    """Slice with Orca's CLI and pull the G-code out of the sliced 3mf it writes."""
    import zipfile
    work = out.parent / "_work"
    if work.exists():
        shutil.rmtree(work)
    work.mkdir(parents=True)
    out.parent.mkdir(parents=True, exist_ok=True)
    log = out.with_suffix(".log")
    base = [
        str(orca), str(stl),
        "--load-settings", f"{process};{PROFILES / 'machine_k1.json'}",
        "--load-filaments", str(PROFILES / "filament_petg.json"),
        "--arrange", "0",                 # keep our layout; Orca's auto-arrange rotates the plate 90 deg
        "--debug", "2", "--logfile", str(log),
        "--outputdir", str(work),
    ]
    print(f"\nslicing {stl.name} with {process.name} ...")
    produced = None
    if True:
        r = subprocess.run(base + ["--slice", "0", "--export-3mf", "sliced.3mf"], capture_output=True, text=True)
        threemf = next(iter(work.glob("*.3mf")), None)
        if threemf is not None:
            with zipfile.ZipFile(threemf) as z:
                names = [n for n in z.namelist() if n.lower().endswith(".gcode")]
                if names:
                    (work / "from3mf.gcode").write_bytes(z.read(names[0]))
                    produced = work / "from3mf.gcode"
    if produced is None:
        print("  FAILED (exit", r.returncode, ")")
        if log.exists():
            print("  last log lines:")
            print("   " + "\n   ".join(log.read_text(errors="replace").splitlines()[-25:]))
        sys.exit(1)
    if out.exists():
        out.unlink()
    shutil.move(str(produced), str(out))
    shutil.rmtree(work, ignore_errors=True)


def summarize(gcode: Path) -> None:
    text = gcode.read_text(errors="replace")
    def grab(pat, default="?"):
        m = re.search(pat, text, re.M)
        return m.group(1).strip() if m else default
    xs, ys, zs = [], [], []
    for m in re.finditer(r"^G[0-3] [^;\n]*?X(-?[\d.]+)[^;\n]*?Y(-?[\d.]+)", text, re.M):
        xs.append(float(m.group(1))); ys.append(float(m.group(2)))
    for m in re.finditer(r"^G[0-3] [^;\n]*?Z(-?[\d.]+)", text, re.M):
        zs.append(float(m.group(1)))
    print(f"\n{gcode.name}  ({gcode.stat().st_size/1e6:.1f} MB)")
    print(f"  time      : {grab(r'^; estimated printing time.*?=\s*(.+)$')}")
    print(f"  filament  : {grab(r'^; (?:total )?filament used \[g\]\s*=\s*(.+)$')} g, {grab(r'^; (?:total )?filament used \[mm\]\s*=\s*(.+)$')} mm")
    print(f"  nozzle    : {grab(r'^; nozzle_temperature = (.+)$')} C (first layer {grab(r'^; nozzle_temperature_initial_layer = (.+)$')})")
    print(f"  bed       : {grab(r'^; (?:hot|cool|textured|eng)_plate_temp = (.+)$')} C")
    print(f"  walls     : {grab(r'^; wall_loops = (.+)$')}, infill {grab(r'^; sparse_infill_density = (.+)$')} {grab(r'^; sparse_infill_pattern = (.+)$')}")
    print(f"  aux fan   : {grab(r'^; additional_cooling_fan_speed = (.+)$')}%, part fan {grab(r'^; fan_min_speed = (.+)$')}-{grab(r'^; fan_max_speed = (.+)$')}%")
    if xs:
        print(f"  extents   : X {min(xs):.1f}..{max(xs):.1f}  Y {min(ys):.1f}..{max(ys):.1f}  Z max {max(zs):.1f}  (bed 0..220)")
    print(f"  start/end : {'START_PRINT' in text}/{'END_PRINT' in text}   layers: {text.count(';LAYER_CHANGE') or text.count(';AFTER_LAYER_CHANGE')}")


def main() -> None:
    orca = find_orca()
    print("orca-slicer:", orca)
    build_profiles(orca)
    jobs = [
        ("plate_A_plates_legs_bracket.stl", "process_plates.json", "flydrone_K1_plateA_plates_legs_bracket_PETG.gcode"),
        ("plate_B_arms.stl", "process_arms.json", "flydrone_K1_plateB_arms_PETG.gcode"),
    ]
    for stl, proc, out in jobs:
        run_slice(orca, STL_DIR / stl, PROFILES / proc, GCODE / out)
    for _, _, out in jobs:
        summarize(GCODE / out)


if __name__ == "__main__":
    main()
