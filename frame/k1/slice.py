"""
Slice the pico35 print plates for a Creality K1 with OrcaSlicer's CLI.

    uv run python frame/k1/slice.py              # 0.6 mm nozzle (the K1 this was built for)
    uv run python frame/k1/slice.py 0.4          # 0.4 mm nozzle
    uv run python frame/k1/slice.py 0.4 0.6      # both

What it does:
1. Flattens OrcaSlicer's built-in Creality K1 profiles (they use "inherits" chains) into
   standalone JSON files in frame/k1/profiles/<nozzle>/, then applies our overrides:
     process_plates.json  plates, legs, camera bracket: 40% gyroid, small fixed brim
     process_arms.json    arms: 100% infill, no brim
     filament_petg.json   245 C / 75 C bed, part fan 30-50%, K1 auxiliary fan OFF
     machine_k1.json      Creality K1 (<nozzle>), Klipper START_PRINT / END_PRINT macros
2. Runs orca-slicer on plate_A and plate_B -> frame/k1/gcode/flydrone_K1_<nozzle>nozzle_*.gcode
3. Prints time, filament, temps and bed extents parsed from each G-code file.

Orca CLI notes (2.4.2), learned the hard way:
- profile JSON needs from="system" and compatible_printers=[<machine name>] or the CLI
  refuses the process/filament as "not compatible with printer";
- --export-gcode is a dead action; --slice 0 --export-3mf writes plate_1.gcode to --outputdir;
- pass --arrange 0 or the auto-arrange rotates the plate 90 degrees;
- auto_brim sized a ~20 mm brim off the 32 mm legs around the whole combined object and ran
  off the bed, hence the fixed outer_only brim.
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
GCODE = HERE / "gcode"

ORCA_CANDIDATES = [
    Path(os.environ.get("LOCALAPPDATA", "")) / "Programs" / "OrcaSlicer-portable" / "orca-slicer.exe",
    Path(r"C:\Program Files\OrcaSlicer\orca-slicer.exe"),
]

FILAMENT = "Creality Generic PETG"
STRIP = {"inherits", "from", "setting_id", "instantiation", "compatible_printers", "compatible_printers_condition"}

# Per-nozzle: which Orca system profiles to start from and what to override.
# Layer heights are chosen so every part thickness is a whole number of layers
# (plates 3.5 / 2.5 mm, arms 7 mm, legs and bracket 32 mm).
NOZZLES = {
    "0.4": dict(
        machine="Creality K1 (0.4 nozzle)",
        process="0.20mm Standard @Creality K1 (0.4 nozzle)",
        layer="0.2",
        speeds=dict(outer_wall_speed="150", inner_wall_speed="200", sparse_infill_speed="200",
                    internal_solid_infill_speed="200", top_surface_speed="150"),
        plates=dict(wall_loops="4", top_shell_layers="5", bottom_shell_layers="4"),
        arms=dict(wall_loops="6"),
        hole_comp="0",
        max_vol="9",
    ),
    "0.6": dict(
        machine="Creality K1 (0.6 nozzle)",
        process="0.30mm Standard @Creality K1 (0.6 nozzle)",
        layer="0.25",
        speeds=dict(outer_wall_speed="100", inner_wall_speed="150", sparse_infill_speed="150",
                    internal_solid_infill_speed="150", top_surface_speed="100"),
        plates=dict(wall_loops="3", top_shell_layers="4", bottom_shell_layers="3"),   # 3 x 0.62 = 1.9 mm of wall
        arms=dict(wall_loops="4"),
        hole_comp="0.1",       # small holes close up more with a fat nozzle; opens M2/M3 holes by 0.1 mm
        max_vol="12",
    ),
}

PROCESS_COMMON = {
    "initial_layer_speed": "50",
    "initial_layer_infill_speed": "60",
    "enable_prime_tower": "0",
    "skirt_loops": "1",
    "skirt_distance": "3",
    "seam_position": "aligned",
    "enable_support": "0",
    "gcode_label_objects": "1",
    "exclude_object": "1",
}
PLATES_EXTRA = {"sparse_infill_density": "40%", "sparse_infill_pattern": "gyroid",
                "brim_type": "outer_only", "brim_width": "4", "brim_object_gap": "0.1"}
ARMS_EXTRA = {"sparse_infill_density": "100%", "sparse_infill_pattern": "rectilinear", "brim_type": "no_brim"}

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
}


def find_orca() -> Path:
    for c in ORCA_CANDIDATES:
        if c.is_file():
            return c
    on_path = shutil.which("orca-slicer")
    if on_path:
        return Path(on_path)
    sys.exit("orca-slicer.exe not found; unzip the OrcaSlicer portable build into %LOCALAPPDATA%\\Programs\\OrcaSlicer-portable")


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
    # The CLI's compatibility check compares the process's compatible_printers with the printer's
    # *system* name, which for from=="system" is simply its name. So present these as system presets.
    prof["from"] = "system"
    prof["version"] = "2.4.2.0"
    path.write_text(json.dumps(prof, indent=2), encoding="utf-8")
    print(f"  {path.name:22s} <- {' -> '.join(chain)}")


def build_profiles(orca: Path, nozzle: str) -> Path:
    n = NOZZLES[nozzle]
    out = HERE / "profiles" / f"{nozzle}mm"
    out.mkdir(parents=True, exist_ok=True)
    idx = load_vendor(orca)
    compat = {"compatible_printers": [n["machine"]]}
    common = {**PROCESS_COMMON, **n["speeds"], "layer_height": n["layer"], "initial_layer_print_height": n["layer"],
              "xy_hole_compensation": n["hole_comp"], **compat}
    print(f"profiles ({nozzle} mm nozzle):")
    write_profile(out / "machine_k1.json", flatten(n["machine"], idx), {}, n["machine"])
    proc = flatten(n["process"], idx)
    write_profile(out / "process_plates.json", proc, {**common, **n["plates"], **PLATES_EXTRA}, f"flydrone plates @K1 {nozzle}")
    write_profile(out / "process_arms.json", proc, {**common, **n["arms"], **ARMS_EXTRA}, f"flydrone arms @K1 {nozzle}")
    write_profile(out / "filament_petg.json", flatten(FILAMENT, idx),
                  {**FILAMENT_OVERRIDES, "filament_max_volumetric_speed": [n["max_vol"]], **compat}, f"flydrone PETG @K1 {nozzle}")
    return out


def run_slice(orca: Path, stl: Path, profiles: Path, process: str, out: Path) -> None:
    """Slice with Orca's CLI and take the G-code it writes next to the sliced 3mf."""
    work = out.parent / "_work"
    if work.exists():
        shutil.rmtree(work)
    work.mkdir(parents=True)
    log = out.with_suffix(".log")
    cmd = [
        str(orca), str(stl),
        "--load-settings", f"{profiles / process};{profiles / 'machine_k1.json'}",
        "--load-filaments", str(profiles / "filament_petg.json"),
        "--arrange", "0",
        "--slice", "0", "--export-3mf", "sliced.3mf",
        "--debug", "2", "--logfile", str(log),
        "--outputdir", str(work),
    ]
    print(f"\nslicing {stl.name} with {process} ...")
    r = subprocess.run(cmd, capture_output=True, text=True)
    produced = work / "plate_1.gcode"
    if r.returncode != 0 or not produced.exists():
        print("  FAILED (exit", r.returncode, ")")
        if log.exists():
            print("   " + "\n   ".join(log.read_text(errors="replace").splitlines()[-15:]))
        sys.exit(1)
    if out.exists():
        out.unlink()
    shutil.move(str(produced), str(out))
    shutil.rmtree(work, ignore_errors=True)
    log.unlink(missing_ok=True)


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
    print(f"  filament  : {grab(r'^; (?:total )?filament used \[g\]\s*=\s*(.+)$')} g")
    print(f"  nozzle    : {grab(r'^; nozzle_diameter = (.+)$')} mm, {grab(r'^; nozzle_temperature = (.+)$')} C (first layer {grab(r'^; nozzle_temperature_initial_layer = (.+)$')}), bed {grab(r'^; (?:hot|cool|textured|eng)_plate_temp = (.+)$')} C")
    print(f"  layers    : {grab(r'^; layer_height = (.+)$')} mm, line {grab(r'^; line_width = (.+)$')} mm, walls {grab(r'^; wall_loops = (.+)$')}, infill {grab(r'^; sparse_infill_density = (.+)$')} {grab(r'^; sparse_infill_pattern = (.+)$')}, brim {grab(r'^; brim_type = (.+)$')}")
    print(f"  fans      : part {grab(r'^; fan_min_speed = (.+)$')}-{grab(r'^; fan_max_speed = (.+)$')}%, aux {grab(r'^; additional_cooling_fan_speed = (.+)$')}%")
    if xs:
        ok = min(xs) >= 0 and max(xs) <= 220 and min(ys) >= 0 and max(ys) <= 220
        print(f"  extents   : X {min(xs):.1f}..{max(xs):.1f}  Y {min(ys):.1f}..{max(ys):.1f}  Z max {max(zs):.1f}  {'inside 220 bed' if ok else 'OUTSIDE BED'}")
    print(f"  macros    : START_PRINT {'START_PRINT' in text}, END_PRINT {'END_PRINT' in text}")


def main() -> None:
    nozzles = sys.argv[1:] or ["0.6"]
    orca = find_orca()
    print("orca-slicer:", orca)
    done = []
    for nozzle in nozzles:
        profiles = build_profiles(orca, nozzle)
        tag = f"{nozzle}nozzle"
        jobs = [
            ("plate_A_plates_legs_bracket.stl", "process_plates.json", f"flydrone_K1_{tag}_plateA_plates_legs_bracket_PETG.gcode"),
            ("plate_B_arms.stl", "process_arms.json", f"flydrone_K1_{tag}_plateB_arms_PETG.gcode"),
        ]
        for stl, proc, out in jobs:
            run_slice(orca, STL_DIR / stl, profiles, proc, GCODE / out)
            done.append(GCODE / out)
    for g in done:
        summarize(g)


if __name__ == "__main__":
    main()
