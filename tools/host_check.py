"""
Cross-check: the firmware's C++ ReflexBrain against the Python reference, on identical
8-bit frames of every synthetic stimulus. Compiles tools/host_test.cpp with zig (installed as
a dev dependency) so no separate compiler is needed.

    uv run python tools/host_check.py
"""
from __future__ import annotations

import subprocess
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
import reflex_sim as rs  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
EXE = ROOT / "build" / ("host_test.exe" if sys.platform == "win32" else "host_test")


def build() -> None:
    EXE.parent.mkdir(exist_ok=True)
    cmd = [sys.executable, "-m", "ziglang", "c++", "-O2", "-std=c++17", "-w", "-Ifirmware/reflex",
           "tools/host_test.cpp", "firmware/reflex/reflex_brain.cpp", "-o", str(EXE)]
    print("compiling:", " ".join(cmd[2:]))
    subprocess.run(cmd, cwd=ROOT, check=True)


def main() -> None:
    build()
    rng = np.random.default_rng(1)
    stimuli = [
        ("grating right", lambda t: rs.grating(t, +20)),
        ("grating left", lambda t: rs.grating(t, -20)),
        ("looming disc", lambda t: rs.looming_disc(t, 1.0, 0.6)),
        ("translating blob", lambda t: rs.translating_blob(t, 25)),
        ("corridor left near", lambda t: rs.corridor(t, "left")),
        ("corridor right near", lambda t: rs.corridor(t, "right")),
        ("noise", lambda t: rs.noise(rng)),
    ]
    n = int(3.0 * rs.FPS)
    worst = 0.0
    all_ok = True
    for name, fn in stimuli:
        frames = np.stack([np.clip(np.round(fn(i * rs.DT) * 255), 0, 255).astype(np.uint8) for i in range(n)])
        # python reference on the same quantised frames
        b = rs.ReflexBrain(rs.Params())
        ref = []
        for f in frames:
            c = b.step(f.astype(np.float64) / 255.0, rs.DT)
            ref.append((c.yaw_rate, c.vx, c.vy, c.vz, 1 if c.escape else 0, c.loom, c.yaw_sig, c.flow_l, c.flow_r))
        ref = np.array(ref)
        out = subprocess.run([str(EXE)], input=frames.tobytes(), capture_output=True, check=True)
        cpp = np.array([[float(v) for v in line.split()] for line in out.stdout.decode().splitlines()])
        assert cpp.shape == ref.shape, (cpp.shape, ref.shape)
        diff = np.abs(cpp - ref).max(axis=0)
        esc_same = (cpp[:, 4] == ref[:, 4]).all()
        ok = diff[[0, 2, 5, 6, 7, 8]].max() < 2e-3 and esc_same
        all_ok &= bool(ok)
        worst = max(worst, float(diff[[0, 2, 5, 6, 7, 8]].max()))
        print(f"{'OK  ' if ok else 'DIFF'} {name:22s} max|diff| yaw={diff[0]:.1e} vy={diff[2]:.1e} loom={diff[5]:.1e} "
              f"yawsig={diff[6]:.1e} flow={max(diff[7], diff[8]):.1e} escapes py={int((np.diff(ref[:,4])>0).sum())} cpp={int((np.diff(cpp[:,4])>0).sum())}")
    print("worst difference", f"{worst:.2e}", "-> C++ matches Python" if all_ok else "-> MISMATCH")
    sys.exit(0 if all_ok else 1)


if __name__ == "__main__":
    main()
