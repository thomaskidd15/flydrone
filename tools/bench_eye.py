"""
Desk test without a camera or a drone. Streams synthetic retina frames to the Pico over USB
(build the firmware with EYE_SOURCE = EyeSource::UsbSerial) and prints the telemetry lines it
sends back, so you can watch the reflexes react.

    uv run python tools/bench_eye.py --port COM5 --stim loom
    stimuli: grating_right, grating_left, loom, blob, corridor_left, corridor_right, noise, cycle

Frame protocol: 'F' 'R' then 32*24 bytes of luminance, row-major from the top-left, ~30 fps.
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np
import serial

sys.path.insert(0, str(Path(__file__).parent))
import reflex_sim as rs  # noqa: E402

STIMS = {
    "grating_right": lambda t, rng: rs.grating(t, +20),
    "grating_left": lambda t, rng: rs.grating(t, -20),
    "loom": lambda t, rng: rs.looming_disc(t % 3.0, 1.0, 0.6),
    "blob": lambda t, rng: rs.translating_blob(t % 2.0, 25),
    "corridor_left": lambda t, rng: rs.corridor(t, "left"),
    "corridor_right": lambda t, rng: rs.corridor(t, "right"),
    "noise": lambda t, rng: rs.noise(rng),
}


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True, help="Pico's USB serial port, e.g. COM5")
    ap.add_argument("--stim", default="cycle", choices=list(STIMS) + ["cycle"])
    ap.add_argument("--fps", type=float, default=30.0)
    ap.add_argument("--seconds", type=float, default=0, help="stop after this long (0 = run until Ctrl-C)")
    a = ap.parse_args()

    rng = np.random.default_rng(1)
    names = list(STIMS) if a.stim == "cycle" else [a.stim]
    ser = serial.Serial(a.port, 115200, timeout=0)
    print(f"streaming to {a.port}; Ctrl-C to stop")
    t0 = time.time()
    period = 1.0 / a.fps
    next_t = time.time()
    buf = b""
    try:
        while True:
            now = time.time()
            t = now - t0
            if a.seconds and t > a.seconds:
                break
            name = names[int(t // 4.0) % len(names)]
            frame = np.clip(np.round(STIMS[name](t, rng) * 255), 0, 255).astype(np.uint8)
            ser.write(b"FR" + frame.tobytes())
            buf += ser.read(4096)
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                text = line.decode(errors="replace").strip()
                if text.startswith("T "):
                    print(f"[{name:14s}] {text}")
            next_t += period
            sleep = next_t - time.time()
            if sleep > 0:
                time.sleep(sleep)
            else:
                next_t = time.time()
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()


if __name__ == "__main__":
    main()
