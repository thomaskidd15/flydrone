"""
Reference implementation of the fly reflexes, in numpy, with synthetic stimuli.

This is the same math as firmware/reflex/reflex_brain.cpp. Run it to see what the drone
would do, and to check any change to the constants before flashing:

    uv run python tools/reflex_sim.py            # writes tools/reflex_sim.png

Circuits imitated (hand-written, NOT the connectome):
  photoreceptor + lamina   per-pixel high-pass (contrast), then a low-pass "delay" arm
  T4/T5 (Reichardt EMD)    delayed(x) * now(x+1) - now(x) * delayed(x+1)  -> local motion
  HS/VS cells -> DNs       whole-field horizontal motion  -> optomotor yaw
  left/right flow balance  more flow on one eye -> sidestep away (corridor centring)
  LPLC2 + LC4 -> giant fiber  outward motion in all four quadrants + growing dark blob -> ESCAPE
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

W, H = 32, 24          # retina, pixels (camera frame block-averaged down to this)
FPS = 30.0
DT = 1.0 / FPS


@dataclass
class Params:
    tau_hp: float = 0.30     # s, contrast high-pass (photoreceptor adaptation)
    tau_delay: float = 0.04  # s, Reichardt delay arm
    eps: float = 1e-3        # contrast-energy floor for normalisation
    k_yaw: float = 4.0       # yaw rate per unit normalised horizontal motion  [rad/s]
    k_center: float = 3.0    # sideways velocity per unit left-right flow imbalance [m/s]
    tau_loom: float = 0.06   # s, low-pass on the looming drive (kills single-frame spikes)
    loom_thresh: float = 0.08
    escape_s: float = 0.4
    refractory_s: float = 1.5
    max_yaw: float = 1.0     # rad/s
    max_v: float = 0.5       # m/s


@dataclass
class Command:
    yaw_rate: float = 0.0    # rad/s, + = clockwise seen from above (NED)
    vx: float = 0.0          # m/s forward
    vy: float = 0.0          # m/s right
    vz: float = 0.0          # m/s down (NED: negative = up)
    escape: bool = False
    # diagnostics
    yaw_sig: float = 0.0
    loom: float = 0.0
    flow_l: float = 0.0
    flow_r: float = 0.0


@dataclass
class ReflexBrain:
    p: Params = field(default_factory=Params)
    hp: np.ndarray = field(default_factory=lambda: np.zeros((H, W)))
    dl: np.ndarray = field(default_factory=lambda: np.zeros((H, W)))
    loom_lp: float = 0.0
    escape_left: float = 0.0
    refractory_left: float = 0.0
    yaw_filt: float = 0.0
    vy_filt: float = 0.0
    primed: bool = False

    def step(self, L: np.ndarray, dt: float = DT) -> Command:
        p = self.p
        L = L.astype(np.float64)
        if not self.primed:              # start with no spurious motion on the first frame
            self.hp[:] = L
            self.dl[:] = 0.0
            self.primed = True
        # photoreceptor / lamina: contrast = high-passed luminance
        self.hp += (L - self.hp) * (dt / p.tau_hp)
        c = L - self.hp
        # delay arm
        self.dl += (c - self.dl) * (dt / p.tau_delay)
        d = self.dl
        # Reichardt correlators. rh > 0: image moves right. rv > 0: image moves down.
        rh = d[:, :-1] * c[:, 1:] - c[:, :-1] * d[:, 1:]        # H x (W-1)
        rv = d[:-1, :] * c[1:, :] - c[:-1, :] * d[1:, :]        # (H-1) x W
        energy = float((c * c).mean()) + p.eps

        # optomotor: whole-field horizontal motion (HS cells)
        yaw_sig = float(rh.mean()) / energy

        # corridor centring: total flow per eye (left half / right half of the field)
        mag_h = np.abs(rh)
        mag_v = np.abs(rv)
        flow_l = (mag_h[:, : (W - 1) // 2].mean() + mag_v[:, : W // 2].mean()) / energy
        flow_r = (mag_h[:, (W - 1) // 2 :].mean() + mag_v[:, W // 2 :].mean()) / energy

        # LPLC2-like looming detectors: an array of units, each with its own receptive-field
        # centre, each responding only when motion points AWAY from its centre in all four of
        # its quadrants. A translating object drives at most two quadrants; an approaching one
        # drives all four of whichever unit it is centred on. Output = the best unit.
        loom_raw = lplc2_array(rh, rv) / energy
        self.loom_lp += (loom_raw - self.loom_lp) * min(1.0, dt / p.tau_loom)
        loom = self.loom_lp

        # giant fiber: threshold, fixed-duration escape, refractory
        cmd = Command(yaw_sig=yaw_sig, loom=loom, flow_l=flow_l, flow_r=flow_r)
        self.refractory_left = max(0.0, self.refractory_left - dt)
        if self.escape_left > 0.0:
            self.escape_left -= dt
        elif loom > p.loom_thresh and self.refractory_left <= 0.0:
            self.escape_left = p.escape_s
            self.refractory_left = p.refractory_s + p.escape_s

        if self.escape_left > 0.0:
            cmd.escape = True
            cmd.vz = -p.max_v          # up
            cmd.vx = -p.max_v          # back
            cmd.yaw_rate = 0.0
            self.yaw_filt = 0.0
            self.vy_filt = 0.0
            return cmd

        # smooth the reflex outputs a little (descending neuron low-pass)
        self.yaw_filt += (p.k_yaw * yaw_sig - self.yaw_filt) * min(1.0, dt / 0.1)
        self.vy_filt += (p.k_center * (flow_l - flow_r) - self.vy_filt) * min(1.0, dt / 0.2)
        cmd.yaw_rate = float(np.clip(self.yaw_filt, -p.max_yaw, p.max_yaw))
        cmd.vy = float(np.clip(self.vy_filt, -p.max_v, p.max_v))
        return cmd


# LPLC2 array geometry (mirror of firmware/reflex/reflex_brain.cpp)
UNIT_CX = (8, 16, 24)
UNIT_CY = (6, 12, 18)
UNIT_HW, UNIT_HH = 8, 6      # half window: each unit looks at a 16 x 12 patch around its centre


def lplc2_array(rh: np.ndarray, rv: np.ndarray) -> float:
    """rh[y][x] samples horizontal motion at (x+0.5, y); rv[y][x] vertical motion at (x, y+0.5).
    Outward = motion component pointing away from the unit centre. Returns max over units of
    min over that unit's four quadrant means, clipped at 0."""
    xh = np.arange(W - 1) + 0.5
    yh = np.arange(H)
    xv = np.arange(W)
    yv = np.arange(H - 1) + 0.5
    best = 0.0
    for cy in UNIT_CY:
        for cx in UNIT_CX:
            q_sum = np.zeros(4)
            q_n = np.zeros(4)
            # horizontal samples
            mh = (np.abs(xh - cx)[None, :] <= UNIT_HW) & (np.abs(yh - cy)[:, None] <= UNIT_HH)
            sx = np.where(xh >= cx, 1.0, -1.0)[None, :]
            qidx = (np.where(yh >= cy, 2, 0)[:, None] + np.where(xh >= cx, 1, 0)[None, :])
            out = rh * sx
            for k in range(4):
                sel = mh & (qidx == k)
                q_sum[k] += out[sel].sum(); q_n[k] += sel.sum()
            # vertical samples
            mv = (np.abs(xv - cx)[None, :] <= UNIT_HW) & (np.abs(yv - cy)[:, None] <= UNIT_HH)
            sy = np.where(yv >= cy, 1.0, -1.0)[:, None]
            qidx = (np.where(yv >= cy, 2, 0)[:, None] + np.where(xv >= cx, 1, 0)[None, :])
            out = rv * sy
            for k in range(4):
                sel = mv & (qidx == k)
                q_sum[k] += out[sel].sum(); q_n[k] += sel.sum()
            unit = float(np.min(q_sum / np.maximum(q_n, 1)))
            best = max(best, unit)
    return best


# ------------------------------------------------------------------ stimuli
def grating(t: float, speed_px_s: float, period_px: float = 8.0, contrast: float = 0.5) -> np.ndarray:
    x = np.arange(W)[None, :].repeat(H, 0)
    phase = 2 * math.pi * (x - speed_px_s * t) / period_px
    return 0.5 + contrast * 0.5 * np.sin(phase)


def looming_disc(t: float, t0: float, dur: float, r0: float = 1.0, r1: float = 14.0) -> np.ndarray:
    L = np.full((H, W), 0.8)
    if t < t0:
        r = r0
    else:
        # approach at constant speed: radius grows like 1/(time to collision)
        f = min(1.0, (t - t0) / dur)
        r = r0 + (r1 - r0) * (f ** 2)
    yy, xx = np.mgrid[0:H, 0:W]
    L[(xx - (W - 1) / 2) ** 2 + (yy - (H - 1) / 2) ** 2 <= r * r] = 0.1
    return L


def translating_blob(t: float, speed_px_s: float, r: float = 3.0) -> np.ndarray:
    L = np.full((H, W), 0.8)
    cx = -4 + speed_px_s * t
    yy, xx = np.mgrid[0:H, 0:W]
    L[(xx - cx) ** 2 + (yy - (H - 1) / 2) ** 2 <= r * r] = 0.1
    return L


def corridor(t: float, near_side: str, v_px_s: float = 3.0) -> np.ndarray:
    """Textured walls; the near wall moves faster across the eye than the far wall."""
    L = np.full((H, W), 0.5)
    x = np.arange(W)
    for side, sl in (("left", slice(0, W // 2)), ("right", slice(W // 2, W))):
        v = v_px_s * (2.5 if side == near_side else 1.0)
        v = -v if side == "left" else v      # front-to-back flow on both sides
        L[:, sl] = 0.5 + 0.25 * np.sign(np.sin(2 * math.pi * (x[sl] - v * t) / 6.0))
    return L


def noise(rng: np.random.Generator) -> np.ndarray:
    return rng.uniform(0.3, 0.7, (H, W))


def run(name: str, frames, seconds: float, p: Params | None = None):
    b = ReflexBrain(p or Params())
    t = 0.0
    rows = []
    n = int(seconds * FPS)
    for i in range(n):
        L = frames(t)
        cmd = b.step(L)
        rows.append((t, cmd.yaw_rate, cmd.vy, cmd.loom, 1.0 if cmd.escape else 0.0, cmd.yaw_sig))
        t += DT
    return name, np.array(rows)


def main() -> None:
    rng = np.random.default_rng(1)
    runs = [
        run("grating moving RIGHT 20 px/s  (expect yaw +)", lambda t: grating(t, +20), 3.0),
        run("grating moving LEFT 20 px/s   (expect yaw -)", lambda t: grating(t, -20), 3.0),
        run("looming disc at t=1s          (expect ESCAPE, no yaw)", lambda t: looming_disc(t, 1.0, 0.6), 3.0),
        run("blob translating 25 px/s      (expect NO escape)", lambda t: translating_blob(t, 25), 3.0),
        run("corridor, LEFT wall near      (expect vy + = move right)", lambda t: corridor(t, "left"), 3.0),
        run("corridor, RIGHT wall near     (expect vy - = move left)", lambda t: corridor(t, "right"), 3.0),
        run("random noise                  (expect nothing)", lambda t: noise(rng), 3.0),
    ]
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(len(runs), 1, figsize=(11, 2.1 * len(runs)), sharex=True)
    ok_all = True
    for ax, (name, r) in zip(axes, runs):
        t, yaw, vy, loom, esc, _ = r.T
        ax.plot(t, yaw, label="yaw_rate [rad/s]")
        ax.plot(t, vy, label="vy [m/s]")
        ax.plot(t, loom, label="loom drive", alpha=0.7)
        ax.fill_between(t, 0, esc * 1.0, color="red", alpha=0.25, label="ESCAPE")
        ax.axhline(Params().loom_thresh, color="k", ls=":", lw=0.8)
        ax.set_ylim(-1.2, 1.2); ax.grid(alpha=0.3); ax.set_title(name, fontsize=9, loc="left")
        # pass/fail
        late = r[int(1.0 * FPS):]
        if "RIGHT 20" in name: res = late[:, 1].mean() > 0.2
        elif "LEFT 20" in name: res = late[:, 1].mean() < -0.2
        elif "looming" in name: res = r[:, 4].max() > 0 and abs(r[:, 1]).max() < 0.3
        elif "translating" in name: res = r[:, 4].max() == 0
        elif "LEFT wall" in name: res = late[:, 2].mean() > 0.05
        elif "RIGHT wall" in name: res = late[:, 2].mean() < -0.05
        else: res = r[:, 4].max() == 0 and abs(r[:, 1]).max() < 0.15
        ok_all &= bool(res)
        ax.text(0.99, 0.85, "PASS" if res else "FAIL", transform=ax.transAxes, ha="right",
                color="green" if res else "red", fontweight="bold")
        print(f"{'PASS' if res else 'FAIL'}  {name}   yaw_mean={late[:,1].mean():+.2f} vy_mean={late[:,2].mean():+.2f} loom_peak={r[:,3].max():.3f} escapes={int((np.diff(r[:,4])>0).sum())}")
    axes[0].legend(loc="upper right", fontsize=7, ncol=4)
    axes[-1].set_xlabel("time [s]")
    fig.tight_layout()
    out = Path(__file__).with_suffix(".png")
    fig.savefig(out, dpi=90)
    print("plot ->", out, " ALL PASS" if ok_all else " SOME FAILED")


if __name__ == "__main__":
    main()
