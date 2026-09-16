#include "reflex_brain.h"
#include <math.h>

// LPLC2 array geometry (mirror of UNIT_* in tools/reflex_sim.py)
static const int UNIT_CX[3] = { 8, 16, 24 };
static const int UNIT_CY[3] = { 6, 12, 18 };
static const int UNIT_HW = 8, UNIT_HH = 6;

static inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
static inline float minf(float a, float b) { return a < b ? a : b; }

void ReflexBrain::reset() {
  primed_ = false;
  loom_lp_ = escape_left_ = refractory_left_ = yaw_filt_ = vy_filt_ = 0.f;
}

// For each unit: split its window into four quadrants around the unit centre, average the
// component of motion pointing away from the centre in each, take the smallest of the four
// (all four must agree), then take the best unit. Never negative.
float ReflexBrain::lplc2Array() const {
  float best = 0.f;
  for (int iy = 0; iy < 3; iy++) {
    const int cy = UNIT_CY[iy];
    for (int ix = 0; ix < 3; ix++) {
      const int cx = UNIT_CX[ix];
      float qsum[4] = { 0, 0, 0, 0 };
      int qn[4] = { 0, 0, 0, 0 };
      // horizontal samples: rh_[y][x] sits at (x + 0.5, y)
      for (int y = 0; y < RET_H; y++) {
        if (fabsf((float)y - cy) > UNIT_HH) continue;
        for (int x = 0; x < RET_W - 1; x++) {
          const float px = x + 0.5f;
          if (fabsf(px - cx) > UNIT_HW) continue;
          const bool right = px >= cx, below = y >= cy;
          const int q = (below ? 2 : 0) + (right ? 1 : 0);
          qsum[q] += rh_[y][x] * (right ? 1.f : -1.f);
          qn[q]++;
        }
      }
      // vertical samples: rv_[y][x] sits at (x, y + 0.5)
      for (int y = 0; y < RET_H - 1; y++) {
        const float py = y + 0.5f;
        if (fabsf(py - cy) > UNIT_HH) continue;
        for (int x = 0; x < RET_W; x++) {
          if (fabsf((float)x - cx) > UNIT_HW) continue;
          const bool right = x >= cx, below = py >= cy;
          const int q = (below ? 2 : 0) + (right ? 1 : 0);
          qsum[q] += rv_[y][x] * (below ? 1.f : -1.f);
          qn[q]++;
        }
      }
      float unit = 1e30f;
      for (int q = 0; q < 4; q++) unit = minf(unit, qsum[q] / (qn[q] > 0 ? qn[q] : 1));
      if (unit > best) best = unit;
    }
  }
  return best;
}

DescendingCommand ReflexBrain::step(const Retina& r, float dt) {
  if (!primed_) {
    for (int y = 0; y < RET_H; y++)
      for (int x = 0; x < RET_W; x++) { hp_[y][x] = r.L[y][x]; dl_[y][x] = 0.f; }
    primed_ = true;
  }
  const float a_hp = dt / TAU_HP, a_dl = dt / TAU_DELAY;

  // photoreceptor / lamina: contrast = high-passed luminance; then the delay arm
  float energy = 0.f;
  for (int y = 0; y < RET_H; y++) {
    for (int x = 0; x < RET_W; x++) {
      const float L = r.L[y][x];
      hp_[y][x] += (L - hp_[y][x]) * a_hp;
      const float c = L - hp_[y][x];
      c_[y][x] = c;
      dl_[y][x] += (c - dl_[y][x]) * a_dl;
      energy += c * c;
    }
  }
  energy = energy / (RET_H * RET_W) + CONTRAST_EPS;

  // Reichardt correlators
  float rh_sum = 0.f;
  for (int y = 0; y < RET_H; y++)
    for (int x = 0; x < RET_W - 1; x++) {
      const float v = dl_[y][x] * c_[y][x + 1] - c_[y][x] * dl_[y][x + 1];
      rh_[y][x] = v;
      rh_sum += v;
    }
  for (int y = 0; y < RET_H - 1; y++)
    for (int x = 0; x < RET_W; x++)
      rv_[y][x] = dl_[y][x] * c_[y + 1][x] - c_[y][x] * dl_[y + 1][x];

  // optomotor: whole-field horizontal motion (HS cells)
  const float yaw_sig = rh_sum / (RET_H * (RET_W - 1)) / energy;

  // corridor centring: total flow per eye. Same column splits as the numpy version:
  // horizontal samples split at (W-1)//2, vertical samples at W//2.
  const int hsplit = (RET_W - 1) / 2, vsplit = RET_W / 2;
  float mh_l = 0.f, mh_r = 0.f, mv_l = 0.f, mv_r = 0.f;
  for (int y = 0; y < RET_H; y++)
    for (int x = 0; x < RET_W - 1; x++) (x < hsplit ? mh_l : mh_r) += fabsf(rh_[y][x]);
  for (int y = 0; y < RET_H - 1; y++)
    for (int x = 0; x < RET_W; x++) (x < vsplit ? mv_l : mv_r) += fabsf(rv_[y][x]);
  const float flow_l = (mh_l / (RET_H * hsplit) + mv_l / ((RET_H - 1) * vsplit)) / energy;
  const float flow_r = (mh_r / (RET_H * (RET_W - 1 - hsplit)) + mv_r / ((RET_H - 1) * (RET_W - vsplit))) / energy;

  // looming
  const float loom_raw = lplc2Array() / energy;
  loom_lp_ += (loom_raw - loom_lp_) * minf(1.f, dt / TAU_LOOM);
  const float loom = loom_lp_;

  DescendingCommand cmd;
  cmd.dbg_yaw_sig = yaw_sig; cmd.dbg_loom = loom; cmd.dbg_flow_l = flow_l; cmd.dbg_flow_r = flow_r;

  // giant fiber: threshold, fixed-duration escape, refractory
  refractory_left_ = refractory_left_ > dt ? refractory_left_ - dt : 0.f;
  if (escape_left_ > 0.f) {
    escape_left_ -= dt;
  } else if (loom > LOOM_THRESH && refractory_left_ <= 0.f) {
    escape_left_ = ESCAPE_S;
    refractory_left_ = REFRACTORY_S + ESCAPE_S;
  }
  if (escape_left_ > 0.f) {
    cmd.escape = true;
    cmd.vz = -MAX_V;      // climb
    cmd.vx = -MAX_V;      // back off
    cmd.vy = 0.f;
    cmd.yaw_rate = 0.f;
    yaw_filt_ = vy_filt_ = 0.f;
    return cmd;
  }

  // descending-neuron low-pass on the steady reflexes
  yaw_filt_ += (K_YAW * yaw_sig - yaw_filt_) * minf(1.f, dt / YAW_FILT_S);
  vy_filt_ += (K_CENTER * (flow_l - flow_r) - vy_filt_) * minf(1.f, dt / VY_FILT_S);
  cmd.yaw_rate = clampf(yaw_filt_, -MAX_YAW_RATE, MAX_YAW_RATE);
  cmd.vy = clampf(vy_filt_, -MAX_V, MAX_V);
  cmd.vx = CRUISE_VX;
  cmd.vz = 0.f;
  return cmd;
}
