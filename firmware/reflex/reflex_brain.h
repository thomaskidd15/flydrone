// Hand-written fly reflexes. A straight port of tools/reflex_sim.py; keep the two in step.
//
//   photoreceptor + lamina   per-pixel high-pass (contrast), then a low-pass "delay" arm
//   T4/T5 (Reichardt EMD)    delayed(x) * now(x+1) - now(x) * delayed(x+1)  -> local motion
//   HS cells -> DNs          whole-field horizontal motion  -> optomotor yaw
//   left/right flow balance  more flow on one eye -> sidestep away (corridor centring)
//   LPLC2 array -> giant fiber  outward motion in all four quadrants of a unit -> ESCAPE
//
// Pure C++, no Arduino dependency, so the same file is compiled for the host cross-check.
#pragma once
#include "brain.h"

class ReflexBrain : public Brain {
 public:
  void reset() override;
  DescendingCommand step(const Retina& r, float dt_s) override;
  const char* name() const override { return "reflex"; }

 private:
  float hp_[RET_H][RET_W];        // slow luminance estimate (adaptation)
  float dl_[RET_H][RET_W];        // delayed contrast
  float c_[RET_H][RET_W];         // contrast
  float rh_[RET_H][RET_W - 1];    // horizontal EMD, + = image moves right
  float rv_[RET_H - 1][RET_W];    // vertical EMD,   + = image moves down
  float loom_lp_ = 0.f;
  float escape_left_ = 0.f;
  float refractory_left_ = 0.f;
  float yaw_filt_ = 0.f;
  float vy_filt_ = 0.f;
  bool primed_ = false;

  float lplc2Array() const;
};
