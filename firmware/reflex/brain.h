// The seam between "senses" and "muscles".
//
// Anything that turns a Retina into a DescendingCommand is a Brain. The hand-written
// ReflexBrain lives in reflex_brain.h. A connectome-driven brain (LIF network of real fly
// neurons) will implement the same interface, so the drone, the camera and the flight
// controller link do not care which one is running.
#pragma once
#include <stdint.h>
#include "config.h"

struct Retina {
  float L[RET_H][RET_W];   // luminance 0..1, row 0 = top, col 0 = left. Left eye = left half.
  uint32_t t_ms = 0;       // capture time
  uint32_t frame_no = 0;
};

// What the fly's descending neurons would carry down to the motor centres, expressed as
// body-frame velocities for the flight controller. NED body frame: x forward, y right, z DOWN.
struct DescendingCommand {
  float yaw_rate = 0.f;   // rad/s, + = clockwise seen from above
  float vx = 0.f;         // m/s forward
  float vy = 0.f;         // m/s right
  float vz = 0.f;         // m/s down (negative = climb)
  bool escape = false;    // giant-fiber burst in progress
  // diagnostics for the telemetry line
  float dbg_yaw_sig = 0.f, dbg_loom = 0.f, dbg_flow_l = 0.f, dbg_flow_r = 0.f;
};

class Brain {
 public:
  virtual ~Brain() {}
  virtual void reset() = 0;
  virtual DescendingCommand step(const Retina& r, float dt_s) = 0;
  virtual const char* name() const = 0;
};
