// Runs the firmware's ReflexBrain on the PC. Frames come in on stdin as raw 8-bit
// RET_W x RET_H luminance, one after another; one line of outputs per frame goes to stdout.
// tools/host_check.py feeds it the same stimuli as the Python reference and compares.
//
//   uv run python -m ziglang c++ -O2 -std=c++17 -Ifirmware/reflex tools/host_test.cpp \
//       firmware/reflex/reflex_brain.cpp -o build/host_test.exe
#include <cstdio>
#include <cstdint>
#ifdef _WIN32
#include <io.h>
#include <fcntl.h>
#endif
#include "reflex_brain.h"

int main() {
#ifdef _WIN32
  _setmode(_fileno(stdin), _O_BINARY);   // text mode would treat byte 0x1A as end-of-file
#endif
  ReflexBrain brain;
  brain.reset();
  Retina r;
  uint8_t buf[RET_H * RET_W];
  const float dt = 1.0f / 30.0f;
  uint32_t n = 0;
  while (fread(buf, 1, sizeof(buf), stdin) == sizeof(buf)) {
    for (int y = 0; y < RET_H; y++)
      for (int x = 0; x < RET_W; x++) r.L[y][x] = buf[y * RET_W + x] * (1.0f / 255.0f);
    r.frame_no = ++n;
    DescendingCommand c = brain.step(r, dt);
    printf("%.6f %.6f %.6f %.6f %d %.6f %.6f %.6f %.6f\n",
           c.yaw_rate, c.vx, c.vy, c.vz, c.escape ? 1 : 0, c.dbg_loom, c.dbg_yaw_sig, c.dbg_flow_l, c.dbg_flow_r);
  }
  return 0;
}
