#include "eye.h"
#include "config.h"

// ---------------------------------------------------------------- camera
bool EyeHM01B0::begin() {
  PicoHM01B0_config cfg;
  cfg.i2c_dat_gpio = CAM_SDA;
  cfg.i2c_clk_gpio = CAM_SCL;
  cfg.vsync_gpio = CAM_VSYNC;
  cfg.d0_gpio = CAM_D0;
  cfg.pclk_gpio = CAM_PCLK;
  cfg.mclk_gpio = CAM_MCLK;
  cfg.mclk_freq = 0;
  cfg.bus_4bit = false;
  cfg.flip_horizontal = CAM_FLIP_H;
  cfg.flip_vertical = CAM_FLIP_V;
  if (!cam_.begin(cfg)) return false;
  cam_.start_streaming(CAM_FPS, /*binning_2x2=*/true, /*qvga_mode=*/true);   // 164 x 122
  cam_.set_auto_exposure();
  capturing_ = 0;
  cam_.start_capture(buf_[capturing_]);
  return true;
}

bool EyeHM01B0::poll(Retina& out) {
  if (!cam_.is_frame_ready()) return false;
  const uint8_t* done = buf_[capturing_];
  capturing_ ^= 1;
  cam_.start_capture(buf_[capturing_]);      // next frame streams in while we process this one
  downsample(done, out);
  out.t_ms = millis();
  out.frame_no = ++frame_no_;
  return true;
}

// 164 x 122 -> 32 x 24 by averaging 5 x 5 blocks (2 px trimmed left/right, 1 px top/bottom)
void EyeHM01B0::downsample(const uint8_t* frame, Retina& out) {
  constexpr int BX = 5, BY = 5;
  constexpr int OX = (COLS - RET_W * BX) / 2;   // 2
  constexpr int OY = (ROWS - RET_H * BY) / 2;   // 1
  constexpr float scale = 1.0f / (BX * BY * 255.0f);
  for (int ry = 0; ry < RET_H; ry++) {
    for (int rx = 0; rx < RET_W; rx++) {
      uint32_t acc = 0;
      const int y0 = OY + ry * BY, x0 = OX + rx * BX;
      for (int y = 0; y < BY; y++) {
        const uint8_t* row = frame + (y0 + y) * COLS + x0;
        for (int x = 0; x < BX; x++) acc += row[x];
      }
      out.L[ry][rx] = acc * scale;
    }
  }
}

// ---------------------------------------------------------------- usb serial
bool EyeUsbSerial::begin() {
  state_ = WAIT_F;
  got_ = 0;
  return true;
}

bool EyeUsbSerial::poll(Retina& out) {
  while (Serial.available()) {
    int b = Serial.read();
    if (b < 0) break;
    switch (state_) {
      case WAIT_F:
        if (b == 'F') state_ = WAIT_R;
        break;
      case WAIT_R:
        state_ = (b == 'R') ? PAYLOAD : WAIT_F;
        got_ = 0;
        break;
      case PAYLOAD:
        pix_[got_++] = (uint8_t)b;
        if (got_ >= RET_W * RET_H) {
          state_ = WAIT_F;
          for (int y = 0; y < RET_H; y++)
            for (int x = 0; x < RET_W; x++) out.L[y][x] = pix_[y * RET_W + x] * (1.0f / 255.0f);
          out.t_ms = millis();
          out.frame_no = ++frame_no_;
          return true;
        }
        break;
    }
  }
  return false;
}
