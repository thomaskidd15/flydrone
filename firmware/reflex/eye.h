// An Eye fills a Retina. Two of them:
//   EyeHM01B0    the Arducam HM01B0 camera, via the PicoHM01B0 PIO/DMA driver
//   EyeUsbSerial frames pushed over USB by tools/bench_eye.py, for testing on the desk
#pragma once
#include <Arduino.h>
#include "brain.h"
#include "src/PicoHM01B0/PicoHM01B0.h"

class Eye {
 public:
  virtual ~Eye() {}
  virtual bool begin() = 0;
  // Non-blocking. Returns true when a new frame has been written into `out`.
  virtual bool poll(Retina& out) = 0;
  virtual const char* name() const = 0;
};

class EyeHM01B0 : public Eye {
 public:
  bool begin() override;
  bool poll(Retina& out) override;
  const char* name() const override { return "hm01b0"; }

 private:
  static constexpr int COLS = 164, ROWS = 122;           // 2x2 binning + QVGA mode
  PicoHM01B0 cam_;
  uint8_t buf_[2][ROWS * COLS] __attribute__((aligned(4)));
  int capturing_ = 0;
  uint32_t frame_no_ = 0;
  void downsample(const uint8_t* frame, Retina& out);
};

class EyeUsbSerial : public Eye {
 public:
  bool begin() override;
  bool poll(Retina& out) override;
  const char* name() const override { return "usb"; }

 private:
  // protocol: 'F' 'R' then RET_W*RET_H bytes of luminance, row-major from the top-left
  enum { WAIT_F, WAIT_R, PAYLOAD } state_ = WAIT_F;
  uint8_t pix_[RET_H * RET_W];
  int got_ = 0;
  uint32_t frame_no_ = 0;
};
