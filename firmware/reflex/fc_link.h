// MAVLink 2 link to an ArduPilot flight controller. Hand-rolled framing for the two messages
// we need, so there is no dependency on the (huge) generated MAVLink library.
//
//   HEARTBEAT (id 0)                       so the FC knows the companion is alive
//   SET_POSITION_TARGET_LOCAL_NED (id 84)  body-frame velocity + yaw rate, GUIDED mode
//
// The Pico never arms, never changes mode. You do that from the transmitter. The FC only
// follows these setpoints while it is in GUIDED, and your RC kill switch always wins.
#pragma once
#include <Arduino.h>
#include "brain.h"

class FcLink {
 public:
  void begin(HardwareSerial& serial, uint32_t baud);
  void sendHeartbeat();
  void sendVelocity(const DescendingCommand& c, uint32_t t_boot_ms);
  void sendHover(uint32_t t_boot_ms);   // all-zero velocity: hold position
  uint32_t framesSent() const { return frames_; }

 private:
  HardwareSerial* ser_ = nullptr;
  uint8_t seq_ = 0;
  uint32_t frames_ = 0;
  void sendFrame(uint32_t msgid, const uint8_t* payload, uint8_t len, uint8_t crc_extra);
  static void crcAccumulate(uint8_t b, uint16_t& crc);
};
