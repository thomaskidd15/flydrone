#include "fc_link.h"
#include <string.h>
#include "config.h"

// MAVLink message ids and their CRC_EXTRA bytes (from the generated headers in
// mavlink/c_library_v2: minimal/mavlink_msg_heartbeat.h, common/mavlink_msg_set_position_target_local_ned.h)
static constexpr uint32_t MSG_HEARTBEAT = 0;
static constexpr uint8_t  CRC_HEARTBEAT = 50;
static constexpr uint32_t MSG_SET_POS_TARGET_LOCAL_NED = 84;
static constexpr uint8_t  CRC_SET_POS_TARGET_LOCAL_NED = 143;

static constexpr uint8_t MAV_TYPE_ONBOARD_CONTROLLER = 18;
static constexpr uint8_t MAV_AUTOPILOT_INVALID = 8;
static constexpr uint8_t MAV_STATE_ACTIVE = 4;
static constexpr uint8_t MAV_FRAME_BODY_OFFSET_NED = 9;   // velocities relative to the vehicle's heading

// type_mask bits: 0..2 ignore x,y,z | 3..5 ignore vx,vy,vz | 6..8 ignore afx,afy,afz |
// 9 force | 10 ignore yaw | 11 ignore yaw_rate.  We use velocity + yaw_rate only.
static constexpr uint16_t TYPE_MASK_VEL_YAWRATE = 0x0007 | 0x01C0 | 0x0400;   // = 0x05C7

static void put_u32(uint8_t* p, uint32_t v) { memcpy(p, &v, 4); }
static void put_u16(uint8_t* p, uint16_t v) { memcpy(p, &v, 2); }
static void put_f32(uint8_t* p, float v)    { memcpy(p, &v, 4); }

void FcLink::begin(HardwareSerial& serial, uint32_t baud) {
  ser_ = &serial;
  ser_->begin(baud);
}

// x25 / CRC-16-MCRF4XX, as used by MAVLink
void FcLink::crcAccumulate(uint8_t b, uint16_t& crc) {
  uint8_t tmp = b ^ (uint8_t)(crc & 0xff);
  tmp ^= (tmp << 4);
  crc = (crc >> 8) ^ ((uint16_t)tmp << 8) ^ ((uint16_t)tmp << 3) ^ ((uint16_t)tmp >> 4);
}

void FcLink::sendFrame(uint32_t msgid, const uint8_t* payload, uint8_t len, uint8_t crc_extra) {
  if (!ser_) return;
  // MAVLink 2 trims trailing zero bytes from the payload (keeps at least one byte)
  uint8_t plen = len;
  while (plen > 1 && payload[plen - 1] == 0) plen--;

  uint8_t hdr[10] = {
    0xFD, plen, 0 /*incompat*/, 0 /*compat*/, seq_++, MAV_SYSID, MAV_COMPID,
    (uint8_t)(msgid & 0xFF), (uint8_t)((msgid >> 8) & 0xFF), (uint8_t)((msgid >> 16) & 0xFF)
  };
  uint16_t crc = 0xFFFF;
  for (int i = 1; i < 10; i++) crcAccumulate(hdr[i], crc);
  for (int i = 0; i < plen; i++) crcAccumulate(payload[i], crc);
  crcAccumulate(crc_extra, crc);

  ser_->write(hdr, 10);
  ser_->write(payload, plen);
  uint8_t ck[2] = { (uint8_t)(crc & 0xFF), (uint8_t)(crc >> 8) };
  ser_->write(ck, 2);
  frames_++;
}

void FcLink::sendHeartbeat() {
  // field order is by size: custom_mode u32, type, autopilot, base_mode, system_status, mavlink_version
  uint8_t p[9];
  put_u32(p + 0, 0);
  p[4] = MAV_TYPE_ONBOARD_CONTROLLER;
  p[5] = MAV_AUTOPILOT_INVALID;
  p[6] = 0;
  p[7] = MAV_STATE_ACTIVE;
  p[8] = 3;
  sendFrame(MSG_HEARTBEAT, p, sizeof(p), CRC_HEARTBEAT);
}

void FcLink::sendVelocity(const DescendingCommand& c, uint32_t t_boot_ms) {
  // time_boot_ms u32, x y z vx vy vz afx afy afz yaw yaw_rate (11 floats), type_mask u16,
  // target_system u8, target_component u8, coordinate_frame u8  = 53 bytes
  uint8_t p[53];
  put_u32(p + 0, t_boot_ms);
  const float f[11] = { 0.f, 0.f, 0.f, c.vx, c.vy, c.vz, 0.f, 0.f, 0.f, 0.f, c.yaw_rate };
  for (int i = 0; i < 11; i++) put_f32(p + 4 + 4 * i, f[i]);
  put_u16(p + 48, TYPE_MASK_VEL_YAWRATE);
  p[50] = MAV_TARGET_SYS;
  p[51] = MAV_TARGET_COMP;
  p[52] = MAV_FRAME_BODY_OFFSET_NED;
  sendFrame(MSG_SET_POS_TARGET_LOCAL_NED, p, sizeof(p), CRC_SET_POS_TARGET_LOCAL_NED);
}

void FcLink::sendHover(uint32_t t_boot_ms) {
  DescendingCommand zero;
  sendVelocity(zero, t_boot_ms);
}
