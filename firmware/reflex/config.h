// Everything you might want to change lives here.
#pragma once
#include <stdint.h>

// ---------------------------------------------------------------- retina
// The camera frame is block-averaged down to this many "ommatidia". A fly has ~800 per eye;
// 32 x 24 for both eyes together is plenty for the reflexes and cheap to process.
constexpr int RET_W = 32;
constexpr int RET_H = 24;

// ---------------------------------------------------------------- eye source
enum class EyeSource { Camera, UsbSerial };
// Camera:    Arducam HM01B0 module on the pins below.
// UsbSerial: frames pushed from the PC by tools/bench_eye.py (no camera needed).
constexpr EyeSource EYE_SOURCE = EyeSource::Camera;

// Arducam HM01B0 "camera module for Raspberry Pi Pico". GPIO numbers, not board pin numbers.
constexpr int CAM_SDA   = 4;
constexpr int CAM_SCL   = 5;
constexpr int CAM_VSYNC = 16;
constexpr int CAM_D0    = 6;      // 1-bit data bus on this module
constexpr int CAM_PCLK  = 14;
constexpr int CAM_MCLK  = 3;      // the Pico generates the camera clock on this pin
constexpr float CAM_FPS = 30.0f;  // 30 divides 60 Hz mains, so auto-exposure does not flicker
constexpr bool CAM_FLIP_H = false;
constexpr bool CAM_FLIP_V = false;

// ---------------------------------------------------------------- flight controller link
// MAVLink 2 over UART0. Wire Pico GP0 (TX) -> FC RX, GP1 (RX) <- FC TX, GND-GND.
// On the ArduPilot side: SERIALn_PROTOCOL = 2 (MAVLink2), SERIALn_BAUD = 115.
constexpr int FC_UART_TX = 0;
constexpr int FC_UART_RX = 1;
constexpr uint32_t FC_BAUD = 115200;
constexpr uint8_t MAV_SYSID = 1;            // same vehicle
constexpr uint8_t MAV_COMPID = 191;         // MAV_COMP_ID_ONBOARD_COMPUTER
constexpr uint8_t MAV_TARGET_SYS = 1;
constexpr uint8_t MAV_TARGET_COMP = 1;      // MAV_COMP_ID_AUTOPILOT1
constexpr uint32_t SETPOINT_PERIOD_MS = 100;   // 10 Hz velocity setpoints (ArduPilot needs >= ~3 Hz)
constexpr uint32_t HEARTBEAT_PERIOD_MS = 1000;
constexpr uint32_t EYE_TIMEOUT_MS = 300;    // no fresh frame for this long -> command hover

// ---------------------------------------------------------------- reflex gains
// These mirror Params in tools/reflex_sim.py. Change them there first, look at the plot,
// then copy here.
constexpr float TAU_HP       = 0.30f;   // s, contrast high-pass (photoreceptor adaptation)
constexpr float TAU_DELAY    = 0.04f;   // s, Reichardt delay arm
constexpr float CONTRAST_EPS = 1e-3f;
constexpr float K_YAW        = 4.0f;    // rad/s per unit normalised horizontal motion
constexpr float K_CENTER     = 3.0f;    // m/s per unit left-right flow imbalance
constexpr float TAU_LOOM     = 0.06f;   // s, low-pass on the looming drive
constexpr float LOOM_THRESH  = 0.08f;   // sim: looming disc peaks ~0.15, nothing else above 0.03
constexpr float ESCAPE_S     = 0.4f;
constexpr float REFRACTORY_S = 1.5f;
constexpr float MAX_YAW_RATE = 1.0f;    // rad/s
constexpr float MAX_V        = 0.5f;    // m/s
constexpr float CRUISE_VX    = 0.0f;    // m/s forward drift when nothing is happening. 0 = hover.
constexpr float YAW_FILT_S   = 0.1f;
constexpr float VY_FILT_S    = 0.2f;

// ---------------------------------------------------------------- misc
constexpr int LED_PIN = 25;             // on-board LED: heartbeat blink, solid during ESCAPE
constexpr uint32_t TELEMETRY_PERIOD_MS = 200;   // "T ..." lines on USB serial
