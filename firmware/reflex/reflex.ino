// fly-drone reflex firmware for a Raspberry Pi Pico 2.
//
//   eye (camera or USB frames) -> brain (hand-written fly reflexes) -> flight controller (MAVLink)
//
// Board: "Raspberry Pi Pico 2" from the arduino-pico core (Earle Philhower).
// Build:  arduino-cli compile --fqbn rp2040:rp2040:rpipico2 firmware/reflex
// Flash:  hold BOOTSEL, plug in, copy the .uf2, or arduino-cli upload -p COMx
//
// The flight controller must be armed and put in GUIDED mode from the transmitter; the Pico
// never arms or changes modes. If the eye stops delivering frames the Pico commands hover.
#include "config.h"
#include "brain.h"
#include "reflex_brain.h"
#include "eye.h"
#include "fc_link.h"

static EyeHM01B0 eye_cam;
static EyeUsbSerial eye_usb;
static Eye* eye = nullptr;
static ReflexBrain brain;
static FcLink link;
static Retina retina;
static DescendingCommand cmd;

static bool eye_ok = false;
static uint32_t last_frame_ms = 0, prev_frame_ms = 0;
static uint32_t last_setpoint_ms = 0, last_hb_ms = 0, last_tel_ms = 0;
static uint32_t frames = 0, frames_at_last_tel = 0;

void setup() {
  pinMode(LED_PIN, OUTPUT);
  Serial.begin(115200);

  Serial1.setTX(FC_UART_TX);
  Serial1.setRX(FC_UART_RX);
  link.begin(Serial1, FC_BAUD);

  eye = (EYE_SOURCE == EyeSource::Camera) ? static_cast<Eye*>(&eye_cam) : static_cast<Eye*>(&eye_usb);
  eye_ok = eye->begin();
  brain.reset();

  // three quick blinks = booted; a long one = eye failed
  for (int i = 0; i < 3; i++) { digitalWrite(LED_PIN, HIGH); delay(80); digitalWrite(LED_PIN, LOW); delay(80); }
  if (!eye_ok) { digitalWrite(LED_PIN, HIGH); delay(1000); digitalWrite(LED_PIN, LOW); }
}

void loop() {
  const uint32_t now = millis();

  // 1. senses -> brain, whenever a new frame lands
  if (eye_ok && eye->poll(retina)) {
    float dt = prev_frame_ms ? (now - prev_frame_ms) * 0.001f : (1.0f / CAM_FPS);
    if (dt < 0.005f) dt = 0.005f;
    if (dt > 0.2f) dt = 0.2f;
    cmd = brain.step(retina, dt);
    prev_frame_ms = now;
    last_frame_ms = now;
    frames++;
  }

  // 2. brain -> flight controller at a steady rate; hover if the eye has gone quiet
  if (now - last_setpoint_ms >= SETPOINT_PERIOD_MS) {
    last_setpoint_ms = now;
    const bool eye_fresh = eye_ok && (now - last_frame_ms) <= EYE_TIMEOUT_MS && frames > 0;
    if (eye_fresh) link.sendVelocity(cmd, now);
    else link.sendHover(now);
  }
  if (now - last_hb_ms >= HEARTBEAT_PERIOD_MS) {
    last_hb_ms = now;
    link.sendHeartbeat();
  }

  // 3. LED: solid during an escape, slow blink while frames are flowing, off otherwise
  const bool eye_fresh = eye_ok && (now - last_frame_ms) <= EYE_TIMEOUT_MS && frames > 0;
  digitalWrite(LED_PIN, cmd.escape ? HIGH : (eye_fresh && ((now / 500) % 2 == 0)) ? HIGH : LOW);

  // 4. telemetry on USB for tools/bench_eye.py and for watching in a serial monitor
  if (now - last_tel_ms >= TELEMETRY_PERIOD_MS) {
    const float fps = (frames - frames_at_last_tel) * 1000.0f / (float)(now - last_tel_ms);
    last_tel_ms = now;
    frames_at_last_tel = frames;
    Serial.printf("T t=%lu eye=%s fps=%.1f yaw=%.3f vx=%.2f vy=%.2f vz=%.2f esc=%d loom=%.3f yawsig=%.3f fl=%.3f fr=%.3f mav=%lu\n",
                  (unsigned long)now, eye_ok ? eye->name() : "none", fps,
                  cmd.yaw_rate, cmd.vx, cmd.vy, cmd.vz, cmd.escape ? 1 : 0,
                  cmd.dbg_loom, cmd.dbg_yaw_sig, cmd.dbg_flow_l, cmd.dbg_flow_r,
                  (unsigned long)link.framesSent());
  }
}
