# fly-drone

A 3D-printed quadcopter that carries a simulated fruit-fly brain and is left to do whatever the
wiring does. The brain is a leaky integrate-and-fire (LIF) spiking network built from the
FlyWire connectome, running on a Raspberry Pi Pico 2 on the drone. A normal flight controller
keeps the drone in the air; the fly brain only decides where it goes.

```
frame/gen_frame.py      parametric frame generator (manifold3d), two presets
frame/check_2d.py       flat sections of every part, for checking hole placement
frame/stl/pico35/       3.5-inch build for the Pico 2   <- print this one
frame/stl/pi5_5in/      5-inch build sized for a Raspberry Pi 5 (whole brain onboard), for later
```

Regenerate after editing parameters:

```
uv run python frame/gen_frame.py            # pico35
uv run python frame/gen_frame.py pi5_5in
uv run python frame/check_2d.py pico35      # writes sections.png
```

## The frame (pico35)

Fully printed X quad, 180 mm wheelbase, 3.5-inch props. Every part fits a 180 mm bed.

| Part | Qty | Material | Print notes |
|---|---|---|---|
| bottom_plate.stl | 1 | PETG | flat, 4 walls, 40% infill |
| top_plate.stl | 1 | PETG | flat, 4 walls, 40% infill |
| arm_x4.stl | 4 | PETG or PETG-CF | lying flat as exported, 6 walls or 100% infill |
| leg_x4.stl | 4 | PETG or TPU | standing as exported |
| camera_bracket.stl | 1 | PETG | exported on its side, print as is |

Frame weight is about 65 g in PETG. Print two spare arms; they are the crash part.

### Ready-to-print G-code for a Creality K1 (PETG)

Files are in `frame/k1/gcode/`. Use the set that matches the nozzle on your K1.

| Nozzle | File | What | Time | Filament |
|---|---|---|---|---|
| **0.6 mm** | `flydrone_K1_0.6nozzle_plateA_plates_legs_bracket_PETG.gcode` | both plates, 4 legs, camera bracket. 0.25 mm layers, 3 walls, 40% gyroid, 4 mm brim | 2 h 42 min | 56 g |
| **0.6 mm** | `flydrone_K1_0.6nozzle_plateB_arms_PETG.gcode` | 4 arms. 0.25 mm layers, 4 walls, 100% infill, no brim | 1 h 32 min | 37 g |
| 0.4 mm | `flydrone_K1_0.4nozzle_plateA_plates_legs_bracket_PETG.gcode` | same parts, 0.2 mm layers, 4 walls | 2 h 15 min | 55 g |
| 0.4 mm | `flydrone_K1_0.4nozzle_plateB_arms_PETG.gcode` | same arms, 0.2 mm layers, 6 walls | 1 h 27 min | 37 g |

Copy them to a USB stick and print from the K1's screen, or upload through the K1's web
page (Fluidd, `http://<printer-ip>:4408`). Print plate B twice if you want the spare arms.
The G-code is for a stock K1 with the Creality Klipper `START_PRINT` / `END_PRINT` macros,
245 C nozzle, 75 C bed, auxiliary fan off. A K1C or K1 Max can run it as-is too (same
macros, bigger bed on the Max).

The 0.6 mm set uses 0.62 mm lines and 0.25 mm layers, which divides every part thickness
into whole layers (3.5 / 2.5 / 7 / 32 mm), opens the M2 and M3 holes by 0.1 mm because a
fat nozzle closes small holes, and is capped at 12 mm3/s of PETG. Raise `max_vol` in
`frame/k1/slice.py` if your hotend keeps up and you want it faster.

The layouts are also there as STLs if you would rather slice yourself:
`plate_A_plates_legs_bracket.stl`, `plate_B_arms.stl`, or `print_plate_220mm.stl` with
everything in one job. All are already centred on a 220 x 220 bed in print orientation,
no supports needed. The 5-inch preset does not fit one plate; print its parts in two jobs.

To re-slice after changing the frame, `uv run python frame/k1/slice.py` (0.6 mm) or
`uv run python frame/k1/slice.py 0.4`. It uses OrcaSlicer's
command line (portable build in `%LOCALAPPDATA%\Programs\OrcaSlicer-portable`, or on PATH),
flattens Orca's Creality K1 profiles into `frame/k1/profiles/<nozzle>/` and applies the settings below.

The settings baked into that G-code, if you slice it yourself (0.4 mm numbers; the 0.6 mm set uses 0.25 mm layers and one fewer wall):

- 0.2 mm layers, 4 walls on plates, 6 walls or 100% infill on arms.
- 240 to 250 C nozzle, 75 to 80 C bed. Slow it to about 150 mm/s on outer walls; the
  stock 300 mm/s PETG profile is fine for the plates but the arms like it slower and hotter.
- Part fan 30 to 50%. Turn the auxiliary side fan OFF for PETG; it wrecks layer bonding.
  Chamber fan off. Keep the lid on and the door shut.
- PETG bonds too hard to the K1's smooth PEI sheet and can tear the coating. Wipe a thin
  film of glue stick on first as a release layer, or use a textured PEI plate.
- PETG-CF only on a K1C or a K1 with a hardened-steel nozzle; the stock brass nozzle wears
  out. Plain PETG with 6 walls is strong enough for the arms.
- TPU legs are optional. If you try them, 30 to 40 mm/s, retraction 0.5 mm, 220 C.

How it goes together:

- Each arm has a thin tab at the inner end. The tab sits between the bottom and top plates on
  the diagonal "ear" and is held by two M3 x 20 bolts. The arm steps up to full height right
  where the ear ends, so the step locates it against the plate edge.
- The same two bolts continue through the bottom plate into a nut trap in a leg under each
  corner. Slide an M3 nut into the side slot of each leg first.
- The flight controller stack goes on the top plate. Both the 20 x 20 mm (M2) and
  30.5 x 30.5 mm (M3) patterns are there.
- The Pico 2 mounts across the body on four 20 mm M2 standoffs, above the stack.
- The camera bracket bolts to the two M3 holes at the nose of the top plate and hooks over the
  plate edge. The upright has two vertical slots 21 mm apart (fits the Pi Camera Module 3
  pattern) plus zip-tie slots for anything else, tilted 15 degrees up.
- The optical-flow sensor mounts on the tail of the bottom plate, looking down through the
  10 mm hole. There is a generic 15 x 15 mm M2 pattern around it; zip-tie if yours differs.
- Battery goes under the bottom plate, strap through the two slots, between the legs.

Motor pads carry both a 16 x 16 M3 and a 12 x 12 M2 pattern (rotated 45 degrees from each
other), so any 1404 / 1504 / 2004 class motor bolts on.

Clearances as generated: prop disc to camera 13.5 mm, prop disc to Pico 21 mm,
prop tip to prop tip 38 mm.

## Parts to buy (pico35)

| Item | Spec | Notes |
|---|---|---|
| Motors x4 | 1404 ~3800 KV or 1504 ~3000 KV, 4S | 12 x 12 M2 mount is most common in this size |
| Props | 3.5-inch tri-blade, 4 + spares | e.g. Gemfan 3520 or HQProp 3.5 x 2 x 3 |
| Flight controller | 20 x 20 or 30.5 x 30.5, **ArduPilot-supported**, one spare UART | e.g. Kakute H7 Mini (20 x 20) or Matek H743-SLIM (30.5) |
| ESC | 4-in-1, 20 to 35 A, 4S, same mount as the FC | comes as a stack with many FCs |
| Optical flow + rangefinder | Matek 3901-L0X or ARK Flow | indoor position hold, no GPS |
| Battery | 4S 650 to 850 mAh LiPo, XT30 | plus a LiPo charger and a charging bag |
| Radio | ELRS receiver + an ELRS transmitter (RadioMaster Pocket is the cheap one) | needed to arm, to test-fly, and as the kill switch. Not optional |
| Brain | Raspberry Pi Pico 2 (you have it) | powered from the FC's 5 V rail |
| Eye | Arducam HM01B0 mono camera for Pico, or an OV7670; a VL53L1X ToF sensor is the simplest "something is approaching" input | fly eyes are low-res, 320 x 240 is plenty |
| Hardware | 8x M3 x 20 bolts, 8x M3 nuts, 4x M2 x 20 standoffs + screws, FC stack screws/gummies, 200 mm battery strap, zip ties, XT30 pigtail, 20 AWG wire, 35 V 470 uF cap for the ESC | |

Expected all-up weight is around 250 g. 1404 motors on 4S give roughly 1.5 kg of thrust
for that, so it is not marginal. Flight time will be 5 to 7 minutes per battery; the brain
keeps running between flights, flying is just when the battery is in.

## Firmware: the hand-written reflex brain (Pico 2)

`firmware/reflex/` is an Arduino sketch for the Pico 2 (arduino-pico core). It is the
"acts like a fly" version: no connectome, just the two or three reflexes a fly is known for,
written by hand. It exists to debug the eye, the flight-controller link and the flying before
the real fly circuit goes in, and to be the thing the connectome version is compared against.

```
eye (HM01B0 camera, or frames from the PC over USB)
  -> 32 x 24 retina
  -> ReflexBrain        photoreceptor high-pass, Reichardt motion detectors (T4/T5),
                        whole-field motion -> optomotor yaw (HS cells),
                        left/right flow balance -> sidestep (corridor centring),
                        LPLC2-style looming units -> giant fiber -> ESCAPE
  -> DescendingCommand  body-frame velocity + yaw rate
  -> FcLink             MAVLink 2 SET_POSITION_TARGET_LOCAL_NED to ArduPilot, 10 Hz
```

Files:

| File | What |
|---|---|
| `reflex.ino` | main loop: eye -> brain -> flight controller, LED, telemetry on USB |
| `config.h` | pins, camera mode, MAVLink ids, every reflex gain and threshold |
| `brain.h` | `Retina`, `DescendingCommand`, and the `Brain` interface the connectome version will also implement |
| `reflex_brain.h/.cpp` | the reflexes; pure C++, no Arduino dependency |
| `eye.h/.cpp` | `EyeHM01B0` (camera via the vendored PicoHM01B0 PIO/DMA driver) and `EyeUsbSerial` (frames from the PC) |
| `fc_link.h/.cpp` | hand-rolled MAVLink 2 framing for HEARTBEAT and SET_POSITION_TARGET_LOCAL_NED |
| `src/PicoHM01B0/` | camera driver by pmarques-dev, BSD-2, vendored unchanged |

Wiring: camera on GP4 (SDA), GP5 (SCL), GP16 (VSYNC), GP6 (D0), GP14 (PCLK), GP3 (MCLK),
3V3 and GND. Flight controller UART: Pico GP0 (TX) to FC RX, GP1 (RX) to FC TX, GND to GND.
Pico powered from the FC's 5 V pad into VSYS. On the ArduPilot side set that serial port to
`SERIALn_PROTOCOL = 2` and `SERIALn_BAUD = 115`, and fly in GUIDED. The Pico never arms and
never changes mode; your transmitter does that, and its kill switch always wins. If frames stop
arriving for 300 ms the Pico commands hover.

Build and flash (arduino-cli, portable copy in `%LOCALAPPDATA%\Programs\arduino-cli`, or the
Arduino IDE with the "Raspberry Pi Pico/RP2040/RP2350" core installed):

```
arduino-cli compile --fqbn rp2040:rp2040:rpipico2 --output-dir build/pico2 firmware/reflex
arduino-cli upload  --fqbn rp2040:rp2040:rpipico2 -p COM5 firmware/reflex
```

or hold BOOTSEL while plugging the Pico in and copy the `.uf2` onto the drive that appears.
`firmware/reflex/prebuilt/reflex_pico2_camera.uf2` is a ready-built image of the camera
configuration, so you can try it without installing any toolchain.

Testing without hardware, in this order:

1. `uv run python tools/reflex_sim.py` runs the same math in numpy on synthetic stimuli
   (gratings, a looming disc, a passing object, corridors, noise) and writes
   `tools/reflex_sim.png` with PASS/FAIL per case. Tune constants here, then copy to `config.h`.
2. `uv run python tools/host_check.py` compiles the firmware's `reflex_brain.cpp` for the PC
   (with zig, a dev dependency) and checks it gives the same outputs as the Python on identical
   8-bit frames.
3. Set `EYE_SOURCE = EyeSource::UsbSerial` in `config.h`, flash, then
   `uv run python tools/bench_eye.py --port COM5 --stim cycle` streams the synthetic stimuli
   to the Pico over USB and prints the telemetry it sends back. Watch `yaw` flip sign with the
   grating direction and `esc=1` on the looming disc.
4. Camera in, `EYE_SOURCE = EyeSource::Camera`, flash, open a serial monitor at 115200, wave a
   hand at it. Then wire the flight controller.

What it does in the air: hovers, turns to follow motion across its view, drifts away from
the side with more motion, and jumps up and back when something comes straight at it. Set
`CRUISE_VX` above zero and it wanders forward. Everything else is you tuning gains.

## The brain on a Pico 2

The Pico 2 is a microcontroller (two Cortex-M33 at 150 MHz, 520 KB RAM, 4 MB flash), so it
cannot hold the whole 139k-neuron brain. It can hold a real subcircuit of it:

- Flash budget: ~4 MB of connections at 6 bytes each (uint32 target + int16 weight) is about
  600k synapses. RAM budget: ~40k neurons of state. A 5k to 15k neuron visual-to-descending
  subcircuit at 1 ms steps, event-driven, fits with room to spare.
- Core 1 runs the LIF network. Core 0 reads the camera, turns it into ~200 "ommatidia" per
  eye, drives the input neurons, reads the descending neurons, and talks to the flight
  controller.
- The subcircuit is extracted on the PC from the FlyWire v783 release the way
  [mhdsilva/flywire-arduino](https://github.com/mhdsilva/flywire-arduino) does it (looming
  detectors LC4/LPLC2 through interneurons to the giant fiber DNp01), extended with the
  motion pathway (T4/T5 to the lobula plate tangential cells to turning descending neurons).
  [Ranuja01/fly-simulator](https://github.com/Ranuja01/fly-simulator) has a 15k-neuron
  looming-escape cut that is a good size reference.
- Output: giant fiber rate -> escape burst (up and back), left/right descending neuron rates
  -> yaw and sideways velocity, forward descending neurons -> forward velocity. Sent as
  MAVLink velocity setpoints in GUIDED mode (the MAVLink C library runs fine on an MCU).
- The flight controller (ArduPilot, FlowHold or Loiter indoors) does all stabilisation.
  The brain gives decisions; the stabiliser gives reflexes.

If you later want the whole brain onboard, the `pi5_5in` preset carries a Raspberry Pi 5.
[flykeeper](https://github.com/fortunto2/flykeeper) runs the full FlyWire connectome in
real time in Rust on a phone-class ARM chip, so a Pi 5 is realistic.

## Build order

1. Print the frame and buy the parts above.
2. Build it as a normal ArduPilot quad first: bind the radio, calibrate, hover in Stabilize,
   then FlowHold. No brain yet. A drone that hovers on its own is the prerequisite.
3. Wire the Pico 2 to a spare FC UART and prove it can push velocity setpoints in GUIDED mode
   from a hard-coded script, indoors, in a net.
4. Bench the brain on the Pico: run the subcircuit, wave a hand at the camera, watch the giant
   fiber fire on the serial monitor.
5. Plug the brain output into step 3. Fly in a net with a finger on the kill switch.

Expect: nothing without input; a flinch when something approaches; turning toward motion if
the motion pathway is in the cut. Anything beyond that is your decoder, not the fly.
