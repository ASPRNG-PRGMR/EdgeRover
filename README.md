# EdgeRover 🤖

A 4-wheeled robot learning to drive itself — starting with a human on the stick, ending with the rover following a person around a room on vision alone.

> **Current Status:** Pivoting to AprilTag-based visual following. Electrical rework in progress (5A BEC replacing prior capacitor-based brownout mitigation).

<p align="center">
  <img src="images/car.jpg" alt="EdgeRover chassis" width="48%" />
  <img src="images/controller.jpg" alt="Handheld ESP-NOW controller" width="48%" />
</p>
<p align="center"><sub>The rover (left) and the handheld ESP-NOW controller it started life under (right).</sub></p>

---

## Overview

EdgeRover is a ground-up hardware + firmware project: a 4WD chassis that started life under manual ESP-NOW control and is now being converted into an autonomous follower. The rover tracks a printed **AprilTag** worn/carried by a person, steers toward it using nothing but the tag's pixel position and apparent size in frame, and stops when it gets close enough — all decision-making running locally on an ESP32-S3-CAM, no host PC, no ROS/micro-ROS, no CNN.

This is deliberately **not** a CNN/vision-model project for the core following behavior. AprilTag detection is classical computer vision — deterministic, lightweight, and it hands you exactly the two numbers the control loop needs: where the tag is in the frame, and how big it is. No training data, no inference engine required for v1.

---

## Why AprilTag, not a learned model

- A CNN-based "follow a hand/shoe" approach was considered and shelved for v1 — multiple hands/shoes in frame create ambiguity a bounding-box detector can't resolve on its own, and it needs a trained model + dataset before it does anything.
- AprilTag detection gives an unambiguous single target (unique ID, precise corners) with zero training, and the tag's apparent size in frame is enough to estimate distance via the pinhole camera model — no depth sensor, no stereo camera, no CNN.
- The CNN-based approach (hand/shoe detection via Edge Impulse FOMO + ESP-DL) remains a documented future extension — see [Roadmap](#roadmap).

---

## Why it's built this way

- **Reactive, not planned.** No map, no localization, no path planning — just "where's the tag, how big is it, react." This is the right amount of autonomy for "go where it can and follow," not full navigation.
- **Fully local.** The entire detect → steer → speed decision loop runs on the ESP32-S3-CAM. No WiFi dependency, no external agent, no host computer in the loop.
- **Speed control via PWM ceiling + distance-based ramp** means the rover decelerates smoothly as it closes in on the target instead of lurching to a stop.
- **Differential (tank) drive** — two independently driven sides, pivot turns, no steering servo — keeps the mechanical side simple so all the interesting work happens in the vision/control math.

---

## Electrical Architecture

### Current build — isolated dual-battery power

| Component | Spec |
|---|---|
| Motor battery | 12V pack |
| BEC | 5A-rated, regulated output — feeds TB6612FNG **VCC only** |
| ESP battery | 6V pack (4× AA NiMH) — separate, dedicated to the ESP32 |
| Camera | ESP32-S3-CAM / ESP32 DevKit V1 (onboard AMS1117-3.3, VIN accepts ~4.5–12V) |
| Drive motors | 4× DC gear motors (4WD) |
| Motor driver | TB6612FNG (or equivalent H-bridge rated for 4-motor draw) |

![Isolated dual-battery power wiring diagram](images/rover_power_wiring.svg)

Two fully separate battery packs, two fully separate power domains — **no wire carries both motor current and ESP logic current at any point.** The only connection between the two domains is:
1. A common ground (mandatory — without it, the PWM/direction signal lines have no shared reference and will read as garbage on the driver side).
2. The PWM/direction signal wires themselves (low current, signal only, from the ESP32 to the TB6612FNG's logic-level input pins).

This means a motor stall or start/stop current spike physically cannot reach the ESP32's supply rail anymore — it has no electrical path to get there.

**ESP32 side:** confirmed safe — 6V lands on the DevKit V1's `VIN`/`5V` pin (not `3V3` directly), which routes through the board's onboard AMS1117-3.3 linear regulator, comfortably inside its rated 4.5–12V input range.

**Motor side:** the 5A BEC's job shrunk considerably — it now only powers the TB6612FNG's `VCC` logic pin, not a camera/WiFi board, so its load is smaller and steadier than in the previous shared-rail design. `VM` (motor power, up to ~15V) still comes straight off the 12V pack.

### How this evolved

1. **4S battery pack + bulk capacitors (2200µF, then 470µF) directly on the supply rail** — motor current draw at startup/stall caused voltage sag severe enough to reset the ESP32. Capacitors alone were a band-aid: they smooth transients but don't fix a supply that can't source enough sustained current.
2. **Single regulated 5A BEC feeding both the ESP32-S3-CAM and all 4 motors off one rail** — fixed the root current-delivery problem, but the ESP32 still occasionally reset near full motor power, since it shared a regulator and rail with the motor load.
3. **Current: fully isolated dual-battery domains** (this section) — removes the shared rail entirely. Motor current and ESP logic current now have physically separate paths back to their own batteries; the domains only meet at a common ground and the low-current signal wires.

### Wiring checklist

- Common ground between the 12V pack, BEC output, TB6612FNG, the 6V pack, and the ESP32 — mandatory, and now doing double duty as the *only* electrical link between the two battery domains.
- Fuse or polyfuse between the 12V pack and the BEC input.
- Bulk capacitor (e.g., 1000µF+) across the BEC output near the TB6612FNG's VCC pin — supplementary smoothing, no longer the primary defense against brownout.
- PWM/direction signal wires from ESP32 to TB6612FNG: keep runs short; consider a small series resistor (e.g. 220Ω) on each if noise coupling shows up between the two battery systems.
- Confirm the 6V pack lands on the ESP32's `VIN`/`5V` pin, never `3V3` directly (which bypasses the onboard regulator and would overvolt the MCU).
- If the ESP32 still resets near full motor power after this change, the shared ground connection is the first thing to check — a loose/missing common ground reintroduces exactly this symptom via a floating signal reference, not a power-delivery issue this time.

---

## Software Stack

| Layer | Tool |
|---|---|
| Framework | ESP-IDF (preferred over Arduino here — more control over camera driver + easier integration of a C detection library) |
| Camera driver | `esp32-camera` (Espressif official component) |
| AprilTag detection | Port of the UMich `apriltag` C library (or lighter ArUco-style detector if frame rate demands it) |
| Motor control | ESP32 LEDC peripheral, PWM |

---

## Control Loop

```
loop (target ~15–30 fps, camera-limited):
    frame = capture_frame()
    detections = apriltag_detect(frame)

    if len(detections) > 0:
        tag = pick_best(detections)      # e.g., largest / highest confidence
        cx, cy = tag.center
        size_px = tag.apparent_size

        steer_error = compute_steer_error(cx, frame_width)
        distance_est = estimate_distance(size_px)

        if distance_est <= STOP_DISTANCE:
            motors.stop()
        else:
            speed = compute_speed(distance_est)
            turn  = compute_turn(steer_error)
            motors.drive(speed, turn)
    else:
        motors.stop()   # v1 default; frame-exit-direction logic is a planned upgrade
```

---

## Math Reference

### Steering error

```
frame_center_x = W / 2
error_x = cx - frame_center_x
error_x_norm = error_x / (W / 2)        # -1.0 (full left) to +1.0 (full right)
```

`error_x > 0` → tag right of center → steer right. `|error_x| < deadzone` → go straight (start deadzone ≈ 5% of frame width to kill jitter).

### Turn command (proportional, optionally PD)

```
turn = Kp * error_x_norm
turn = Kp * error_x_norm + Kd * (error_x_norm - prev_error_x_norm) / dt   # if oscillation appears
```

Tune `Kp` empirically, starting ~0.3–0.5.

### Distance from apparent tag size (pinhole camera model)

```
distance = (real_tag_size × focal_length_px) / apparent_tag_size_px
```

- `real_tag_size` — physical side length of the printed tag (fixed, measured once)
- `focal_length_px` — camera focal length in pixels (calibrate below)
- `apparent_tag_size_px` — measured tag side length in the current frame

**Calibration (one-time):** place tag at known distance `D_known`, measure `apparent_tag_size_px`, then:

```
focal_length_px = (apparent_tag_size_px × D_known) / real_tag_size
```

### Speed mapping (smooth deceleration on approach)

```
if distance <= STOP_DISTANCE:      speed = 0
elif distance <= SLOW_DISTANCE:    speed = map(distance, STOP_DISTANCE, SLOW_DISTANCE, MIN_SPEED, MAX_SPEED)
else:                              speed = MAX_SPEED

map(x, in_min, in_max, out_min, out_max) =
    (x - in_min) × (out_max - out_min) / (in_max - in_min) + out_min
```

### "Target has stopped" detection

```
size_history = [last N frames of apparent_tag_size_px]     # e.g., N = 10 @ 20fps
variance = statistical_variance(size_history)

if variance < STILL_THRESHOLD and distance > STOP_DISTANCE:
    motors.stop()   # target present but not moving — hold
```

### PWM duty (ESP32 LEDC) + differential drive mixing

```
duty_value = speed_fraction × (2^resolution_bits - 1)

left_speed  = clamp(speed - turn, MIN_PWM, MAX_PWM)
right_speed = clamp(speed + turn, MIN_PWM, MAX_PWM)
```

(Verify sign convention empirically against your wiring, then keep it consistent.)

---

## Constants Checklist (before first test)

- [ ] `real_tag_size` — measured, cm
- [ ] `focal_length_px` — calibrated per above
- [ ] `STOP_DISTANCE`, `SLOW_DISTANCE`
- [ ] `Kp` (and `Kd` if needed) — tune wheels-off-ground first
- [ ] `deadzone` — start ~5% of frame width
- [ ] `STILL_THRESHOLD`
- [ ] `MIN_SPEED` / `MAX_SPEED` — respect motor driver + BEC current limits

---

## Roadmap

### ✅ ESP-NOW Manual Control (Complete, superseded)
- Original handheld transmitter ↔ receiver ESP-NOW link with pot-based speed ceiling and rotary-encoder steering — proved out PWM motor control and packet-based control architecture. No longer the primary control path, but the wiring/PWM groundwork carries forward.

### 🔧 AprilTag Visual Following (In Progress)
- Electrical rework: 4S + capacitor setup → 5A BEC (see [Electrical Architecture](#electrical-architecture))
- `vision_control/apriltag_follower/` firmware written: camera capture, AprilTag (tag36h11) detection, steering/distance/still-target control math, forward-only PWM mixing, sends `ControlPacket` to the existing, unmodified `receiver/`
- Remaining: vendor the AprilTag C library into the build, calibrate `REAL_TAG_SIZE_CM`/`FOCAL_LENGTH_PX`, tune `STEER_KP`/`STOP_DISTANCE_CM`/etc. in `constants.h`
- Bench testing wheels-off-ground before first drive test

### 🔭 Future Extensions
- Frame-exit reactive behavior: rover reacts differently depending on whether the tag exits frame left, right, or top (and whether it was shrinking or growing before it vanished) — disambiguates "target walked away" vs. "target got too close."
- Hand/shoe-based following via a small trained model (Edge Impulse FOMO, deployed via ESP-DL) as an alternative to the AprilTag, once marker-following is solid.
- Second onboard camera for rear-facing detection.
- micro-ROS bridge — only if telemetry visualization on a PC or integration with a broader ROS-based system becomes a goal. Not required for the core following behavior.

---

## Drive Architecture

Differential (tank) drive — two independently driven sides, pivot turns, no steering servo. The steering math (§ [Math Reference](#math-reference)) computes `left_speed`/`right_speed` directly from the tag's position and size; the receiver-side logic doesn't need to know or care that the numbers now come from vision instead of a rotary encoder.

---

## Hardware

| Component | Details |
|---|---|
| Camera / compute | ESP32-S3-CAM (PSRAM) |
| Target marker | Printed AprilTag, fixed known size |
| Motor driver | TB6612FNG (or higher-current equivalent — confirm against 4-motor draw) |
| Drive motors | 4× DC gear motors (4WD) |
| Power | 12V battery pack + 5A BEC |
| Chassis | 4WD kit |

---

## Repository Structure

```
EdgeRover/
├── README.md
├── devlog.md
├── images/
│   ├── car.jpg
│   ├── controller.jpg
│   └── rover_power_wiring.svg
└── src/
    ├── bot_controller/
    │   ├── transmitter/                 # handheld controller (manual mode, superseded by vision)
    │   │   ├── transmitter.ino
    │   │   ├── inputs.h
    │   │   ├── inputs.cpp
    │   │   ├── display.h
    │   │   ├── display.cpp
    │   │   ├── espnow_tx.h
    │   │   ├── espnow_tx.cpp
    │   │   └── packet.h
    │   └── receiver/                    # onboard, drives the TB6612FNG (unchanged, shared by both control modes)
    │       ├── receiver.ino
    │       ├── outputs.h
    │       ├── outputs.cpp
    │       ├── espnow_rx.h
    │       ├── espnow_rx.cpp
    │       └── packet.h
    └── vision_control/
        └── apriltag_follower/            # ESP32-S3-CAM: detects tag, drives receiver/ directly
            ├── apriltag_follower.ino     # main loop: capture -> detect -> control -> send
            ├── camera.h                  # esp32-camera init + grayscale frame capture
            ├── camera.cpp
            ├── tag_detector.h            # AprilTag (tag36h11) detection wrapper
            ├── tag_detector.cpp
            ├── tracker_control.h         # steering/distance/PWM mixing math
            ├── tracker_control.cpp
            ├── constants.h               # all tunables/calibration constants, uncalibrated by default
            ├── packet.h                  # copied byte-identical from bot_controller/
            ├── espnow_tx.h                # copied unchanged from bot_controller/transmitter/
            └── espnow_tx.cpp
```

> **Heads up:** `packet.h` must be byte-identical across every folder that sends or receives a `ControlPacket` — Arduino sketches don't share headers across folders, and this struct is sent over the wire raw (`__attribute__((packed))`). If you edit one copy, copy it into all the others, or boards will silently disagree about what a byte means.
>
> `vision_control/apriltag_follower/` reuses `bot_controller/receiver/` **as-is, unmodified** — the receiver only understands `leftPWM`/`rightPWM`, so it doesn't care whether those numbers came from the handheld transmitter or the onboard camera. `apriltag_follower/` additionally requires the UMich AprilTag C library vendored in separately (not included in this repo — see `tag_detector.cpp` for integration notes).

---

## Getting Started

**Dependencies:**
- Arduino IDE (or arduino-cli) with ESP32 board support
- `esp32-camera` component (Espressif)
- UMich AprilTag C library, vendored separately — see the top comment in `tag_detector.cpp` for integration notes (not bundled in this repo)

**Steps:**
1. Verify power rail first: confirm BEC output voltage under load with motors connected, per [Electrical Architecture](#electrical-architecture), before flashing/running any vision code.
2. Flash `src/bot_controller/receiver/receiver.ino` to the onboard ESP32 (unchanged from manual-control days), note the MAC address it prints.
3. Set that MAC as `RECEIVER_MAC` in `src/vision_control/apriltag_follower/espnow_tx.cpp`.
4. Print your AprilTag (tag36h11 family) at a known, fixed size — measure it precisely.
5. Fill in `REAL_TAG_SIZE_CM` in `src/vision_control/apriltag_follower/constants.h`.
6. Calibrate `FOCAL_LENGTH_PX` (known-distance method, see [Math Reference](#math-reference)) and set it in `constants.h`.
7. Flash `src/vision_control/apriltag_follower/apriltag_follower.ino` to the ESP32-S3-CAM.
8. Bench test wheels-off-ground — watch serial output, confirm computed `leftPWM`/`rightPWM` behave sensibly as the tag moves before trusting it near the floor.
9. First drive test with `MAX_PWM_CEILING` in `constants.h` deliberately capped low, then tune `STEER_KP`/`STOP_DISTANCE_CM`/etc. from there.

---

## Devlog

Every bug, every wrong turn, every fix — from the original ESP-NOW control link through the electrical rework and into the vision pivot — is in [`devlog.md`](./devlog.md).

> **Running into a debugging, wiring/connection, or logic issue?** Check [`devlog.md`](./devlog.md) first — it's a running log of mistakes actually made on this project and how each was root-caused and fixed (power/brownout issues, pin conflicts, driver mismatches, etc.). Good chance whatever you're hitting has already been hit and solved here.

---

*Building toward autonomous edge robotics, one phase at a time. EdgeRover started on a human's stick — now it's learning to follow on its own.*
