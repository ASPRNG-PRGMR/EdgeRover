# EdgeRover 🤖

A 4-wheeled robot learning to drive itself — starting with a human on the stick, ending with the rover following a person around a room on vision alone.

> **Current Status:** Manual control (ESP-NOW, tank-turn + in-place pivot) is code-complete and compiles clean — bench testing pending, blocked on hardware access. Electrical rework is done: motor-load resets traced to EN-pin noise coupling (not brownout) and fixed via BEC-unified logic rail + EN-pin cap + motor snubbers + star ground. AprilTag follower firmware (ESP32-S3-CAM + OV3660 + SG90 camera tilt, Pythagoras ground-distance model, 50 Hz sender task) is written and compiles clean — untested on hardware; calibration + bench test are next. Geometry check says a flat pocket tag is not decodable from 15 cm at chassis height, so the tag needs a ~40° downward wedge or the camera a ~50 cm mast.

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

### Current build — BEC-fed logic, 12V direct to motor power, single star ground

| Component | Spec |
|---|---|
| Motor battery | 12V pack — feeds BEC input **and** TB6612FNG `VM` directly |
| BEC | 5A-rated, regulated 5V output — feeds **ESP32 VIN and TB6612FNG `VCC` (logic) only**, never motor power |
| Camera | ESP32-S3-CAM / ESP32 DevKit V1 (onboard AMS1117-3.3, VIN accepts ~4.5–12V) |
| Drive motors | 4× DC gear motors (4WD) |
| Motor driver | TB6612FNG (or equivalent H-bridge rated for 4-motor draw) |
| Noise mitigation | 0.1–1µF cap on ESP32 `EN`→GND, 0.1µF snubber cap across each motor's terminals, 100–470µF bulk cap on ESP32 `VIN`→GND |

![Power wiring diagram](images/rover_power_wiring.svg)
> ⚠️ Diagram is stale as of this revision — it still shows the earlier isolated dual-battery layout, not the BEC-unified-logic topology described below. Needs re-drawing.

One battery (12V), one regulator (5A BEC), but the BEC's output only ever touches **logic-level** current — ESP32 + TB6612FNG `VCC`. Actual motor drive current (`VM`) comes straight off the 12V pack and never passes through the BEC at all. This keeps the ESP32 electrically distanced from the high-current motor path while avoiding the ground-topology complexity of running two entirely separate batteries.

**All grounds — 12V pack negative, BEC ground, TB6612FNG `GND`, ESP32 `GND` — land on a single, soldered, star ground point**, not daisy-chained through each other. This matters more than it sounds like it should: motor return current physically flows through whatever wire segment it's routed through, and if that segment is shared with a "quiet" node like the ESP32's ground reference, the IR drop across that shared wire shows up as noise/ground-bounce at the ESP32, indistinguishable from an actual supply dip unless you go looking for the wiring topology specifically.

### How this evolved

1. **4S battery pack + bulk capacitors (2200µF, then 470µF) directly on the supply rail** — motor current draw at startup/stall caused voltage sag severe enough to reset the ESP32. Capacitors alone were a band-aid: they smooth transients but don't fix a supply that can't source enough sustained current.
2. **Single regulated 5A BEC feeding both the ESP32-S3-CAM and all 4 motors off one rail** — fixed the root current-delivery problem, but the ESP32 still occasionally reset near full motor power, since it shared a regulator and rail with the motor load.
3. **Fully isolated dual-battery domains** (6V NiMH dedicated to ESP32, 12V + BEC for motors, joined only at a common ground) — removed the shared rail entirely. Still reset near full motor power, at a repeatable PWM threshold (~27/255). Root-caused via a controlled test: powering the ESP32 from a PC's USB port (fully independent of *both* battery packs) still reproduced the reset at the same threshold, with reset reason `POWERON_RESET` rather than a brownout code — proving this was never a supply-sag problem at all. The real mechanism was motor switching/back-EMF noise coupling into the ESP32's `EN` (reset) pin via the shared ground path.
4. **Current: BEC-unified logic rail (this section)**, plus targeted noise fixes — an `EN`-to-GND capacitor (raised the reset-free PWM threshold immediately, from ~27/255 to ~99/255 on its own), snubber capacitors across each motor's terminals (kills back-EMF at the source instead of filtering it downstream), and rebuilding the ground wiring as a genuine single-point star instead of anything daisy-chained. Combined, these resolved the resets completely — full 0–255 PWM range confirmed stable.

### Wiring checklist

- **Star ground, soldered, not daisy-chained:** 12V pack negative, BEC ground, TB6612FNG `GND`, and ESP32 `GND` all land on one common point. No ground wire should ever be routed *through* another component on its way to this point.
- 0.1–1µF ceramic capacitor between ESP32 `EN` and `GND`, as close to the module as possible. Safe to leave in permanently — most DevKit boards already ship with a smaller cap here for the auto-upload reset circuit; this just adds more filtering. Only watch for: values much above ~1µF can occasionally slow the auto-reset timing used by `esptool`/Arduino IDE uploads — if uploads ever get flaky, hold `BOOT` manually during flash, or reduce the cap value.
- 0.1µF ceramic snubber capacitor across each DC motor's terminals, soldered as close to the motor body as possible — kills brush/commutation noise at the source.
- 100–470µF bulk capacitor across ESP32 `VIN`–`GND`, close to the board — local reservoir for WiFi TX current spikes, cheap insurance.
- Fuse or polyfuse between the 12V pack and the BEC input.
- Bulk capacitor (1000µF+) across the BEC output — supplementary smoothing on the logic rail.
- PWM/direction signal wires from ESP32 to TB6612FNG: keep runs short and physically separated from motor power leads (cross at 90° if they must cross; don't bundle them together) — reduces the chance of noise coupling directly onto signal lines independent of the ground path.
- Confirm the BEC output lands on the ESP32's `VIN`/`5V` pin, never `3V3` directly (which bypasses the onboard regulator and would overvolt the MCU).
- If resets ever reappear near full motor power: check the star ground first (a loosened solder joint or a jumper that's crept back into a daisy-chain arrangement reproduces this exact symptom), then confirm the EN cap and motor snubbers are still intact — in that order, since that's the order these were actually diagnosed and fixed in.

---

## Software Stack

| Layer | Tool |
|---|---|
| Framework | Arduino-ESP32 core 3.x (IDF 5 underneath); the vision board and both control boards build from the Arduino IDE / arduino-cli |
| Camera driver | `esp32-camera` (bundled with the core) — OV3660, grayscale VGA straight from the sensor |
| AprilTag detection | [raspiduino/apriltag-esp32](https://github.com/raspiduino/apriltag-esp32) — UMich `apriltag` 3 as an Arduino library (float math, tag36h11 trimmed to 35 IDs) |
| Camera tilt | SG90 servo via the IDF LEDC driver (timer 1, so it cannot collide with the camera's XCLK on timer 0) |
| Motor control | ESP32 LEDC peripheral, PWM, on the receiver board (`bot_controller/receiver`) |
| Link | ESP-NOW, `ControlPacket` v3, 50 Hz from a dedicated task |

---

## Control Loop

Two loops on the ESP32-S3-CAM, one per core:

```
vision loop (core 1, camera/detector-limited, ~8-15 fps at VGA):
    frame = capture_grayscale()
    tag   = apriltag_detect(frame)            # filtered by ID / hamming / decision margin, largest wins

    if tag:
        range = range_model(tag, servo_tilt)  # slant → rotate by tilt → Pythagoras → ground, bearing
        cmd   = tracker(range)                # hysteresis stop/resume, slow zone, back-off, steer, pivot
        servo_target += TILT_KP × tag.in_frame_elevation   # keep the tag vertically centred
    else:
        cmd = lost_behaviour()                # hold last cmd → pivot toward last side → stop + search tilt

    publish(cmd, armed = seen_a_tag_ever)
    servo_step()                              # rate-limited move toward servo_target

sender task (core 0, 50 Hz, independent of frame rate):
    cmd = latest published command (zeros if older than 400 ms)
    cmd = slew_limit(cmd)                     # soft ramps, fast decel
    espnow_send(ControlPacket{cmd, armed})    # keeps the receiver's 200 ms failsafe fed
```

The receiver-side loop is unchanged from manual control: it drives whatever `leftPWM`/`rightPWM` it last received and fails safe 200 ms after the last packet.

---

## Math Reference

Implemented in `range_model.cpp` (geometry) and `tracker_control.cpp` (control). Frame convention: image x right, y **down**; rover frame x right, y **up**, z forward, origin at the lens.

### Slant range from apparent tag size (pinhole camera model)

```
slant_r = REAL_TAG_SIZE_CM × FOCAL_LENGTH_PX / size_px
```

`size_px` is the larger of (mean horizontal edge, mean vertical edge) of the detected quad — foreshortening only ever shrinks an edge, so this stays close to the true side length when the camera looks steeply up at the tag or the person half-turns. `FOCAL_LENGTH_PX` scales with frame width and is calibrated with `model/fit_camera_model.py`.

### In-frame angles

```
dx = cx − W/2            dy = cy − H/2
in_frame_bearing   = atan2( dx, f)          # + = tag right of centre
in_frame_elevation = atan2(−dy, f)          # + = tag above centre  → drives the tilt servo
```

### Camera tilt → rover frame, then Pythagoras (the servo's job)

The pinhole gives distance *along the line of sight*. Near the person the rover is looking steeply up at a pocket-height tag, so the slant range is dominated by height, not horizontal distance. With the camera pitched up by the servo tilt τ:

```
v_cam   = slant_r × unit(dx, −dy, f)                # vector to the tag in the camera frame (y up)
x_r     = x_cam
y_r     = y_cam·cos τ + z_cam·sin τ                  # vertical leg   (height above the lens)
z_r     = −y_cam·sin τ + z_cam·cos τ

height  = y_r
ground  = sqrt(slant_r² − height²) = sqrt(x_r² + z_r²)   # horizontal leg — what the stop logic uses
bearing = atan2(x_r, z_r)                                # heading error, + = right
elevation = atan2(height, ground)
```

Sanity value: `height` should equal `TAG_HEIGHT_CM − CAMERA_HEIGHT_CM` at every distance. Worked example at the heels: rise 73 cm, ground 15 cm → slant 74.5 cm, elevation 78°; the pinhole alone would say "75 cm away", Pythagoras says 15.

Optional (`USE_POSE_ESTIMATE 1`): `v_cam` comes from the library's homography pose solver instead of the size model; everything after step one is identical.

### Tilt servo

```
tilt_target = tilt_now + TILT_KP × in_frame_elevation      (deadzone TILT_DEADZONE_DEG)
tilt_now   += clamp(tilt_target − tilt_now, ±TILT_RATE_DEG_PER_S × dt)
servo_deg   = SERVO_LEVEL_DEG + SERVO_UP_SIGN × tilt_now    → 500-2500 µs pulse at 50 Hz
```

### Turn command (P, optionally PD, on rover-frame bearing)

```
turn = STEER_KP × bearing_deg  [+ STEER_KD × d(bearing)/dt]      clamped to ±1, 0 inside STEER_DEADZONE_DEG
```

Default `STEER_KP 0.03`/deg: inner wheel stops at ~17° bearing, full lock at ~33° (the VGA field of view is ±28°).

### Speed from ground distance (hysteresis, slow zone, back-off)

```
if ground <= MIN_FOLLOW_DISTANCE_CM:  holding = true        # 15 cm
if ground >= RESUME_DISTANCE_CM:      holding = false       # 25 cm — no chatter at the boundary

if ground <  BACKOFF_DISTANCE_CM:     both wheels = −BACKOFF_PWM       # person stepped into us
elif holding:                         speed = 0  (pivot allowed if |turn| > deadzone)
elif ground <  SLOW_DISTANCE_CM:      speed = map(ground, MIN_FOLLOW, SLOW, MIN_MOVING_PWM/ceiling, 1) × ceiling
else:                                 speed = ceiling

ceiling = MAX_PWM_CEILING, or UNCALIBRATED_PWM_CAP while FOCAL_IS_CALIBRATED == 0
```

The earlier "target has stopped" variance heuristic was removed: with a real distance estimate, a person standing still simply means "drive up to 15 cm and hold".

### Differential drive mixing (identical to `transmitter/inputs.cpp`)

```
speed == 0 and |turn| > deadzone  →  pivot: left = +p, right = −p (turn right), p = PIVOT_PWM_MIN..MAX × |turn|

otherwise:  outer wheel = speed
            inner wheel = speed × (1 − 2·|turn|)     # +1 centred → 0 at half lock → −1 (reverse) at full lock
            turn > 0 → right wheel is inner
```

### Lost target

```
< LOST_HOLD_MS  (300 ms):   repeat last command
< +LOST_PIVOT_MS (1.5 s):   pivot at LOST_PIVOT_PWM toward the side the tag was last seen on
after:                      stop, camera to search tilt = atan2(TAG_HEIGHT − CAMERA_HEIGHT, TILT_SEARCH_RANGE_CM)
```

---

## Constants Checklist (before first test)

All in `src/vision_control/apriltag_follower/constants.h`; full procedure in [`src/vision_control/README.md`](./src/vision_control/README.md).

- [ ] `REAL_TAG_SIZE_CM` — outer black border, measured
- [ ] `TARGET_TAG_ID` — the printed ID (0–34)
- [ ] `CAMERA_HEIGHT_CM` / `TAG_HEIGHT_CM` — and `model/follow_geometry.py` run with them (wedge or mast decided)
- [ ] `SERVO_LEVEL_DEG` / `SERVO_UP_SIGN` — camera level at that angle, larger angle tilts up
- [ ] `CAMERA_VFLIP` / `CAMERA_HMIRROR` — `bear` positive when the tag moves right, servo tilts up when it rises
- [ ] `FOCAL_LENGTH_PX` + `FOCAL_IS_CALIBRATED 1` — from `model/fit_camera_model.py`
- [ ] `MIN_FOLLOW_DISTANCE_CM` / `RESUME_DISTANCE_CM` / `SLOW_DISTANCE_CM`
- [ ] `STEER_KP` (and `STEER_KD` if needed), `TILT_KP` — tune wheels-off-ground first
- [ ] `MAX_PWM_CEILING` — start low; respect motor driver + BEC current limits
- [ ] `RECEIVER_MAC` in `espnow_tx.cpp`

---

## Roadmap

### ✅ ESP-NOW Manual Control (Complete, superseded)
- Original handheld transmitter ↔ receiver ESP-NOW link with pot-based speed ceiling and rotary-encoder steering — proved out PWM motor control and packet-based control architecture. No longer the primary control path, but the wiring/PWM groundwork carries forward.

### 🏁 Competition Prep — Tank-Turn Steering (Current Priority)
Robo race entry: square track, rounded corners — one side has an oil/slip section, one side has a ramp, one side is clean, and the last side ends in a standing-block obstacle course requiring tight maneuvering right as the rover exits the corner. Three laps, best lap counts.

**Current focus: get the existing skid-steer (tank-turn) mixing as precise and reliable as possible.** This is the control scheme that will actually run at competition unless confirmed otherwise.

- **Superseded the old floor-based turning (`MIN_INNER_FACTOR`) with a through-zero design.** The previous approach tapered the inner wheel down to a fixed 35% floor and never lower — it prevented a dead stop at full lock, but also made a true in-place pivot unreachable while driving. The inner wheel now follows `1.0 - 2×turnFactor`: full forward at center, exactly `0` at half steering lock, ramping into genuine reverse toward full lock — smoothly covering everything from a gentle curve to a zero-radius spin within one continuous formula, no separate mode required.
- **In-place pivot, no new hardware:** when the throttle pot is idle and the steering encoder is deflected past a small deadband, both wheels drive at equal magnitude in opposite directions, scaled by how far the encoder is turned. Reuses the existing pot + encoder — moving throttle out of idle hands control back to the normal turning behavior above.
- **`ControlPacket` bumped to v3:** `leftPWM`/`rightPWM` changed from `uint8_t` (0–255, forward-only) to `int16_t` (-255..255, sign = direction). This is what makes the reverse-capable inner wheel and pivot possible — the old unsigned fields had no way to represent "spin this wheel backward." `outputs.cpp` now derives the TB6612FNG direction pins from the sign of each value instead of hardcoding forward.
- **Status: code-complete, compiles clean, not yet bench-tested** — testing is blocked on hardware access (ESP32 not currently on hand). First test must be wheels-off-ground: confirm pivot spins the expected direction at a low `PIVOT_PWM_MAX` (start ~80–100, default guess is `180`) before trusting it at speed, since the new turn curve is meaningfully sharper past half steering lock than the old floored version and will feel different even in normal (non-pivot) turns.
- `PIVOT_PWM_MAX`, `THROTTLE_DEADBAND`, and `STEER_DEADBAND` are starting guesses, not calibrated — need a proper bench pass followed by track-condition testing, especially through the obstacle section where turn precision matters most.
- Reverse driving (independent of pivot) was considered and deliberately dropped — doesn't add value for this track layout, and keeps the arm/disarm switch as the only safety-critical button rather than overloading it or the (fully committed) button pins with a second role.
- Considered switching the front axle to a servo-actuated steering knuckle (Ackermann/car-style) for better traction through the oil section, since skid-steering relies on friction to turn and that breaks down on a slippery surface. **Parked for now** — car-style steering has a real minimum turn radius and can't pivot in place, which would hurt badly in the obstacle course (the section that likely matters most for lap time). Possible future direction if competition rules allow a **dual-mode** setup (servo-steered for the oil section, tank-turn for everything else, toggled via the spare pot/encoder/switch already on hand) — but that depends on confirming it's allowed, and isn't the priority until tank-turn itself is bench- and track-validated.

### 🔧 AprilTag Visual Following (In Progress)
- Electrical rework: 4S + capacitor setup → 5A BEC (see [Electrical Architecture](#electrical-architecture))
- `vision_control/apriltag_follower/` firmware written and compiling clean against ESP32 core 3.3.11 + raspiduino/apriltag-esp32: camera capture, AprilTag (tag36h11) detection, SG90 camera-tilt servo, slant-range + tilt → Pythagoras ground-distance model, distance hysteresis with 15 cm stop / gentle back-off, tank-turn + pivot mixing (signed v3 packets), lost-target hold/pivot/search, dedicated 50 Hz sender task so slow detections never trip the receiver's 200 ms failsafe
- `vision_control/model/`: geometry check (`follow_geometry.py`) and focal-length fit (`fit_camera_model.py`) — no ML needed for this stage
- Remaining: install the AprilTag library, print the tag, calibrate `FOCAL_LENGTH_PX`, confirm orientation flags and `SERVO_LEVEL_DEG` on the bench, then tune gains. Physical constraint found by the geometry model: a flat tag on a back pocket is not decodable from 15 cm at chassis height — pitch the tag down ~40° or raise the camera
- Bench testing wheels-off-ground before first drive test

### 🔭 Future Extensions
- Frame-exit reactive behavior: the follower already pivots toward the side the tag was last seen on; still to do is using the tag's size trend before it vanished to disambiguate "target walked away" vs. "target got too close."
- Hand/shoe-based following via a small trained model (Edge Impulse FOMO, deployed via ESP-DL) as an alternative to the AprilTag, once marker-following is solid.
- Second onboard camera for rear-facing detection.
- micro-ROS bridge — only if telemetry visualization on a PC or integration with a broader ROS-based system becomes a goal. Not required for the core following behavior.

---

## Drive Architecture

Differential (tank) drive — two independently driven sides, pivot turns, no steering servo (the only servo on the rover tilts the camera). The steering math (§ [Math Reference](#math-reference)) computes `left_speed`/`right_speed` directly from the tag's position and size; the receiver-side logic doesn't need to know or care that the numbers now come from vision instead of a rotary encoder.

> **Conditional idea, not built:** a servo-actuated front axle for car-style (Ackermann) steering, toggled against tank drive via a mode switch, was considered for the competition's oil/slip section specifically. Not pursued unless confirmed allowed by competition rules — see [Competition Prep](#-competition-prep--tank-turn-steering-current-priority) in the Roadmap.

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
        ├── README.md                     # setup, wiring, calibration, bench tests, tuning, troubleshooting
        ├── apriltag_follower/            # ESP32-S3-CAM (N16R8 + OV3660): detects tag, tilts camera, drives receiver/ directly
        │   ├── apriltag_follower.ino     # main loop: capture -> detect -> range model -> tracker -> publish -> tilt servo
        │   ├── camera.h / camera.cpp     # esp32-camera init (ESP32S3_EYE pin map, OV3660 flip fix) + grayscale capture
        │   ├── tag_detector.h / .cpp     # AprilTag (tag36h11) wrapper: filters, foreshortening-aware size, optional pose
        │   ├── tilt_servo.h / .cpp       # SG90 camera tilt on GPIO21 (IDF LEDC timer 1, away from the camera's timer 0)
        │   ├── range_model.h / .cpp      # slant range + servo tilt -> Pythagoras -> ground distance & bearing
        │   ├── tracker_control.h / .cpp  # distance hysteresis, steering P(D), tank-turn/pivot mixing, lost-target logic
        │   ├── command_sender.h / .cpp   # 50 Hz ESP-NOW task (core 0): slew limit + stale cut-off, keeps receiver out of failsafe
        │   ├── constants.h               # all tunables/calibration constants
        │   ├── packet.h                  # copied byte-identical from bot_controller/
        │   ├── espnow_tx.h / .cpp        # copied unchanged from bot_controller/transmitter/
        └── model/                        # measurement model tooling (no ML): geometry check + focal-length fit
            ├── README.md
            ├── follow_geometry.py        # per-distance viewing angle / pixel size table; shows where detection breaks
            └── fit_camera_model.py       # least-squares FOCAL_LENGTH_PX (+ servo level offset) from tape-measure data
```

> **Heads up:** `packet.h` must be byte-identical across every folder that sends or receives a `ControlPacket` — Arduino sketches don't share headers across folders, and this struct is sent over the wire raw (`__attribute__((packed))`). If you edit one copy, copy it into all the others, or boards will silently disagree about what a byte means.
>
> **`PACKET_VERSION` is now `3`:** `leftPWM`/`rightPWM` changed from `uint8_t` to `int16_t` (sign = direction, magnitude = duty) to support tank-turn's reverse-capable inner wheel and in-place pivot. Any copy of `packet.h` still on v2 (unsigned fields) is incompatible — mismatched transmitter/receiver builds won't just get rejected by the version check, they'll misinterpret the struct's byte layout entirely since the field widths changed.
>
> `vision_control/apriltag_follower/` reuses `bot_controller/receiver/` **as-is, unmodified** — the receiver only understands `leftPWM`/`rightPWM`, so it doesn't care whether those numbers came from the handheld transmitter or the onboard camera. The vision path emits the same signed v3 values (through-zero tank turn while moving, in-place pivot when stopped, gentle reverse back-off) using the same mixing formula as `transmitter/inputs.cpp`. `apriltag_follower/` additionally requires the [raspiduino/apriltag-esp32](https://github.com/raspiduino/apriltag-esp32) Arduino library (UMich AprilTag 3 with float math and a trimmed tag36h11 table — print an ID in 0..34) — see `tag_detector.cpp`.

---

## Getting Started

**Rover / manual link** (unchanged):
1. Verify the power rail first: confirm BEC output voltage under load with motors connected, per [Electrical Architecture](#electrical-architecture).
2. Flash `src/bot_controller/receiver/receiver.ino` to the onboard ESP32; note the MAC address it prints.
3. Flash `src/bot_controller/transmitter/transmitter.ino` with that MAC in `espnow_tx.cpp` for manual driving.

**Vision follower:** the complete walkthrough — servo wiring, board settings, library install, tag printing, orientation checks, focal-length calibration, bench test sequence, tuning table and troubleshooting — lives in [`src/vision_control/README.md`](./src/vision_control/README.md). Short version:

1. Run `python3 src/vision_control/model/follow_geometry.py` with your camera and tag heights; pitch the tag ~40° down or mast the camera before building (a flat pocket tag is not decodable from 15 cm at chassis height).
2. Wire the SG90 to GPIO21 / 5 V BEC / star ground. Install [raspiduino/apriltag-esp32](https://github.com/raspiduino/apriltag-esp32); board = ESP32S3 Dev Module, PSRAM = OPI, Flash = 16 MB, Partition = Huge APP.
3. Print a tag36h11 tag (ID 0–34) on stiff card; set `REAL_TAG_SIZE_CM`, `TARGET_TAG_ID`, heights and servo mounting constants; put the receiver's MAC in `espnow_tx.cpp`.
4. Flash `src/vision_control/apriltag_follower/apriltag_follower.ino`, run the bench tests wheels-off-ground, calibrate `FOCAL_LENGTH_PX`, then drive with `MAX_PWM_CEILING` capped low and tune.

---

## Devlog

Every bug, every wrong turn, every fix — from the original ESP-NOW control link through the electrical rework and into the vision pivot — is in [`devlog.md`](./devlog.md).

> **Running into a debugging, wiring/connection, or logic issue?** Check [`devlog.md`](./devlog.md) first — it's a running log of mistakes actually made on this project and how each was root-caused and fixed (power/brownout issues, pin conflicts, driver mismatches, etc.). Good chance whatever you're hitting has already been hit and solved here.

---

*Building toward autonomous edge robotics, one phase at a time. EdgeRover started on a human's stick — now it's learning to follow on its own.*
