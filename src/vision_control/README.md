# vision_control — AprilTag follower: setup, calibration, testing

Everything needed to go from a bare ESP32-S3-CAM to a rover that follows
an AprilTag on your back pocket. The top-level README covers the rover's
electrical build and the manual-control link; this file covers only the
vision side. Read `model/README.md` too — it explains the distance model
and why the tag's mounting angle matters.

Contents
1. [Hardware and wiring](#1-hardware-and-wiring)
2. [Software install and board settings](#2-software-install-and-board-settings)
3. [Print and mount the tag](#3-print-and-mount-the-tag)
4. [Configure constants.h](#4-configure-constantsh)
5. [Flash and read the telemetry](#5-flash-and-read-the-telemetry)
6. [Bench tests (wheels off the ground)](#6-bench-tests-wheels-off-the-ground)
7. [Calibrate the focal length](#7-calibrate-the-focal-length)
8. [First drive and tuning](#8-first-drive-and-tuning)
9. [Troubleshooting](#9-troubleshooting)
10. [Constants reference](#10-constants-reference)

---

## 1. Hardware and wiring

| Part | Notes |
|---|---|
| ESP32-S3-CAM, module **ESP32-S3-WROOM-1 N16R8** (16 MB flash, 8 MB **octal** PSRAM) | Goouuu / Keyestudio MB0184 / Freenove S3-CAM style boards. All use the `ESP32S3_EYE` camera pin map hard-coded in `camera.cpp` |
| OV3660 camera module (stock ~66° lens) | Wide-angle 120°/160° variants work but need a ~2–3× smaller `FOCAL_LENGTH_PX` (calibration handles it) |
| SG90 micro servo | Tilts the camera up/down. Side-to-side is done by the rover itself (tank turn / pivot) |
| Existing rover receiver (`bot_controller/receiver`) | **Unchanged.** It only sees `leftPWM`/`rightPWM` |

### Servo wiring

| SG90 lead | Goes to |
|---|---|
| Orange/yellow (signal) | **GPIO21** on the ESP32-S3-CAM header (`SERVO_PIN`). 14, 41, 42 or 47 are fine alternatives; avoid 0/3/45/46 (strapping), 19/20 (USB), 35–37 (octal PSRAM), 38–40 (SD), 48 (RGB LED), 4–18 (camera) |
| Red (+) | **5 V BEC rail** — the same rail that feeds the ESP32 `5V`/`VIN`. Never the ESP32 `3V3` pin: an SG90 draws 250–650 mA when it moves |
| Brown (−) | The **star ground** point, like everything else |

Mount the servo so that its horn at `SERVO_LEVEL_DEG` (default 90°) has
the camera looking horizontal, and increasing angle tilts the lens **up**.
If your linkage works the other way, set `SERVO_UP_SIGN` to `-1`. The
servo must be able to reach at least +85° (`TILT_MAX_DEG`) — that is what
"looking up at someone's pocket from 15 cm" needs.

### Camera placement

Run the geometry check before deciding where the camera goes:

```
python3 model/follow_geometry.py --cam-height <lens height cm> --tag-height <tag centre height cm>
```

With the lens ~12 cm up and a tag ~85 cm up (back pocket), a flat tag is
seen ~78° edge-on at 15 cm and **will not decode**; the closest working
distance is ~40 cm. Either pitch the tag ~40° downward on a wedge
(`--tag-pitch 40`) or raise the camera to ~50–55 cm on a mast. Pick one
before building.

---

## 2. Software install and board settings

1. **Arduino IDE 2.x** (or arduino-cli) with **ESP32 board support 3.x**.
   Built and checked against core 3.3.11. Core 2.x will not compile the
   ESP-NOW callbacks (`wifi_tx_info_t`) or the camera driver's API.
2. **AprilTag library:** download
   [raspiduino/apriltag-esp32](https://github.com/raspiduino/apriltag-esp32)
   as a ZIP → Sketch › Include Library › Add .ZIP Library. It is the UMich
   AprilTag 3 library with float math and the tag36h11 table trimmed to
   **35 codes** (IDs 0–34) to fit RAM. `esp32-camera` is bundled with the
   core; nothing else to install.
3. **Board menu** (Tools):

   | Setting | Value |
   |---|---|
   | Board | ESP32S3 Dev Module |
   | PSRAM | **OPI PSRAM** (N16R8 has octal PSRAM; the camera + detector buffers do not fit without it) |
   | Flash Size | 16MB (128Mb) |
   | Partition Scheme | **Huge APP (3MB No OTA/1MB SPIFFS)** — the sketch is ~1.0 MB |
   | USB CDC On Boot | Enabled (serial monitor over the USB-C port) |
   | CPU Frequency | 240 MHz |

4. Set the receiver's MAC in `apriltag_follower/espnow_tx.cpp`
   (`RECEIVER_MAC`). The receiver prints it on boot. It is the same value
   already used by `bot_controller/transmitter/espnow_tx.cpp`.

arduino-cli equivalent:

```
arduino-cli compile -b "esp32:esp32:esp32s3:PSRAM=opi,FlashSize=16M,PartitionScheme=huge_app,CDCOnBoot=cdc" src/vision_control/apriltag_follower
```

---

## 3. Print and mount the tag

- Family **tag36h11**, **ID 0–34** (the trimmed table). Generator:
  <https://github.com/AprilRobotics/apriltag-imgs> (`tag36h11/tag36_11_00007.png` etc., scale up with nearest-neighbour, no smoothing).
- **10 cm** black-border side length is the default (`REAL_TAG_SIZE_CM`).
  Readable to ~2 m at VGA; 15 cm reaches ~3 m. Measure the **outer edge of
  the black border** — that is where the detector's corners land — not the
  white margin. Keep at least one bit-width of white around it.
- Print matte, mount on **stiff card**. A tag that flexes with the
  fabric breaks the square-corner assumption and the decode.
- Set `TARGET_TAG_ID` to the printed ID once things work so a second tag
  in the room cannot hijack the rover.

---

## 4. Configure constants.h

Minimum before the first flash:

| Constant | Set to |
|---|---|
| `REAL_TAG_SIZE_CM` | measured black-border side |
| `TARGET_TAG_ID` | your ID, or `-1` for any while bringing up |
| `CAMERA_HEIGHT_CM`, `TAG_HEIGHT_CM` | lens height, tag-centre height (only drives the search tilt and the `h=` sanity value) |
| `SERVO_LEVEL_DEG`, `SERVO_UP_SIGN` | from how you mounted the servo (§1) |
| `MAX_PWM_CEILING` | leave at 200 for now; forward is capped at `UNCALIBRATED_PWM_CAP` (120) anyway until calibration |

Leave `FOCAL_LENGTH_PX` at the estimate and `FOCAL_IS_CALIBRATED 0` until §7.

---

## 5. Flash and read the telemetry

Flash `apriltag_follower/apriltag_follower.ino`, open the serial monitor at
115200. Boot prints one line per subsystem; a `FATAL:` line halts before
the sender task starts, so the receiver stays in failsafe.

Then, 10 times a second:

```
[tag 7] px=(318,161) size=61.3 margin=58 | slant=98cm bear=-0.3 elev=+36.4 tilt=29.1 ground=79cm(f=80) h=58cm | FOLLOW cmd L=120 R=120 out L=118 R=118  | det=112ms
[no tag] lost=430ms tilt=29.1 | LOST_PIVOT cmd L=90 R=-90 out L=90 R=-90  | det=95ms
```

| Field | Meaning |
|---|---|
| `px=(cx,cy)` `size` `margin` | tag centre in pixels, apparent side length, decode confidence (`MIN_DECISION_MARGIN` rejects below 30) |
| `slant` | straight-line lens → tag from the pinhole model |
| `bear` | heading error, **+ = tag is to the right** → rover should turn right |
| `elev` | line-of-sight angle above horizontal, servo tilt included |
| `tilt` | current commanded camera tilt |
| `ground(f=…)` | horizontal distance by Pythagoras, raw and filtered — the number the stop logic uses |
| `h` | vertical leg; should sit near `TAG_HEIGHT_CM − CAMERA_HEIGHT_CM` at every distance once calibrated |
| state | `IDLE` (never seen a tag, driver disarmed) · `FOLLOW` · `HOLD` (inside 15 cm) · `BACKOFF` (inside 10 cm, reversing) · `LOST_HOLD` · `LOST_PIVOT` · `LOST_STOP` |
| `cmd` / `out` | what the tracker asked for vs. what the 50 Hz sender is actually transmitting after slew limiting |
| `det` | detector time for this frame. Aim for ≤ 150 ms; see §8 |
| `(SEND FAILED)` | ESP-NOW could not hand the packet to the radio — check the receiver is powered and the MAC matches |

---

## 6. Bench tests (wheels off the ground)

Do these in order; each one validates a sign convention the next relies on.

1. **Camera alive.** `camera OK 640x480 grayscale` at boot and `det=` times
   appearing. No tag needed.
2. **Detection.** Hold the tag ~1 m in front, square to the lens. `[tag N]`
   lines with a steady `size`. If nothing: more light, less motion, check
   the ID is 0–34.
3. **Steering sign.** Move the tag to *your* left (the rover's right as it
   faces you). `bear` must go **positive** and, if the rover is armed, the
   right wheel should slow/reverse relative to the left (`L > R`). If the
   sign is backwards, flip `CAMERA_HMIRROR`.
4. **Tilt sign.** Raise the tag. `elev` in-frame error is positive and the
   servo must tilt **up** to re-centre it (`tilt` increases, `cy` returns
   toward 240). If the servo drives the wrong way: flip `SERVO_UP_SIGN` if
   `tilt` increases but the lens goes down; flip `CAMERA_VFLIP` if `tilt`
   *decreases* when the tag rises.
5. **Range trend.** Walk the tag from 2 m to 30 cm along the floor line.
   `ground` must fall monotonically and `h` stay roughly constant. If `h`
   climbs or drops with distance the focal length is off (§7); if `h` is
   constant but wrong, `SERVO_LEVEL_DEG` is off.
6. **Stop / resume.** Bring the tag in until `ground` ≤ 15 → state `HOLD`,
   wheels stop. Back out past 25 → `FOLLOW` resumes. Push in under 10 →
   `BACKOFF`, both wheels reverse at 70.
7. **Loss.** Cover the tag. `LOST_HOLD` (300 ms, last command repeated) →
   `LOST_PIVOT` toward the side it was last seen (1.5 s) → `LOST_STOP`, wheels
   zero, servo returns to the search tilt.
8. **Failsafe.** Unplug the camera ribbon or reset the S3 while the
   receiver is running: receiver prints `Link lost - entering failsafe` within
   ~200 ms and the driver goes to standby.

---

## 7. Calibrate the focal length

1. Build with `CALIBRATION_LOG 1`. The sketch adds `CAL,size_px,cx,cy,tilt_deg`
   lines.
2. Rover on the floor, tag held level with the lens and square to it, at
   tape-measured lens-to-tag distances of 30 / 50 / 80 / 120 / 160 cm.
   Copy one `CAL` line per distance into a CSV as
   `distance_cm,size_px` (see `model/calib_example.csv`).
3. `python3 model/fit_camera_model.py calib.csv --tag-size 10.0`
4. Paste the printed `FOCAL_LENGTH_PX` and `FOCAL_IS_CALIBRATED 1` into
   `constants.h`, set `CALIBRATION_LOG` back to 0, reflash. This lifts the
   forward PWM cap to `MAX_PWM_CEILING`.
5. Optional tilt-offset fit: tag taped to a wall at a measured height
   above the lens, rover parked at a measured distance, servo tracking.
   Rows of `ground_cm,height_cm,tilt_deg,cy` via `--tilt-csv` correct
   `SERVO_LEVEL_DEG`.

Worst-case error above ~8 % on any point means the tag was not square to
the lens or the size was measured to the white margin.

---

## 8. First drive and tuning

Floor test with `MAX_PWM_CEILING` at 120–150 first, someone ready to lift
the rover, tag on the wedge/mast geometry chosen in §1.

| Symptom | Knob | Direction |
|---|---|---|
| Weaves side to side while following | `STEER_KP` | lower (0.03 → 0.02); add `STEER_KD` 0.001–0.003 only if it still oscillates |
| Turns too lazily, loses the tag off the side | `STEER_KP` | raise; check `LOST_PIVOT_MS` covers a half-turn |
| Twitches when stopped | `STEER_DEADZONE_DEG` | raise (2 → 4); `PIVOT_PWM_MIN` lower |
| Stops too early / too late | `MIN_FOLLOW_DISTANCE_CM` | move; keep `RESUME_DISTANCE_CM` ≥ 8 cm above it or it hunts |
| Slams to a stop | `SLOW_DISTANCE_CM` | raise; `PWM_SLEW_DOWN_PER_TICK` lower |
| Servo hunts up and down | `TILT_KP` | lower (0.7 → 0.4); `TILT_DEADZONE_DEG` raise |
| Servo lags a rising tag | `TILT_RATE_DEG_PER_S` | raise toward 400 |
| `det=` above ~150 ms | `QUAD_DECIMATE` | 3 → 4, or `CAM_FRAMESIZE` → `FRAMESIZE_HVGA` (set `FRAME_WIDTH/HEIGHT` 480×320 and scale `FOCAL_LENGTH_PX` × 0.75) |
| Tag lost past ~2 m | tag size | bigger tag, or `QUAD_DECIMATE` 2 (costs frame rate) |
| False detections on clothing | `MIN_DECISION_MARGIN` | raise to 40–50; set `TARGET_TAG_ID` |
| Rover backs into things | `BACKOFF_DISTANCE_CM` | set 0 to disable reverse |

---

## 9. Troubleshooting

| Problem | Likely cause |
|---|---|
| `FATAL: camera_init failed` | Wrong PSRAM setting (must be OPI), ribbon seated backwards, or a board with a different pin map — compare `camera.cpp` against your board's schematic |
| `frame is WxH but constants.h says …` | `CAM_FRAMESIZE` changed without updating `FRAME_WIDTH/HEIGHT` |
| Camera works until the servo moves, then freezes | Something attached the servo with Arduino `ledcAttach()` on timer 0. `tilt_servo.cpp` uses IDF LEDC timer 1 on purpose — keep it that way |
| `no PSRAM detected` warning | Board menu PSRAM not set to OPI |
| Detection fine, motors never move, receiver says `armed=0` | Normal until the first tag is seen (`IDLE`); after that check the receiver's `armed=1 L=.. R=..` line |
| Receiver flickers in/out of failsafe | Wi-Fi channel mismatch (both sides must be STA on channel 1, which the sketches force) or the S3 is browning out — the servo must not be on the 3V3 pin |
| `ground` reads ~5× too small/large | Focal length estimate is for the 66° lens at VGA; wide-angle lens or a different frame size — calibrate |
| Servo buzzes at an end stop | `SERVO_MIN_US`/`SERVO_MAX_US` outside your unit's range (try 600/2400) or `TILT_MAX_DEG` + `SERVO_LEVEL_DEG` > 180 |
| Resets under motor load | Not a vision problem — see the EN-pin / star-ground notes in the top-level README |

---

## 10. Constants reference

All in `apriltag_follower/constants.h`, grouped as in the file.

| Group | Constants |
|---|---|
| Frame | `CAM_FRAMESIZE`, `FRAME_WIDTH`, `FRAME_HEIGHT`, `CAMERA_VFLIP`, `CAMERA_HMIRROR` |
| Detector | `QUAD_DECIMATE`, `QUAD_SIGMA`, `REFINE_EDGES`, `DECODE_SHARPENING`, `MIN_DECISION_MARGIN`, `MAX_HAMMING`, `TARGET_TAG_ID` |
| Tag / intrinsics | `REAL_TAG_SIZE_CM`, `FOCAL_LENGTH_PX`, `FOCAL_IS_CALIBRATED`, `PRINCIPAL_X_PX`, `PRINCIPAL_Y_PX`, `USE_POSE_ESTIMATE` |
| Mounting | `CAMERA_HEIGHT_CM`, `TAG_HEIGHT_CM` |
| Servo | `SERVO_PIN`, `SERVO_MIN_US`, `SERVO_MAX_US`, `SERVO_LEVEL_DEG`, `SERVO_UP_SIGN`, `TILT_MIN_DEG`, `TILT_MAX_DEG`, `TILT_RATE_DEG_PER_S`, `TILT_KP`, `TILT_DEADZONE_DEG`, `TILT_SEARCH_RANGE_CM` |
| Distances | `MIN_FOLLOW_DISTANCE_CM`, `RESUME_DISTANCE_CM`, `SLOW_DISTANCE_CM`, `BACKOFF_DISTANCE_CM`, `BACKOFF_PWM`, `RANGE_EMA_ALPHA` |
| Steering | `STEER_KP`, `STEER_KD`, `STEER_DEADZONE_DEG` |
| Motor limits | `MAX_PWM_CEILING`, `MIN_MOVING_PWM`, `PIVOT_PWM_MAX`, `PIVOT_PWM_MIN`, `UNCALIBRATED_PWM_CAP` |
| Lost target | `LOST_HOLD_MS`, `LOST_PIVOT_MS`, `LOST_PIVOT_PWM` |
| Link | `SEND_INTERVAL_MS`, `COMMAND_STALE_MS`, `PWM_SLEW_UP_PER_TICK`, `PWM_SLEW_DOWN_PER_TICK`, `VISION_MODE_ID` |
| Telemetry | `TELEMETRY_INTERVAL_MS`, `CALIBRATION_LOG` |

`packet.h` must stay byte-identical to `bot_controller/receiver/packet.h`
(v3, signed `int16_t` PWM). `espnow_tx.*` is a verbatim copy of the
transmitter's.
