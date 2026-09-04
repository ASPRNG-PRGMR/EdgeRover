# AprilTag-Following Autonomous Rover — Project Reference

A rover that visually tracks an AprilTag (worn/carried by the user), steers toward it, matches distance by approaching/stopping, and reacts to the tag leaving frame — entirely on local compute, no host PC, no ROS/micro-ROS required for this core behavior.

---

## 1. Goal & Scope

- **Primary behavior:** Detect an AprilTag in the camera frame → steer toward it → stop when close enough → hold if tag stops moving.
- **No CNN required.** This is classical computer vision (fiducial marker detection), not deep learning. Deterministic, lightweight, no training data needed.
- **No ROS / micro-ROS needed** for this core loop — it's a local sensor-read → decision → motor-drive loop running entirely on the ESP32(-S3).
- **Explicitly out of scope for v1:** path planning, SLAM, mapping, multi-target CNN detection (hand/shoe). Those are noted as future extensions in §8.
- **Frame-exit reactive behavior** (tag leaving top/left/right of frame) is designed for later — noted in §7 as a stretch goal, not required for first working version.

---

## 2. Hardware

| Component | Role | Notes |
|---|---|---|
| ESP32-S3-CAM | Main compute + camera | Has PSRAM (check your board — most have 8MB), enough headroom for camera buffer + AprilTag detection |
| 12V battery | Power source | Confirm chemistry (LiPo / Li-ion pack / SLA) — affects safe low-voltage cutoff |
| BEC (Battery Eliminator Circuit) | Steps 12V down to a regulated voltage for logic/servo rail | Confirm output voltage/current rating matches ESP32 (needs 5V or 3.3V depending on your board's onboard regulator) and motor driver logic needs |
| Motor driver (TBD) | Drives drive motors from 12V, controlled by ESP32 GPIO/PWM | e.g., L298N, TB6612FNG, or a BTS7960 if higher current — pick based on your motor stall current |
| Drive motors + chassis | Physical movement | Differential drive (2 motors, skid steer) is simplest to reason about with the steering math below |
| AprilTag (printed) | The "target" the rover follows | Print at a **known, fixed physical size** — this size is a required constant in the distance math (§5) |

### 2.1 Power architecture (sanity check before wiring)

```
12V Battery ──┬── Motor Driver (12V rail, high current, motors)
              │
              └── BEC ── regulated V (5V/3.3V) ── ESP32-S3-CAM logic rail
```

- **Never** power the ESP32-S3-CAM camera module directly off unregulated 12V — camera + WiFi radio are sensitive to voltage sag/spikes, hence the BEC.
- Common ground between the battery rail, BEC output, ESP32, and motor driver logic pins is mandatory — a floating/missing common ground is the #1 cause of "motors twitch but nothing responds correctly" bugs.
- Add a fuse or resettable fuse (polyfuse) between battery and BEC input — cheap insurance.
- Motor stall current spikes can brown out shared rails — if you see the ESP32 randomly resetting when motors start/stop, this is almost always a power/decoupling issue, not code. Add a bulk capacitor (e.g., 1000µF) across the BEC output near the ESP32.

---

## 3. Software Stack

| Layer | Tool |
|---|---|
| Framework | ESP-IDF (recommended over Arduino for this — more control over camera driver + easier to integrate a C detection library) |
| Camera driver | `esp32-camera` component (Espressif's official driver) |
| AprilTag detection | Port of the UMich `apriltag` C library, or a lighter ArUco-style detector if a full AprilTag port proves too heavy — evaluate both for your frame rate needs |
| Motor control | ESP32 LEDC peripheral for PWM (both drive speed and, if used, steering-servo control) |

### 3.1 Why not micro-ROS for this part

You already reasoned through this correctly in earlier discussion: this is a single rover, single behavior, no need for the ROS ecosystem (RViz/nav2/multi-node graph). A local control loop is simpler, faster, and has zero network dependency. Keep micro-ROS in your back pocket only if you later want to visualize telemetry on a PC or coordinate multiple robots — not needed to make this rover move.

---

## 4. Core Control Loop (Pseudocode)

```
loop (target ~15–30 fps, camera-limited):
    frame = capture_frame()
    detections = apriltag_detect(frame)

    if len(detections) > 0:
        tag = pick_best(detections)      # e.g., highest decision_margin / largest area
        cx, cy = tag.center
        size_px = tag.apparent_size      # e.g., avg side length in pixels

        steer_error = compute_steer_error(cx, frame_width)
        distance_est = estimate_distance(size_px)

        if distance_est <= STOP_DISTANCE:
            motors.stop()
        else:
            speed = compute_speed(distance_est)
            turn  = compute_turn(steer_error)
            motors.drive(speed, turn)

    else:
        motors.stop()   # v1: simple safe default. See §7 for frame-exit logic upgrade.
```

---

## 5. Math Reference

### 5.1 Steering error (horizontal alignment)

Let the frame have width `W` pixels. Tag center horizontal pixel = `cx`.

```
frame_center_x = W / 2
error_x = cx - frame_center_x
```

- `error_x > 0` → tag is to the **right** of center → steer right
- `error_x < 0` → tag is to the **left** of center → steer left
- `|error_x| < deadzone` → go straight (deadzone prevents jitter from noise — start with ~5% of `W`)

**Normalized error** (useful for PID / proportional control, independent of resolution):

```
error_x_norm = error_x / (W / 2)      # range: -1.0 (full left) to +1.0 (full right)
```

### 5.2 Turn command (proportional control)

Simplest usable controller — proportional-only:

```
turn = Kp * error_x_norm
```

- `Kp` is tuned empirically — start small (e.g., 0.3–0.5) and increase until steering response feels responsive without oscillating.
- If you get overshoot/oscillation (rover wags side to side), add a derivative term (PD controller):

```
turn = Kp * error_x_norm + Kd * (error_x_norm - prev_error_x_norm) / dt
```

- Full PID (adding an integral term) is usually unnecessary here — steady-state offset isn't a meaningful failure mode for "steer toward a moving target."

### 5.3 Distance estimation from apparent tag size

This is the key formula — it replaces needing depth sensors or stereo vision.

**Pinhole camera model, similar triangles:**

```
distance = (real_tag_size × focal_length_px) / apparent_tag_size_px
```

Where:
- `real_tag_size` = the actual physical side length of your printed tag (e.g., in cm) — **you define this when you print it, keep it fixed**
- `focal_length_px` = your camera's focal length **in pixels** (not mm) — see §5.4 for how to get this
- `apparent_tag_size_px` = the tag's side length as measured in the current frame, in pixels (AprilTag library gives you corner coordinates — average the 4 side lengths, or use the diagonal / √2)

### 5.4 Getting `focal_length_px` (one-time calibration)

Two options:

**Option A — Simple manual calibration (fast, good enough for this project):**
1. Place the tag at a known distance `D_known` from the camera (measure with a tape measure, e.g., 50cm).
2. Capture a frame, measure `apparent_tag_size_px` at that distance.
3. Rearrange the formula:
```
focal_length_px = (apparent_tag_size_px × D_known) / real_tag_size
```
4. Hardcode this `focal_length_px` constant in your firmware. Re-calibrate if you change camera, lens, or resolution.

**Option B — Formal camera calibration (more accurate, more setup):**
Use a checkerboard pattern + OpenCV's `cv2.calibrateCamera` on your PC (using saved frames from the ESP32-CAM) to get the full camera intrinsic matrix, which includes focal length in pixels directly (`fx`, `fy` terms). Overkill for a first version, but worth doing later if distance accuracy matters more.

### 5.5 Stop / speed decision

```
STOP_DISTANCE = <your chosen threshold, e.g., 40 cm>
SLOW_DISTANCE = <your chosen threshold, e.g., 100 cm>

if distance <= STOP_DISTANCE:
    speed = 0
elif distance <= SLOW_DISTANCE:
    speed = map(distance, STOP_DISTANCE, SLOW_DISTANCE, MIN_SPEED, MAX_SPEED)
else:
    speed = MAX_SPEED
```

Where `map()` is standard linear interpolation:
```
map(x, in_min, in_max, out_min, out_max) =
    (x - in_min) × (out_max - out_min) / (in_max - in_min) + out_min
```

This gives a smooth deceleration curve as the rover approaches you, rather than full speed → sudden stop.

### 5.6 "Stopped following stopped" detection (hold position when you stop moving)

Track `distance` (or raw `size_px`) over the last `N` frames (e.g., N = 10 at 20fps = 0.5s window):

```
size_history = [last N values of apparent_tag_size_px]
variance = statistical_variance(size_history)

if variance < STILL_THRESHOLD and distance > STOP_DISTANCE:
    # target present, roughly same distance for the last N frames → they've stopped
    motors.stop()   # or hold at current position
```

Tune `STILL_THRESHOLD` empirically — you want it to tolerate normal walking-speed size fluctuation but catch "person has stopped moving."

### 5.7 PWM duty cycle → motor speed (ESP32 LEDC)

```
duty_value = (speed_fraction × (2^resolution_bits - 1))
```

Where `speed_fraction` is your 0.0–1.0 desired speed (from §5.5), and `resolution_bits` is your configured LEDC timer resolution (commonly 8-bit → max value 255, or higher for finer control).

For **differential/skid steering**, convert `speed` + `turn` into individual left/right motor speeds:

```
left_speed  = clamp(speed - turn, MIN_PWM, MAX_PWM)
right_speed = clamp(speed + turn, MIN_PWM, MAX_PWM)
```

(Sign convention depends on your wiring — verify empirically which sign makes the rover turn toward positive `turn`, then keep it consistent.)

---

## 6. Constants You'll Need to Define (checklist before first test)

- [ ] `real_tag_size` — measure your printed tag precisely (cm)
- [ ] `focal_length_px` — calibrate per §5.4
- [ ] `STOP_DISTANCE`, `SLOW_DISTANCE` — pick based on rover size / safety margin
- [ ] `Kp` (and `Kd` if needed) — tune empirically on the bench (wheels off ground) first
- [ ] `deadzone` — steering noise tolerance, start ~5% of frame width
- [ ] `STILL_THRESHOLD` — variance tolerance for "target has stopped"
- [ ] `MIN_SPEED` / `MAX_SPEED` (PWM duty range) — respect your motor driver + battery limits

---

## 7. Frame-Exit Reactive Behavior (Stretch Goal — Later)

Noted for when you're ready to revisit (per your earlier design):

| Tag exits frame via | Likely meaning | Reaction (to design later) |
|---|---|---|
| Left edge | Target moved left | Rotate/search left |
| Right edge | Target moved right | Rotate/search right |
| Top edge, was shrinking | Target moving away, out of vertical FOV | Continue forward briefly, then search |
| Top edge, was growing | Target got very close | Treat as stop, not search |

Key implementation note for later: track **last known `(cx, cy, size_px)` and its trend over the last few frames before loss** — this trend (not just the final position) is what disambiguates "went far" vs "got close."

---

## 8. Future Extensions (Not v1)

- Hand/shoe CNN-based following (Edge Impulse FOMO model, ESP-DL deployment) — separate project path, revisit once marker-following is solid.
- Second camera (original ESP32-CAM) for a rear-facing tag or secondary sensing.
- micro-ROS bridge — only if you want telemetry visualization on a PC or plan to integrate with other ROS-based systems later.
- Formal camera calibration (§5.4 Option B) for improved distance accuracy.

---

## 9. Suggested Build Order

1. **Bench test power rail** — confirm BEC output voltage under load, confirm ESP32-S3-CAM boots stable on it before touching motors.
2. **Camera + AprilTag detection running standalone** — print a tag, get detection + corner coordinates printed over serial, no motors yet.
3. **Calibrate `focal_length_px`** (§5.4) using the same setup.
4. **Wheels-off-ground bench test** — run the full loop, watch computed `turn`/`speed` values over serial, verify they make sense as you move the tag around, before ever powering motors.
5. **Motors on, low speed cap** — first physical test with `MAX_SPEED` deliberately low.
6. **Tune `Kp`, `deadzone`, `STOP_DISTANCE`** on the floor.
7. Only then consider §7/§8 extensions.
