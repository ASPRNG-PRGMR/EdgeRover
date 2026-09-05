# vision_control/model — the follower's measurement model

There is deliberately **no neural network here**. The accuracy problem in
an AprilTag follower is not *finding* the tag (the detector does that
deterministically) but turning "tag is N pixels wide at pixel (x, y)"
into "the person is D cm ahead, B degrees to the right". That is a
geometry model with two unknowns you measure once, and this folder is
the tooling for it.

## The model the firmware runs (range_model.cpp)

```
slant r      = FOCAL_LENGTH_PX × REAL_TAG_SIZE_CM / size_px      (pinhole)
direction    = (dx, −dy, f) normalised, dx/dy = pixel offset from centre
rotate       about the x axis by the SERVO TILT (camera pitched up)
height   h   = y component after rotation
ground   d   = sqrt(r² − h²)                                     (Pythagoras)
bearing      = atan2(x, z)   → steering
elevation    = atan2(h, d)   → telemetry / sanity
```

Why the servo matters: with the tag on a back pocket (~85 cm up) and the
camera on the chassis (~12 cm up), a rover 15 cm behind the heels is
looking up at ~78°. The pinhole range along that line of sight is ~75 cm,
not 15 cm — without the tilt angle the rover would happily keep driving.
The servo angle plus the tag's residual offset from frame centre gives the
line-of-sight elevation; Pythagoras gives the horizontal leg.

## Scripts

| Script | What it does |
|---|---|
| `follow_geometry.py` | Prints, per ground distance, the slant range, elevation, tag viewing angle and pixel size, and flags where detection breaks. Run it **before** building anything mechanical. |
| `fit_camera_model.py` | Least-squares fit (see `calib_example.csv` for the input shape) of `FOCAL_LENGTH_PX` (and optionally the servo level offset) from a CSV of tape-measured distances vs. logged `size_px`. Prints the lines to paste into `constants.h`. |

Both need only Python 3 and numpy.

## The 15 cm problem (read this first)

`python3 follow_geometry.py` with the default heights says:

```
 ground  slant   elev   view   size  verdict
     15     75   78.4   78.4     81  view angle 78 > 65
     30     79   67.7   67.7     76  view angle 68 > 65
     40     83   61.3   61.3     72  ok
    ...
    200    213   20.1   20.1     28  ok
    250    260   16.3   16.3     23  23 px < 24
Closest reliably detectable ground distance with this geometry: ~40 cm
```

A tag hanging flat on a pocket is seen nearly edge-on from 15 cm away at
floor level; tag36h11 stops decoding somewhere past 65–75° off-normal.
Two cheap fixes, either one is enough:

* **Pitch the tag down ~40°** — mount it on a small wedge (or a folded
  card) so its face points down toward the rover. `--tag-pitch 40` makes
  every distance from 10 cm out decodable.
* **Raise the camera** to ~50–55 cm on a mast (`--cam-height 55`).

Range at the far end is set by tag size and frame size: a 10 cm tag at
VGA is readable to ~2 m; a 15 cm tag to ~3 m. `QUAD_DECIMATE` trades that
range for frame rate.

## Calibration procedure

1. Build with `CALIBRATION_LOG 1`, flash, open the serial monitor.
2. Rover on the floor, tag square to the lens at 30 / 50 / 80 / 120 / 160 cm
   (tape measure from the lens to the tag face). Copy a `CAL,...` line at
   each distance into `calib.csv` as `distance_cm,size_px`.
3. `python3 fit_camera_model.py calib.csv --tag-size 10.0`
4. Paste `FOCAL_LENGTH_PX` and `FOCAL_IS_CALIBRATED 1` into `constants.h`.
   Until that flag is set, forward PWM is capped at `UNCALIBRATED_PWM_CAP`.
5. Optional: tape the tag to a wall at a measured height, park the rover
   at a measured distance, let the servo track it, and feed
   `ground_cm,height_cm,tilt_deg,cy` rows via `--tilt-csv` to correct
   `SERVO_LEVEL_DEG`.

Sanity check while driving: the `h=` value in the telemetry should sit
near `TAG_HEIGHT_CM − CAMERA_HEIGHT_CM` at every distance. If it drifts
with distance, the focal length is off; if it is constant but wrong, the
servo level angle is off.

## When a learned model *would* be worth it

Only if the marker goes away — following a hand/shoe/person silhouette
with no tag. That is the Edge Impulse FOMO / ESP-DL path already
sketched in the top-level README. It is a different project: a detector
would replace `tag_detector.cpp` and hand `range_model.cpp` a bounding
box instead of tag corners; everything from the servo down stays.
