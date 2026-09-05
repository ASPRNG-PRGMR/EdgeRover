#!/usr/bin/env python3
"""
Fit the camera model the AprilTag follower needs: the focal length in
pixels (FOCAL_LENGTH_PX) and, optionally, the servo "level" offset
(SERVO_LEVEL_DEG correction). No OpenCV, no training data — a least
squares fit of the pinhole model to a handful of measurements you take
with a tape measure.

Data collection (firmware built with CALIBRATION_LOG 1 in constants.h):

  1. Put the rover on the floor, camera tilt at 0 (the firmware prints
     tilt= in every telemetry line; set TILT_SEARCH_RANGE_CM huge or
     just hold the tag level with the lens so the servo settles at 0).
  2. Hold the tag square-on to the lens at a MEASURED straight-line
     distance from the lens: 30, 50, 80, 120, 160 cm ... at least four
     points, the more spread the better.
  3. At each distance copy a few "CAL,size_px,cx,cy,tilt_deg" lines
     from the serial monitor into a CSV with the true distance in front:

         distance_cm,size_px,cx,cy,tilt_deg
         30,198.4,321.0,238.5,0.0
         50,119.1,318.2,241.0,0.0
         ...

     (cx, cy, tilt_deg are optional; only distance_cm and size_px are
     required for the focal length fit.)

  4. Run:   python3 fit_camera_model.py calib.csv --tag-size 10.0

The script prints FOCAL_LENGTH_PX to paste into constants.h, the
residual of each point, and — if the second experiment below was done
— the corrected SERVO_LEVEL_DEG.

Optional tilt-offset experiment (fixes "the servo's 90° isn't really
level"): with the rover on the floor at a measured GROUND distance from
a wall, tape the tag to the wall at a measured HEIGHT above the lens
and let the servo track it. Add rows with columns
ground_cm,height_cm,tilt_deg,cy to a second CSV and pass it as
--tilt-csv. The fit solves for the constant tilt offset that makes the
predicted elevation match atan2(height, ground).
"""
import argparse
import csv
import math
import sys

import numpy as np


def read_csv(path):
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        sys.exit(f"{path}: no rows")
    return rows


def fit_focal(rows, tag_size_cm):
    d = np.array([float(r["distance_cm"]) for r in rows])
    s = np.array([float(r["size_px"]) for r in rows])
    # Pinhole: size_px = f * S / d  ->  size_px = f * (S/d). Linear in f,
    # through the origin; least squares in that form weights every point
    # by its own pixel size, which is what we want (near points are
    # measured more precisely in pixels than far ones).
    x = tag_size_cm / d
    f = float(np.dot(x, s) / np.dot(x, x))
    pred_d = f * tag_size_cm / s
    return f, d, s, pred_d


def fit_tilt_offset(rows, focal_px, principal_y):
    # Predicted elevation with the firmware's model:
    #   elev = tilt + atan2(principal_y - cy, f)
    # True elevation: atan2(height, ground). Offset = mean(true - predicted).
    errs = []
    for r in rows:
        ground = float(r["ground_cm"])
        height = float(r["height_cm"])
        tilt = float(r["tilt_deg"])
        cy = float(r["cy"])
        in_frame = math.degrees(math.atan2(principal_y - cy, focal_px))
        true_elev = math.degrees(math.atan2(height, ground))
        errs.append(true_elev - (tilt + in_frame))
    errs = np.array(errs)
    return float(errs.mean()), float(errs.std(ddof=1)) if len(errs) > 1 else 0.0


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", help="distance_cm,size_px[,cx,cy,tilt_deg]")
    ap.add_argument("--tag-size", type=float, required=True, help="REAL_TAG_SIZE_CM (outer black border)")
    ap.add_argument("--frame-width", type=int, default=640)
    ap.add_argument("--frame-height", type=int, default=480)
    ap.add_argument("--tilt-csv", help="optional: ground_cm,height_cm,tilt_deg,cy rows for the servo level fit")
    ap.add_argument("--servo-level-deg", type=float, default=90.0, help="current SERVO_LEVEL_DEG in constants.h")
    ap.add_argument("--servo-up-sign", type=float, default=1.0, help="current SERVO_UP_SIGN in constants.h")
    args = ap.parse_args()

    rows = read_csv(args.csv)
    f, d, s, pred_d = fit_focal(rows, args.tag_size)

    print(f"Fitted focal length: {f:.1f} px  (frame {args.frame_width}x{args.frame_height})")
    hfov = 2 * math.degrees(math.atan(args.frame_width / (2 * f)))
    print(f"Implied horizontal FOV: {hfov:.1f} deg")
    print()
    print(f"{'true_cm':>8} {'size_px':>8} {'pred_cm':>8} {'err_cm':>7} {'err_%':>6}")
    for di, si, pi in zip(d, s, pred_d):
        print(f"{di:8.1f} {si:8.1f} {pi:8.1f} {pi - di:7.1f} {100 * (pi - di) / di:6.1f}")
    rms = float(np.sqrt(np.mean((pred_d - d) ** 2)))
    worst = float(np.max(np.abs(100 * (pred_d - d) / d)))
    print(f"\nRMS range error {rms:.1f} cm, worst {worst:.1f} %")
    if worst > 8:
        print("  ! > 8 % at some point: check the tag is square to the lens, the size is the")
        print("    outer BLACK border, and that the same frame size was used for every row.")

    print("\nPaste into constants.h:")
    print(f"#define FOCAL_LENGTH_PX     {f:.1f}f")
    print("#define FOCAL_IS_CALIBRATED 1")

    if args.tilt_csv:
        trows = read_csv(args.tilt_csv)
        off, sd = fit_tilt_offset(trows, f, args.frame_height / 2)
        new_level = args.servo_level_deg - args.servo_up_sign * off
        print(f"\nTilt offset: the camera actually looks {off:+.2f} deg from where the firmware "
              f"thinks (spread {sd:.2f} deg over {len(trows)} rows)")
        print("Paste into constants.h:")
        print(f"#define SERVO_LEVEL_DEG     {new_level:.1f}f")


if __name__ == "__main__":
    main()
