#!/usr/bin/env python3
"""
Geometry model of the follower: for each ground distance, what the
camera sees and whether the detector can still read the tag.

Answers the two questions that decide whether "stop at 15 cm" is
physically reachable with the tag on a back pocket:

  * viewing angle — a tag hanging vertically on the pocket is seen
    almost edge-on when the rover is right at the heels and looking
    steeply up. tag36h11 decodes reliably to ~60 deg off-normal and falls
    apart past ~70-75 deg.
  * pixel size — the tag needs roughly >= 24 px per side in the frame
    the detector decodes (a bit more with QUAD_DECIMATE 3-4 so the quad
    is still found at the decimated scale).

Examples
  python3 follow_geometry.py                        # defaults from constants.h
  python3 follow_geometry.py --cam-height 55        # camera on a mast
  python3 follow_geometry.py --tag-pitch 35         # tag tilted 35 deg to face downward
  python3 follow_geometry.py --tag-size 15 --focal 300 --frame-width 320
"""
import argparse
import math


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--cam-height", type=float, default=12.0, help="CAMERA_HEIGHT_CM")
    ap.add_argument("--tag-height", type=float, default=85.0, help="TAG_HEIGHT_CM (centre of tag)")
    ap.add_argument("--tag-size", type=float, default=10.0, help="REAL_TAG_SIZE_CM")
    ap.add_argument("--focal", type=float, default=600.0, help="FOCAL_LENGTH_PX")
    ap.add_argument("--frame-width", type=int, default=640)
    ap.add_argument("--frame-height", type=int, default=480)
    ap.add_argument("--tag-pitch", type=float, default=0.0,
                    help="deg the tag's face is tilted DOWN toward the floor (0 = hanging vertical)")
    ap.add_argument("--max-view-angle", type=float, default=65.0, help="deg off-normal still decodable")
    ap.add_argument("--min-px", type=float, default=24.0, help="min tag side in px to decode")
    ap.add_argument("--tilt-max", type=float, default=85.0, help="TILT_MAX_DEG")
    args = ap.parse_args()

    rise = args.tag_height - args.cam_height
    print(f"camera {args.cam_height:.0f} cm, tag centre {args.tag_height:.0f} cm "
          f"(rise {rise:.0f} cm), tag {args.tag_size:.0f} cm, f={args.focal:.0f} px, "
          f"tag pitched down {args.tag_pitch:.0f} deg")
    print()
    print(f"{'ground':>7} {'slant':>6} {'elev':>6} {'view':>6} {'size':>6}  verdict")
    print(f"{'cm':>7} {'cm':>6} {'deg':>6} {'deg':>6} {'px':>6}")

    first_ok = None
    for ground in [10, 15, 20, 25, 30, 40, 50, 60, 80, 100, 150, 200, 250, 300, 400]:
        slant = math.hypot(ground, rise)
        elev = math.degrees(math.atan2(rise, ground))
        view = abs(elev - args.tag_pitch)           # angle between line of sight and tag normal
        size = args.focal * args.tag_size / slant   # unforeshortened side (horizontal edges)
        problems = []
        if view > args.max_view_angle:
            problems.append(f"view angle {view:.0f} > {args.max_view_angle:.0f}")
        if size < args.min_px:
            problems.append(f"{size:.0f} px < {args.min_px:.0f}")
        if elev > args.tilt_max:
            problems.append(f"needs tilt {elev:.0f} > TILT_MAX {args.tilt_max:.0f}")
        # can the whole tag fit vertically? (foreshortened height)
        tag_h_px = size * math.cos(math.radians(view))
        if tag_h_px > args.frame_height * 0.9:
            problems.append("tag taller than frame")
        verdict = "ok" if not problems else "; ".join(problems)
        if not problems and first_ok is None:
            first_ok = ground
        print(f"{ground:7.0f} {slant:6.0f} {elev:6.1f} {view:6.1f} {size:6.0f}  {verdict}")

    print()
    if first_ok is None:
        print("No distance in the table is detectable — raise the camera or pitch the tag.")
    else:
        print(f"Closest reliably detectable ground distance with this geometry: ~{first_ok} cm")
        if first_ok > 15:
            need = rise_for(15, args.max_view_angle + args.tag_pitch)
            print(f"To stop at 15 cm: keep the tag centre within ~{need:.0f} cm above the lens "
                  f"(mast the camera to ~{args.tag_height - need:.0f} cm), or pitch the tag down by "
                  f"~{math.degrees(math.atan2(rise, 15)) - args.max_view_angle:.0f} deg on a wedge.")


def rise_for(ground, max_angle_deg):
    return ground * math.tan(math.radians(max_angle_deg))


if __name__ == "__main__":
    main()
