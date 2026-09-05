#ifndef CONSTANTS_H
#define CONSTANTS_H

// =====================================================================
// EdgeRover AprilTag follower — all tunables in one place.
//
// Target hardware: ESP32-S3-CAM "N16R8" (16 MB flash, 8 MB octal PSRAM)
// with the OV3660 module, plus one SG90 servo tilting the camera.
// Steering is done by the rover itself (tank turn / pivot) via the
// unmodified bot_controller/receiver — see packet.h.
//
// Geometry model (the "Pythagoras" part):
//   The tag rides on a back pocket, ~TAG_HEIGHT_CM off the floor, while
//   the camera sits ~CAMERA_HEIGHT_CM off the floor. The pinhole model
//   only gives the SLANT range r along the line of sight. As the rover
//   closes in, the tag climbs in the frame and the servo tilts up to
//   keep it centred, so the line of sight steepens. With the tilt
//   angle known from the servo (plus the tag's remaining offset from
//   frame centre), the vertical leg is h = r·sin(elevation) and the
//   ground distance is  d = sqrt(r² − h²)  — that is what the stop /
//   slow / back-off thresholds below are compared against.
// =====================================================================

// --- Camera / frame -----------------------------------------------------
// VGA + quad_decimate 3 gives the best detection range per millisecond
// on the S3: quads are searched at ~213x160, the code is decoded at full
// VGA. If the "det=" time in the telemetry stays above ~150 ms, drop to
// FRAMESIZE_HVGA (480x320) or FRAMESIZE_QVGA (320x240) and scale
// FOCAL_LENGTH_PX by the new width / 640 (the focal length in pixels
// scales linearly with frame width for the same lens).
#define CAM_FRAMESIZE      FRAMESIZE_VGA
#define FRAME_WIDTH        640
#define FRAME_HEIGHT       480

// OV3660 modules ship mounted upside-down relative to the ESP32-S3-CAM
// board silkscreen; the stock CameraWebServer example flips them back.
// If the rover tilts the camera the WRONG way when the tag rises, flip
// CAMERA_VFLIP. If it steers the wrong way but tilt is right, flip
// CAMERA_HMIRROR.
#define CAMERA_VFLIP       1
#define CAMERA_HMIRROR     0

// --- Detector ------------------------------------------------------------
#define QUAD_DECIMATE       3.0f   // 1.0 = max range, ~4x slower than 2.0
#define QUAD_SIGMA          0.0f   // gaussian blur before quad search; 0.8 helps noisy frames
#define REFINE_EDGES        1
#define DECODE_SHARPENING   0.25f
#define MIN_DECISION_MARGIN 30.0f  // reject weak decodes (cloth folds, motion blur)
#define MAX_HAMMING         0      // 0 = only perfect decodes; >0 raises false positives
// -1 = follow any tag36h11 tag; otherwise only this ID. NOTE: the ESP32
// port of the library trims tag36h11 to 35 codes to save RAM, so print
// an ID in 0..34.
#define TARGET_TAG_ID       -1

// --- Tag geometry ----------------------------------------------------------
// Side length of the tag's OUTER BLACK BORDER — that is where the
// detector's corners land, not the white quiet zone around it. A 10 cm
// tag on a stiff card fits a back pocket and is decodable to ~2 m at VGA.
#define REAL_TAG_SIZE_CM    10.0f

// --- Camera intrinsics -----------------------------------------------------
// FOCAL_LENGTH_PX is an ESTIMATE for the stock ~66° OV3660 lens at VGA
// (≈0.94 × frame width). Wide-angle 120°/160° lens variants are ~2-3x
// smaller. Calibrate with model/fit_camera_model.py, paste the result
// here and set FOCAL_IS_CALIBRATED to 1 — until then forward speed is
// capped at UNCALIBRATED_PWM_CAP.
#define FOCAL_LENGTH_PX     600.0f
#define FOCAL_IS_CALIBRATED 0
#define PRINCIPAL_X_PX      (FRAME_WIDTH  * 0.5f)
#define PRINCIPAL_Y_PX      (FRAME_HEIGHT * 0.5f)

// 0 = range from apparent tag size (robust, cheap).
// 1 = range from the homography pose solver (apriltag_pose.h). Slightly
//     better at steep viewing angles, slightly more CPU, needs the same
//     focal length calibration.
#define USE_POSE_ESTIMATE   0

// --- Mounting geometry (nominal, cm above the floor) -------------------------
// Used for two things only: the "search" tilt when the tag is lost, and
// the h= sanity value printed in telemetry (should hover near
// TAG_HEIGHT_CM − CAMERA_HEIGHT_CM once FOCAL_LENGTH_PX is right).
#define CAMERA_HEIGHT_CM    12.0f
#define TAG_HEIGHT_CM       85.0f

// --- Tilt servo (SG90) -------------------------------------------------------
// GPIO21 is a free header pin on the ESP32-S3-CAM boards that use the
// ESP32S3_EYE camera pinout (camera: 4-18 range; SD: 38/39/40; RGB LED:
// 48; strapping: 0/3/45/46; USB: 19/20; octal PSRAM: 35-37). 14, 41, 42
// or 47 are equally fine if 21 is taken.
// Power the servo from the 5 V BEC rail, never from the ESP32 3V3 pin —
// an SG90 pulls 250-650 mA when it moves. Its ground goes to the star
// ground point like everything else.
#define SERVO_PIN           21
#define SERVO_MIN_US        500     // pulse at 0°
#define SERVO_MAX_US        2500    // pulse at 180°
#define SERVO_LEVEL_DEG     90.0f   // servo angle at which the camera looks horizontal
#define SERVO_UP_SIGN       1.0f    // +1: larger servo angle tilts the camera UP; -1 if mirrored
#define TILT_MIN_DEG        -10.0f  // camera tilt limits, degrees above horizontal
#define TILT_MAX_DEG        85.0f
#define TILT_RATE_DEG_PER_S 240.0f  // SG90 is ~0.1 s/60° unloaded; this stays under that
#define TILT_KP             0.7f    // fraction of the in-frame elevation error removed per frame
#define TILT_DEADZONE_DEG   1.0f
#define TILT_SEARCH_RANGE_CM 100.0f // when lost, aim where the tag would be at this ground distance

// --- Distances (ground / horizontal, cm) -------------------------------------
#define MIN_FOLLOW_DISTANCE_CM 15.0f  // stop here (the "15 cm" requirement)
#define RESUME_DISTANCE_CM     25.0f  // start moving again only past this (hysteresis)
#define SLOW_DISTANCE_CM       80.0f  // full speed beyond this, ramp down inside it
#define BACKOFF_DISTANCE_CM    10.0f  // reverse gently if closer than this; 0 disables
#define BACKOFF_PWM            70
#define RANGE_EMA_ALPHA        0.5f   // 1.0 = no filtering

// --- Steering ------------------------------------------------------------------
// turn = STEER_KP × bearing_error_deg (+ STEER_KD × d/dt), clamped to ±1.
// 0.03 → half lock (inner wheel stopped) at ~17°, full lock at ~33°.
// The VGA field of view is about ±28°, so full lock is only reached with
// the tag at the very edge of the frame.
#define STEER_KP            0.03f
#define STEER_KD            0.0f
#define STEER_DEADZONE_DEG  2.0f

// --- Motor limits --------------------------------------------------------------
#define MAX_PWM_CEILING     200   // absolute cap while following (walking pace)
#define MIN_MOVING_PWM      60    // below this the drivetrain may not overcome friction
#define PIVOT_PWM_MAX       140   // in-place pivot speed at full turn
#define PIVOT_PWM_MIN       80    // ... and at the deadzone edge
#define UNCALIBRATED_PWM_CAP 120  // forward cap while FOCAL_IS_CALIBRATED == 0

// --- Lost-target behaviour -------------------------------------------------------
#define LOST_HOLD_MS        300   // keep the last command this long (blink / single missed frame)
#define LOST_PIVOT_MS       1500  // then pivot toward the side the tag left on...
#define LOST_PIVOT_PWM      90
                                  // ...then stop and point the camera at the search tilt.

// --- Link / sender task ----------------------------------------------------------
// The receiver drops into failsafe 200 ms after the last packet. AprilTag
// detection alone can take 100+ ms per frame, so packets are sent from a
// dedicated 50 Hz task (command_sender.*) that repeats the latest command.
#define SEND_INTERVAL_MS        20
#define COMMAND_STALE_MS        400  // no fresh command for this long → send zeros
#define PWM_SLEW_UP_PER_TICK    12   // ramp-up limit per 20 ms tick (0 → 200 in ~0.35 s)
#define PWM_SLEW_DOWN_PER_TICK  40   // decelerate / reverse faster than we accelerate
#define VISION_MODE_ID          2    // ControlPacket.mode value for autonomous mode

// --- Telemetry -------------------------------------------------------------------
#define TELEMETRY_INTERVAL_MS   100
// 1 = also print "CAL,size_px,cx,cy,tilt_deg" lines for model/fit_camera_model.py
#define CALIBRATION_LOG         0

#endif // CONSTANTS_H
