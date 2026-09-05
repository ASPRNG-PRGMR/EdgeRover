#ifndef TILT_SERVO_H
#define TILT_SERVO_H

// SG90 camera-tilt servo. All angles here are CAMERA TILT in degrees
// above horizontal (0 = level, +45 = looking up), not raw servo angle;
// the servo mapping (SERVO_LEVEL_DEG, SERVO_UP_SIGN, pulse range) lives
// in constants.h.

// Call once in setup(), before camera_init(). Configures a 50 Hz PWM
// output on SERVO_PIN and moves the camera to the search tilt.
// Returns true on success.
bool tilt_servo_init();

// Where the camera should point. Clamped to TILT_MIN/MAX_DEG. The move
// itself is rate-limited inside tilt_servo_update().
void tilt_servo_set_target(float tilt_deg);

// Call once per loop iteration. Steps the commanded tilt toward the
// target at TILT_RATE_DEG_PER_S and writes the pulse.
void tilt_servo_update();

// Current commanded camera tilt (degrees above horizontal). This is
// what the range model uses as the line-of-sight elevation offset.
float tilt_servo_get_tilt();

// Tilt at which a tag TAG_HEIGHT_CM off the floor would be centred when
// the rover is TILT_SEARCH_RANGE_CM away — the resting/search pose.
float tilt_servo_search_tilt();

#endif // TILT_SERVO_H
