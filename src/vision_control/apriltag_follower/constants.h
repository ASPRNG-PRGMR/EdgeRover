#ifndef CONSTANTS_H
#define CONSTANTS_H

// ---------------------------------------------------------------------
// All values below are placeholders. Fill in real numbers as you work
// through the roadmap's "Constants Checklist" (Phase 3/4). Nothing here
// is safe to trust until you've measured/calibrated it on your rover.
// ---------------------------------------------------------------------

// --- Camera / frame ---
#define FRAME_WIDTH   320   // QVGA. Lower this if detection FPS is too slow.
#define FRAME_HEIGHT  240

// --- Tag geometry (measure your printed tag) ---
#define REAL_TAG_SIZE_CM   0.0f   // TODO: measure the printed tag's side length

// --- Camera calibration (see roadmap Phase 3, known-distance method) ---
// focal_length_px = (apparent_tag_size_px * D_known) / REAL_TAG_SIZE_CM
#define FOCAL_LENGTH_PX    0.0f   // TODO: calibrate before trusting distance_est

// --- Distance thresholds (cm, same units as REAL_TAG_SIZE_CM) ---
#define STOP_DISTANCE_CM   40.0f
#define SLOW_DISTANCE_CM   100.0f

// --- Steering controller ---
#define STEER_KP           0.4f   // start here, tune on bench (wheels off ground)
#define STEER_KD           0.0f   // only add if steering oscillates
#define STEER_DEADZONE     0.05f  // normalized, ~5% of half-frame-width

// --- "Target has stopped" detection ---
#define STILL_HISTORY_LEN   10    // frames (~0.5s at 20fps)
#define STILL_VARIANCE_THRESHOLD  9.0f  // px^2, tune empirically

// --- Motor ceiling ---
#define MAX_PWM_CEILING    255    // absolute cap regardless of computed speed
#define MIN_MOVING_PWM     60     // below this, motors may not overcome static friction

// --- Loss-of-target behavior (v1: simple stop, no frame-exit logic yet) ---
#define LOST_TARGET_STOP   1

// --- Send timing ---
#define SEND_INTERVAL_MS   0      // 0 = send every loop iteration (camera-rate limited anyway)

#endif // CONSTANTS_H
