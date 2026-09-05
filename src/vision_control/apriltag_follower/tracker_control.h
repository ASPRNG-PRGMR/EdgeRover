#ifndef TRACKER_CONTROL_H
#define TRACKER_CONTROL_H

#include <stdint.h>
#include "tag_detector.h"
#include "range_model.h"

// Signed per-side PWM, same shape as ControlPacket v3: sign = direction,
// magnitude = duty (0-255). Positive = forward.
struct DriveCommand
{
    int16_t leftPWM;
    int16_t rightPWM;
};

enum TrackerState
{
    TRACKER_IDLE = 0,     // never seen a tag since boot — outputs stay at zero
    TRACKER_FOLLOW,       // tag in view, driving toward it
    TRACKER_HOLD,         // at / inside MIN_FOLLOW_DISTANCE_CM, wheels stopped (pivots allowed)
    TRACKER_BACKOFF,      // inside BACKOFF_DISTANCE_CM, reversing gently
    TRACKER_LOST_HOLD,    // tag just vanished, repeating the last command briefly
    TRACKER_LOST_PIVOT,   // pivoting toward the side the tag left on
    TRACKER_LOST_STOP     // gave up, stopped, camera at search tilt
};

void tracker_control_init();

// Call once per frame, found or not — the lost-target timers live here.
DriveCommand tracker_control_update(const TagDetection &det, const RangeEstimate &range);

TrackerState tracker_control_state();
const char  *tracker_control_state_name();

// Milliseconds since the tag was last seen (0 while it is in view).
uint32_t tracker_control_lost_for_ms();

#endif // TRACKER_CONTROL_H
