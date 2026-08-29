#ifndef TRACKER_CONTROL_H
#define TRACKER_CONTROL_H

#include <stdint.h>
#include "tag_detector.h"

// Final drive command in the same shape the receiver expects:
// forward-only PWM per side (see packet.h — leftPWM/rightPWM are
// 0-255, forward only, no reverse in this packet version).
struct DriveCommand
{
    uint8_t leftPWM;
    uint8_t rightPWM;
};

// Call once in setup() to zero internal state (still-detection history,
// previous error for the D term, etc.).
void tracker_control_init();

// Call once per loop iteration, whether or not a tag was found this
// frame — internal history/state needs the "not found" frames too.
// Returns the PWM command to send this cycle.
DriveCommand tracker_control_update(const TagDetection &det);

#endif // TRACKER_CONTROL_H
