#ifndef OUTPUTS_H
#define OUTPUTS_H

#include "packet.h"

// ---- Pin configuration -----------------------------------------------
// TB6612FNG direction + standby + PWM pins. STBY is driven from
// packet.armed: HIGH = driver enabled, LOW = driver fully disabled
// (hardware-level stop, not just direction/PWM at 0).
#define PIN_AIN1   16   // Left motor, direction pin 1
#define PIN_AIN2   17   // Left motor, direction pin 2
#define PIN_BIN1   18   // Right motor, direction pin 1
#define PIN_BIN2   19   // Right motor, direction pin 2
#define PIN_STBY   21   // TB6612FNG STBY - HIGH to enable, tied to armed/failsafe

#define PIN_PWMA   26   // Left motor speed (LEDC PWM)
#define PIN_PWMB   27   // Right motor speed (LEDC PWM)

// ---- LEDC (ESP32 hardware PWM) configuration ---------------------------
#define PWM_FREQ_HZ       20000   // above audible range
#define PWM_RESOLUTION    8       // 0-255, matches packet.leftPWM/rightPWM

// The transmitter (pot + rotary encoder) has no reverse input, so this
// is forward-only by design: AIN1/BIN1 are held HIGH (AIN2/BIN2 LOW)
// whenever armed, and PWMA/PWMB carry the actual speed. If you add a
// reverse control later, this is the file to extend.

// Call once in setup(). Sets pin modes, configures the LEDC PWM
// channels, and forces both motors to a stopped state immediately.
void outputs_init();

// Drives left/right motors directly from packet.leftPWM/rightPWM.
// Prints "armed=.. L=.. R=.." to Serial for debugging.
void outputs_update(const ControlPacket &packet);

// Forces both motors to a stopped state (0 duty, direction pins LOW,
// STBY LOW). Called continuously in failsafe and whenever disarmed.
void outputs_failsafe();

#endif // OUTPUTS_H
