#ifndef INPUTS_H
#define INPUTS_H

#include <stdint.h>

// ---- Pin configuration ----------------------------------------------

// Speed: 10k potentiometer wiper. Sets the overall PWM ceiling shared
// by both motors - this is what caps startup current instead of
// always commanding full duty.
// GPIO32 is an ADC1 pin, so it reads reliably even with Wi-Fi/ESP-NOW
// active (unlike ADC2 pins, which are unusable while Wi-Fi is on).
#define PIN_SPEED_POT       32

// Steering: rotary encoder (CLK/DT quadrature) + its built-in
// pushbutton, which now doubles as arm/disarm - there's no separate
// arm switch anymore.
// IMPORTANT: DT must be on a pin that supports an internal pull-up.
// GPIO34-39 are input-only on the ESP32 and have NO internal pull
// resistors at all - pinMode(INPUT_PULLUP) silently does nothing on
// those pins, leaving the input floating. That floating read is what
// was causing the encoder to register the wrong direction and
// occasionally run away to the clamp: DT was previously on GPIO34.
// GPIO33 is a normal ADC1-capable pin with full pull-up support, so
// DT lives there now.
#define PIN_ENCODER_SW      13   // active LOW (pressed = LOW)
#define PIN_ENCODER_DT      33
#define PIN_ENCODER_CLK     27

// Existing buttons/mode switch.
// PIN_BUTTON_C was previously GPIO27 - the SAME pin as
// PIN_ENCODER_CLK above. Since CLK has an interrupt attached on the
// FALLING edge, every press of button C was indistinguishable from a
// real encoder pulse and silently corrupted the steering count. Moved
// to GPIO26 (free, digital-only use so the ADC2/Wi-Fi caveat doesn't
// apply).
#define PIN_BUTTON_A        25
#define PIN_BUTTON_B        16
#define PIN_BUTTON_C        26
#define PIN_BUTTON_D        17

// Was GPIO32 - the SAME pin as PIN_SPEED_POT above. Flipping the mode
// switch was electrically yanking the line the pot's analogRead()
// depends on. Moved to GPIO4 (free, plain digital input).
#define PIN_MODE_SWITCH     4

// ---- Speed pot scaling -------------------------------------------------
#define SPEED_PWM_MAX         255   // matches receiver's 8-bit LEDC duty
#define AXIS_EXTREME_LOW      130   // pot raw <= this snaps to 0
#define AXIS_EXTREME_HIGH     5000  // pot raw >= this snaps to full scale

// Deadband on the raw ADC reading (0-4095 scale) before it's mapped
// to a PWM/percent value. Pot wipers + ESP32 ADC noise are good for
// maybe +/-1% of jitter at rest, which was enough to flip the mapped
// percent back and forth (47<->46, 45<->44...) every read, spamming
// the receiver with duty changes that never reflected an actual knob
// movement. 40 counts is a little under 1% of the 4095 range - big
// enough to swallow that jitter, small enough not to feel laggy.
#define SPEED_ADC_HYSTERESIS  80

// ---- Steering (encoder) scaling -----------------------------------------
// Number of encoder detents from center to "full lock" (inner wheel
// driven all the way to 0). Raise for a gentler, longer-throw feel.
#define STEER_MAX_STEPS       20
// Flip to 1 if turning the knob steers the wrong way for your wiring.
#define ENCODER_INVERTED      0

// Short settle delay (microseconds) used inside the encoder ISR to
// confirm a CLK falling edge is real and that DT has finished
// transitioning before it's sampled. Mechanical encoders don't
// switch CLK and DT at exactly the same instant, so reading DT the
// moment CLK's interrupt fires can occasionally catch it mid-move
// and register the wrong direction. This is separate from
// ENCODER_DEBOUNCE_US, which rejects repeated edges too close
// together in time - this instead re-checks a single edge shortly
// after it fires. Keep this small; it blocks inside an ISR.
#define ENCODER_SETTLE_US     20

// ---- Public data produced by this module -----------------------------
struct InputState
{
    uint8_t  leftPWM;      // 0-255, final left motor duty (forward only)
    uint8_t  rightPWM;     // 0-255, final right motor duty (forward only)
    uint8_t  speedPWM;     // 0-255, raw pot ceiling before steering is applied - for the display
    int8_t   steerSteps;   // clamped +/-STEER_MAX_STEPS - for the display
    uint8_t  buttons;      // bitmask, see packet.h BTN_* defines
    uint8_t  mode;         // current mode switch position
    uint8_t  armed;        // 0 = disarmed, 1 = armed
};

// Call once in setup(). Sets pin modes and attaches the encoder ISR.
void inputs_init();

// Call every loop iteration. Reads local hardware (pot, encoder,
// buttons, mode switch, arm button) and fills out 'state', including
// the already-computed leftPWM/rightPWM the receiver should drive.
void inputs_read(InputState &state);

#endif // INPUTS_H