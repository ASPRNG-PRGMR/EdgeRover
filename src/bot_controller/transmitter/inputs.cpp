#include "inputs.h"
#include <Arduino.h>

// ---- Encoder state (ISR-driven quadrature decode) ------------------
// Full quadrature state-machine decoder (the well-known "Buxton"
// rotary algorithm), replacing an earlier timing-based debounce.
// Timing debounce (reject edges too close together, then re-check
// levels a few us later) still let through short, individually
// "clean" glitches from vibration or contact chatter - each one
// looked valid on its own, so the display would hunt by +/-1 step
// with the knob just sitting still.
//
// This approach doesn't judge edges individually at all. CLK+DT are
// tracked as one 2-bit state, and a step is only ever emitted when
// the state machine walks through a *complete* detent-to-detent
// sequence and lands back at rest (R_START). Any glitch that doesn't
// walk the full sequence - a single spurious transition, or bounce
// that reverses partway through - just falls back to R_START and
// produces nothing, with no timing guesswork involved.
static volatile int32_t encoderSteps = 0;

#define ENC_R_START     0x0
#define ENC_R_CW_FINAL  0x1
#define ENC_R_CW_BEGIN  0x2
#define ENC_R_CW_NEXT   0x3
#define ENC_R_CCW_BEGIN 0x4
#define ENC_R_CCW_FINAL 0x5
#define ENC_R_CCW_NEXT  0x6
#define ENC_DIR_NONE    0x00
#define ENC_DIR_CW      0x10
#define ENC_DIR_CCW     0x20

// Row = current state, column = (CLK<<1 | DT). Value's low nibble is
// the next state; DIR_CW/DIR_CCW in the high nibble fires only on the
// single transition that completes a full detent cycle.
static const uint8_t ENC_TABLE[7][4] = {
    /* R_START     */ { ENC_R_START,    ENC_R_CW_BEGIN,  ENC_R_CCW_BEGIN, ENC_R_START },
    /* R_CW_FINAL  */ { ENC_R_CW_NEXT,  ENC_R_START,     ENC_R_CW_FINAL,  ENC_R_START | ENC_DIR_CW },
    /* R_CW_BEGIN  */ { ENC_R_CW_NEXT,  ENC_R_CW_BEGIN,  ENC_R_START,     ENC_R_START },
    /* R_CW_NEXT   */ { ENC_R_CW_NEXT,  ENC_R_CW_BEGIN,  ENC_R_CW_FINAL,  ENC_R_START },
    /* R_CCW_BEGIN */ { ENC_R_CCW_NEXT, ENC_R_START,     ENC_R_CCW_BEGIN, ENC_R_START },
    /* R_CCW_FINAL */ { ENC_R_CCW_NEXT, ENC_R_CCW_FINAL, ENC_R_START,     ENC_R_START | ENC_DIR_CCW },
    /* R_CCW_NEXT  */ { ENC_R_CCW_NEXT, ENC_R_CCW_FINAL, ENC_R_CCW_BEGIN, ENC_R_START },
};

static volatile uint8_t encoderState = ENC_R_START;

// Attached to CHANGE on both CLK and DT, so every edge on either pin
// runs this - the table above is what actually filters noise, not
// which pin triggered.
static void IRAM_ATTR onEncoderChange()
{
    uint8_t pins = (digitalRead(PIN_ENCODER_CLK) << 1) | digitalRead(PIN_ENCODER_DT);
    encoderState = ENC_TABLE[encoderState & 0x0F][pins];

    uint8_t dir = encoderState & 0x30;
    if (dir == ENC_DIR_NONE) return;

    int32_t delta = (dir == ENC_DIR_CW) ? 1 : -1;
    if (ENCODER_INVERTED) delta = -delta;

    encoderSteps += delta;
    if (encoderSteps > STEER_MAX_STEPS)  encoderSteps = STEER_MAX_STEPS;
    if (encoderSteps < -STEER_MAX_STEPS) encoderSteps = -STEER_MAX_STEPS;
}

// ---- Arm/disarm latch (encoder pushbutton) ------------------------------
// Same debounced press-to-toggle pattern the old dedicated arm switch
// used: press once to arm, press again to disarm - so you're not stuck
// holding a button down.
#define ARM_DEBOUNCE_MS 30
static bool armedLatched = false;
static int  lastArmReading = HIGH;
static uint32_t lastArmChangeMs = 0;

static void updateArmToggle()
{
    int reading = digitalRead(PIN_ENCODER_SW);
    uint32_t now = millis();

    if (reading != lastArmReading && (now - lastArmChangeMs) > ARM_DEBOUNCE_MS)
    {
        lastArmChangeMs = now;
        lastArmReading = reading;

        if (reading == LOW) // press edge, active-low with pullup
        {
            armedLatched = !armedLatched;
        }
    }
}

// ---- Helpers ------------------------------------------------------------

static uint16_t lastRawSpeed = 0; // last raw ADC value that cleared the deadband

static uint8_t readSpeedPWM()
{
    uint16_t raw = analogRead(PIN_SPEED_POT);

    if (raw <= AXIS_EXTREME_LOW)  raw = 0;
    if (raw >= AXIS_EXTREME_HIGH) raw = 4095;

    // Ignore small jitter around the last accepted reading so the
    // mapped PWM/percent value doesn't flicker between two adjacent
    // values every read when the knob is actually holding still.
    int32_t diff = (int32_t)raw - (int32_t)lastRawSpeed;
    if (diff > -SPEED_ADC_HYSTERESIS && diff < SPEED_ADC_HYSTERESIS)
    {
        raw = lastRawSpeed;
    }
    else
    {
        lastRawSpeed = raw;
    }

    return (uint8_t)map(raw, 0, 4095, 0, SPEED_PWM_MAX);
}

static uint8_t readButtons()
{
    uint8_t buttons = 0;

    // Buttons are wired with INPUT_PULLUP, so a pressed button reads LOW.
    if (digitalRead(PIN_BUTTON_A) == LOW) buttons |= (1 << 0);
    if (digitalRead(PIN_BUTTON_B) == LOW) buttons |= (1 << 1);
    if (digitalRead(PIN_BUTTON_C) == LOW) buttons |= (1 << 2);
    if (digitalRead(PIN_BUTTON_D) == LOW) buttons |= (1 << 3);

    return buttons;
}

// Minimum fraction of the speed ceiling the INNER wheel is allowed to
// drop to while steering. Without this floor, factor -> 0.0 at full
// steering lock, which means full lock literally zeroes that wheel's
// PWM - fine for a stationary pivot, but during a moving turn it means
// steering hard enough (even briefly, e.g. hitting a chicane) can kill
// drive to one side entirely, right when you need both sides pulling.
//
// With the floor, steering always tapers the inner wheel smoothly down
// to MIN_INNER_FACTOR and stops there - a true zero-speed pivot is no
// longer reachable just by cranking the encoder while rolling. If you
// want a dedicated in-place pivot for tight-course maneuvering later,
// that should be its own explicit input (e.g. a button held alongside
// full steer), not something reachable from the normal steering range.
//

// Turning now tapers the inner wheel THROUGH zero and into reverse as
// steering approaches full lock, instead of flooring at a fixed
// minimum. At center: inner = outer (straight). At half lock: inner = 0
// (pivots around that wheel). At full lock: inner = full reverse (true
// zero-radius turn). This replaces the old MIN_INNER_FACTOR floor.
//
// Separately: if the pot is at idle AND the encoder is deflected, that's
// not "driving with a tiny bit of throttle" - it's a request to pivot in
// place, using the existing controls with no new hardware. Both wheels
// spin at equal magnitude, opposite directions, scaled by steer angle.

#define THROTTLE_DEADBAND 8     // pot noise floor - below this counts as "idle"
#define STEER_DEADBAND    2     // encoder noise floor - below this counts as "centered"
#define PIVOT_PWM_MAX     180   // cap for in-place pivot speed - tune on the bench

static void computeDrive(uint8_t speedPWM, int32_t steps, int16_t &leftPWM, int16_t &rightPWM)
{
    int32_t absSteps = abs(steps);
    float turnFactor = (float)absSteps / (float)STEER_MAX_STEPS;  // 0.0 (center) .. 1.0 (full lock)
    if (turnFactor > 1.0f) turnFactor = 1.0f;

    if (speedPWM <= THROTTLE_DEADBAND && absSteps > STEER_DEADBAND)
    {
        // Idle throttle + steering deflected -> pivot in place.
        int16_t pivotPWM = (int16_t)(PIVOT_PWM_MAX * turnFactor);

        if (steps > 0)   // pivot right (CW)
        {
            leftPWM  =  pivotPWM;
            rightPWM = -pivotPWM;
        }
        else              // pivot left (CCW)
        {
            leftPWM  = -pivotPWM;
            rightPWM =  pivotPWM;
        }
        return;
    }

    // Normal driving: outer wheel always runs at commanded throttle.
    // Inner wheel ranges from +1x (centered) through 0 (half lock) to
    // -1x (full lock) of that same throttle.
    float innerFactor = 1.0f - 2.0f * turnFactor;
    int16_t inner = (int16_t)((float)speedPWM * innerFactor);

    if (steps >= 0)   // turning right -> right wheel is inner
    {
        leftPWM  = (int16_t)speedPWM;
        rightPWM = inner;
    }
    else              // turning left -> left wheel is inner
    {
        leftPWM  = inner;
        rightPWM = (int16_t)speedPWM;
    }
}

// ---- Public API -----------------------------------------------------

void inputs_init()
{
    analogReadResolution(12); // make sure we're using the full 0-4095 range

    pinMode(PIN_BUTTON_A, INPUT_PULLUP);
    pinMode(PIN_BUTTON_B, INPUT_PULLUP);
    pinMode(PIN_BUTTON_C, INPUT_PULLUP);
    pinMode(PIN_BUTTON_D, INPUT_PULLUP);
    pinMode(PIN_MODE_SWITCH, INPUT_PULLUP);

    pinMode(PIN_ENCODER_SW, INPUT_PULLUP);
    pinMode(PIN_ENCODER_CLK, INPUT_PULLUP);
    pinMode(PIN_ENCODER_DT, INPUT_PULLUP);

    lastArmReading = digitalRead(PIN_ENCODER_SW);

    // Encoder ISR needs to see every edge on both lines to walk the
    // state table correctly - FALLING-only on CLK isn't enough here.
    attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_CLK), onEncoderChange, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_DT),  onEncoderChange, CHANGE);

    // PIN_SPEED_POT needs no pinMode call on ESP32 ADC pins.
}

void inputs_read(InputState &state)
{
    state.speedPWM = readSpeedPWM();

    int32_t steps = encoderSteps; // snapshot the volatile once
    state.steerSteps = (int8_t)steps;

    computeDrive(state.speedPWM, steps, state.leftPWM, state.rightPWM);

    state.buttons = readButtons();

    // Simple digital mode switch placeholder - expand to a multi-position
    // (e.g. resistor ladder + analogRead) switch later if needed.
    state.mode = (digitalRead(PIN_MODE_SWITCH) == LOW) ? 1 : 0;

    // Arm switch is now the encoder's built-in pushbutton, wired as a
    // toggle in firmware (see updateArmToggle()) - press once to arm,
    // press again to disarm.
    updateArmToggle();
    state.armed = armedLatched ? 1 : 0;
}
