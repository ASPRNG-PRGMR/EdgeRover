#include "tracker_control.h"
#include "constants.h"
#include <math.h>
#include <Arduino.h>

static TrackerState state = TRACKER_IDLE;

static bool     everSeen        = false;
static uint32_t lastSeenMs      = 0;
static float    lastBearingDeg  = 0.0f;
static DriveCommand lastCommand = {0, 0};

static float    prevBearingDeg  = 0.0f;
static uint32_t prevUpdateMs    = 0;
static bool     havePrevBearing = false;

static bool     holding         = false;   // hysteresis latch around MIN/RESUME distance

void tracker_control_init()
{
    state = TRACKER_IDLE;
    everSeen = false;
    lastSeenMs = 0;
    lastBearingDeg = 0.0f;
    lastCommand = {0, 0};
    prevBearingDeg = 0.0f;
    prevUpdateMs = millis();
    havePrevBearing = false;
    holding = false;
}

static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static float mapf(float x, float inMin, float inMax, float outMin, float outMax)
{
    return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

static int16_t clamp_pwm(float v)
{
    return (int16_t)clampf(v, -255.0f, 255.0f);
}

// Same mixing as bot_controller/transmitter/inputs.cpp computeDrive():
// outer wheel at speedPWM, inner wheel from +1x (centred) through 0
// (half lock) to -1x (full lock). turn > 0 = steer right.
static DriveCommand mix_drive(int16_t speedPWM, float turn)
{
    DriveCommand cmd;
    float turnFactor = fabsf(turn);
    float innerFactor = 1.0f - 2.0f * turnFactor;
    int16_t inner = (int16_t)((float)speedPWM * innerFactor);

    if (turn >= 0.0f) { cmd.leftPWM = speedPWM; cmd.rightPWM = inner; }
    else              { cmd.leftPWM = inner;    cmd.rightPWM = speedPWM; }
    return cmd;
}

static DriveCommand pivot(float turn, int16_t pwmMin, int16_t pwmMax)
{
    DriveCommand cmd;
    float turnFactor = clampf(fabsf(turn), 0.0f, 1.0f);
    int16_t pivotPWM = (int16_t)(pwmMin + (pwmMax - pwmMin) * turnFactor);
    if (turn > 0.0f) { cmd.leftPWM =  pivotPWM; cmd.rightPWM = -pivotPWM; }  // right
    else             { cmd.leftPWM = -pivotPWM; cmd.rightPWM =  pivotPWM; }  // left
    return cmd;
}

static DriveCommand handle_lost(uint32_t now)
{
    uint32_t lostFor = now - lastSeenMs;
    havePrevBearing = false;           // don't let the D term spike on re-acquire

    if (!everSeen)
    {
        state = TRACKER_IDLE;
        return {0, 0};
    }

    if (lostFor < LOST_HOLD_MS)
    {
        state = TRACKER_LOST_HOLD;
        return lastCommand;
    }

    if (lostFor < (uint32_t)LOST_HOLD_MS + (uint32_t)LOST_PIVOT_MS &&
        fabsf(lastBearingDeg) > STEER_DEADZONE_DEG)
    {
        // The tag most likely walked out of the side of the frame it was
        // last seen on — turn that way to bring it back.
        state = TRACKER_LOST_PIVOT;
        float dir = (lastBearingDeg > 0.0f) ? 1.0f : -1.0f;
        return pivot(dir, LOST_PIVOT_PWM, LOST_PIVOT_PWM);
    }

    state = TRACKER_LOST_STOP;
    holding = false;
    range_model_reset();
    return {0, 0};
}

DriveCommand tracker_control_update(const TagDetection &det, const RangeEstimate &range)
{
    uint32_t now = millis();

    if (!det.found || !range.valid)
    {
        DriveCommand cmd = handle_lost(now);
        if (state != TRACKER_LOST_HOLD) lastCommand = cmd;
        return cmd;
    }

    everSeen   = true;
    lastSeenMs = now;
    lastBearingDeg = range.bearing_deg;

    // --- Steering: P(D) on the rover-frame bearing -------------------------
    float dt = havePrevBearing ? (float)(now - prevUpdateMs) / 1000.0f : 0.0f;
    if (dt <= 0.0f) dt = 0.001f;

    float bearing = range.bearing_deg;
    float turn = STEER_KP * bearing;
    if (havePrevBearing)
    {
        turn += STEER_KD * (bearing - prevBearingDeg) / dt;
    }
    if (fabsf(bearing) < STEER_DEADZONE_DEG) turn = 0.0f;
    turn = clampf(turn, -1.0f, 1.0f);

    prevBearingDeg  = bearing;
    prevUpdateMs    = now;
    havePrevBearing = true;

    // --- Distance → speed, with hysteresis around the stop point ----------
    float ground = range.ground_filtered_cm;

    if (ground <= MIN_FOLLOW_DISTANCE_CM)      holding = true;
    else if (ground >= RESUME_DISTANCE_CM)     holding = false;

    // --- Back-off: person stepped into us ---------------------------------
    if (BACKOFF_DISTANCE_CM > 0.0f && ground < BACKOFF_DISTANCE_CM)
    {
        state = TRACKER_BACKOFF;
        lastCommand = { (int16_t)-BACKOFF_PWM, (int16_t)-BACKOFF_PWM };
        return lastCommand;
    }

    float ceiling = (float)MAX_PWM_CEILING;
#if !FOCAL_IS_CALIBRATED
    ceiling = fminf(ceiling, (float)UNCALIBRATED_PWM_CAP);
#endif
    float minMovingFraction = (float)MIN_MOVING_PWM / ceiling;

    float speedFraction;
    if (holding)
    {
        speedFraction = 0.0f;
    }
    else if (ground < SLOW_DISTANCE_CM)
    {
        speedFraction = mapf(ground, MIN_FOLLOW_DISTANCE_CM, SLOW_DISTANCE_CM,
                             minMovingFraction, 1.0f);
    }
    else
    {
        speedFraction = 1.0f;
    }
    int16_t speedPWM = clamp_pwm(clampf(speedFraction, 0.0f, 1.0f) * ceiling);

    // --- Mix ---------------------------------------------------------------
    DriveCommand cmd;
    if (speedPWM == 0)
    {
        state = TRACKER_HOLD;
        // Stopped at the person's heels but they drifted sideways: pivot to
        // keep them centred so the next step forward is straight.
        cmd = (turn != 0.0f) ? pivot(turn, PIVOT_PWM_MIN, PIVOT_PWM_MAX) : DriveCommand{0, 0};
    }
    else
    {
        state = TRACKER_FOLLOW;
        cmd = mix_drive(speedPWM, turn);
    }

    lastCommand = cmd;
    return cmd;
}

TrackerState tracker_control_state()
{
    return state;
}

const char *tracker_control_state_name()
{
    switch (state)
    {
        case TRACKER_IDLE:       return "IDLE";
        case TRACKER_FOLLOW:     return "FOLLOW";
        case TRACKER_HOLD:       return "HOLD";
        case TRACKER_BACKOFF:    return "BACKOFF";
        case TRACKER_LOST_HOLD:  return "LOST_HOLD";
        case TRACKER_LOST_PIVOT: return "LOST_PIVOT";
        case TRACKER_LOST_STOP:  return "LOST_STOP";
    }
    return "?";
}

uint32_t tracker_control_lost_for_ms()
{
    if (!everSeen) return UINT32_MAX;
    uint32_t age = millis() - lastSeenMs;
    return age;
}
