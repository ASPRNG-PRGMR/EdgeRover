#include "tracker_control.h"
#include "constants.h"
#include <math.h>
#include <Arduino.h>   // millis()

static float   sizeHistory[STILL_HISTORY_LEN];
static int     sizeHistoryCount = 0;
static int     sizeHistoryIndex = 0;

static float   prevErrorNorm = 0.0f;
static uint32_t prevUpdateMs = 0;
static bool     havePrevError = false;

void tracker_control_init()
{
    sizeHistoryCount = 0;
    sizeHistoryIndex = 0;
    prevErrorNorm = 0.0f;
    prevUpdateMs = millis();
    havePrevError = false;
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

// Pushes a new apparent-size sample and returns the variance of the
// last STILL_HISTORY_LEN samples. Returns a large value (never "still")
// until the history buffer is actually full, so we don't falsely
// declare "target stopped" on the first couple of frames.
static float push_size_and_get_variance(float size_px)
{
    sizeHistory[sizeHistoryIndex] = size_px;
    sizeHistoryIndex = (sizeHistoryIndex + 1) % STILL_HISTORY_LEN;
    if (sizeHistoryCount < STILL_HISTORY_LEN)
    {
        sizeHistoryCount++;
        return 1e9f;  // not enough samples yet
    }

    float mean = 0.0f;
    for (int i = 0; i < STILL_HISTORY_LEN; i++) mean += sizeHistory[i];
    mean /= STILL_HISTORY_LEN;

    float variance = 0.0f;
    for (int i = 0; i < STILL_HISTORY_LEN; i++)
    {
        float d = sizeHistory[i] - mean;
        variance += d * d;
    }
    variance /= STILL_HISTORY_LEN;
    return variance;
}

DriveCommand tracker_control_update(const TagDetection &det)
{
    DriveCommand cmd = {0, 0};

    if (!det.found)
    {
        // v1 loss-of-target behavior: just stop. Frame-exit-direction
        // logic (roadmap Phase 5) replaces this later.
        havePrevError = false;   // don't let D-term spike on reacquire
        return cmd;
    }

    uint32_t nowMs = millis();
    float dt = havePrevError ? (float)(nowMs - prevUpdateMs) / 1000.0f : 0.0f;
    if (dt <= 0.0f) dt = 0.001f;  // guard against div-by-zero on first frame

    // --- Steering error (normalized, -1..+1) ---
    float frameCenterX = FRAME_WIDTH / 2.0f;
    float errorNorm = (det.cx - frameCenterX) / frameCenterX;

    float turn = STEER_KP * errorNorm;
    if (havePrevError)
    {
        turn += STEER_KD * (errorNorm - prevErrorNorm) / dt;
    }
    turn = clampf(turn, -1.0f, 1.0f);

    prevErrorNorm = errorNorm;
    prevUpdateMs = nowMs;
    havePrevError = true;

    // --- Distance estimate (pinhole model) ---
    // Guard against div-by-zero / garbage before calibration constants
    // are actually filled in (constants.h defaults FOCAL_LENGTH_PX to 0).
    float distance = 1e9f;
    if (det.size_px > 0.5f && FOCAL_LENGTH_PX > 0.0f && REAL_TAG_SIZE_CM > 0.0f)
    {
        distance = (REAL_TAG_SIZE_CM * FOCAL_LENGTH_PX) / det.size_px;
    }

    // --- Speed fraction from distance ---
    float minMovingFraction = (float)MIN_MOVING_PWM / (float)MAX_PWM_CEILING;
    float speedFraction;
    if (distance <= STOP_DISTANCE_CM)
    {
        speedFraction = 0.0f;
    }
    else if (distance <= SLOW_DISTANCE_CM)
    {
        speedFraction = mapf(distance, STOP_DISTANCE_CM, SLOW_DISTANCE_CM,
                              minMovingFraction, 1.0f);
    }
    else
    {
        speedFraction = 1.0f;
    }

    // --- "Target has stopped moving" override ---
    float variance = push_size_and_get_variance(det.size_px);
    if (variance < STILL_VARIANCE_THRESHOLD && distance > STOP_DISTANCE_CM)
    {
        speedFraction = 0.0f;
    }

    float ceiling = speedFraction * MAX_PWM_CEILING;

    // --- Mix into forward-only left/right PWM ---
    // Same convention as the transmitter: outer wheel holds the
    // ceiling, inner wheel scales down toward 0 with turn magnitude.
    // Deadzone keeps small noise in cx from causing constant micro-turns.
    float leftFraction = 1.0f;
    float rightFraction = 1.0f;

    if (turn > STEER_DEADZONE)
    {
        // tag right of center -> steer right -> right wheel is inner
        rightFraction = 1.0f - fabsf(turn);
    }
    else if (turn < -STEER_DEADZONE)
    {
        // tag left of center -> steer left -> left wheel is inner
        leftFraction = 1.0f - fabsf(turn);
    }

    cmd.leftPWM  = (uint8_t)clampf(ceiling * leftFraction, 0.0f, (float)MAX_PWM_CEILING);
    cmd.rightPWM = (uint8_t)clampf(ceiling * rightFraction, 0.0f, (float)MAX_PWM_CEILING);

    return cmd;
}
