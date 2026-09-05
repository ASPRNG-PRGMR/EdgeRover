#include "tilt_servo.h"
#include "constants.h"
#include <Arduino.h>
#include <math.h>
#include "driver/ledc.h"

// ---------------------------------------------------------------------
// Why the IDF ledc driver instead of Arduino's ledcAttach():
// esp32-camera generates the OV3660's 20 MHz XCLK on LEDC timer 0 /
// channel 0 (camera.cpp). Arduino core 3.x's ledcAttach() hands out
// timers from its own bookkeeping only — it has no idea the camera
// owns timer 0 — so it can happily reprogram that timer to 50 Hz and
// the camera stops dead. Pinning the servo to timer 1 / channel 2
// through the IDF API makes the assignment explicit and permanent.
// ---------------------------------------------------------------------
#define SERVO_LEDC_TIMER     LEDC_TIMER_1
#define SERVO_LEDC_CHANNEL   LEDC_CHANNEL_2
#define SERVO_LEDC_MODE      LEDC_LOW_SPEED_MODE   // the S3 only has low-speed mode
#define SERVO_LEDC_RES_BITS  14                    // 16384 steps over 20 ms → 1.22 µs/step
#define SERVO_PERIOD_US      20000

static float    tiltTarget  = 0.0f;
static float    tiltCurrent = 0.0f;
static uint32_t lastUpdateMs = 0;
static bool     ready = false;

static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static void write_tilt(float tilt_deg)
{
    float servoDeg = SERVO_LEVEL_DEG + SERVO_UP_SIGN * tilt_deg;
    servoDeg = clampf(servoDeg, 0.0f, 180.0f);

    float pulseUs = SERVO_MIN_US + (servoDeg / 180.0f) * (float)(SERVO_MAX_US - SERVO_MIN_US);
    uint32_t duty = (uint32_t)((pulseUs * (float)(1u << SERVO_LEDC_RES_BITS)) / (float)SERVO_PERIOD_US);

    ledc_set_duty(SERVO_LEDC_MODE, SERVO_LEDC_CHANNEL, duty);
    ledc_update_duty(SERVO_LEDC_MODE, SERVO_LEDC_CHANNEL);
}

float tilt_servo_search_tilt()
{
    float rise = TAG_HEIGHT_CM - CAMERA_HEIGHT_CM;
    float tilt = atan2f(rise, TILT_SEARCH_RANGE_CM) * 180.0f / (float)M_PI;
    return clampf(tilt, TILT_MIN_DEG, TILT_MAX_DEG);
}

bool tilt_servo_init()
{
    ledc_timer_config_t timerCfg = {};
    timerCfg.speed_mode      = SERVO_LEDC_MODE;
    timerCfg.duty_resolution = (ledc_timer_bit_t)SERVO_LEDC_RES_BITS;
    timerCfg.timer_num       = SERVO_LEDC_TIMER;
    timerCfg.freq_hz         = 50;
    timerCfg.clk_cfg         = LEDC_AUTO_CLK;
    if (ledc_timer_config(&timerCfg) != ESP_OK)
    {
        Serial.println("[servo] ledc_timer_config failed");
        return false;
    }

    ledc_channel_config_t chCfg = {};
    chCfg.gpio_num   = SERVO_PIN;
    chCfg.speed_mode = SERVO_LEDC_MODE;
    chCfg.channel    = SERVO_LEDC_CHANNEL;
    chCfg.intr_type  = LEDC_INTR_DISABLE;
    chCfg.timer_sel  = SERVO_LEDC_TIMER;
    chCfg.duty       = 0;
    chCfg.hpoint     = 0;
    if (ledc_channel_config(&chCfg) != ESP_OK)
    {
        Serial.println("[servo] ledc_channel_config failed");
        return false;
    }

    // Start at the search pose so the very first frames already look
    // roughly where a person standing ~1 m away would carry the tag.
    tiltCurrent = tilt_servo_search_tilt();
    tiltTarget  = tiltCurrent;
    write_tilt(tiltCurrent);
    lastUpdateMs = millis();
    ready = true;
    return true;
}

void tilt_servo_set_target(float tilt_deg)
{
    tiltTarget = clampf(tilt_deg, TILT_MIN_DEG, TILT_MAX_DEG);
}

void tilt_servo_update()
{
    if (!ready) return;

    uint32_t now = millis();
    float dt = (float)(now - lastUpdateMs) / 1000.0f;
    lastUpdateMs = now;
    if (dt > 0.25f) dt = 0.25f;   // first call / long stall: don't jump

    float maxStep = TILT_RATE_DEG_PER_S * dt;
    float delta   = tiltTarget - tiltCurrent;
    if (delta >  maxStep) delta =  maxStep;
    if (delta < -maxStep) delta = -maxStep;
    tiltCurrent += delta;

    write_tilt(tiltCurrent);
}

float tilt_servo_get_tilt()
{
    return tiltCurrent;
}
