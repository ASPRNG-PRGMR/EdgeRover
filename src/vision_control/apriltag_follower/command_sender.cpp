#include "command_sender.h"
#include "constants.h"
#include "espnow_tx.h"
#include "packet.h"
#include <Arduino.h>
#include <stdlib.h>

static portMUX_TYPE  cmdMux = portMUX_INITIALIZER_UNLOCKED;
static DriveCommand  latest = {0, 0};
static uint32_t      latestMs = 0;
static bool          haveCommand = false;
static bool          latestArmed = false;

static volatile int16_t  outLeft = 0, outRight = 0;
static volatile uint32_t packetsSent = 0;
static volatile bool     lastOk = false;

static int16_t slew(int16_t current, int16_t target)
{
    int32_t diff = (int32_t)target - (int32_t)current;
    if (diff == 0) return current;

    // "Down" = magnitude shrinking or sign flipping through zero; both
    // are the direction you want to be fast in.
    bool shrinking = (abs((int)target) < abs((int)current)) ||
                     ((target > 0) != (current > 0) && current != 0);
    int32_t step = shrinking ? PWM_SLEW_DOWN_PER_TICK : PWM_SLEW_UP_PER_TICK;

    if (diff >  step) diff =  step;
    if (diff < -step) diff = -step;
    return (int16_t)(current + diff);
}

static void sender_task(void *arg)
{
    (void)arg;
    TickType_t lastWake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(SEND_INTERVAL_MS);

    int16_t curL = 0, curR = 0;
    uint32_t counter = 0;

    for (;;)
    {
        vTaskDelayUntil(&lastWake, period);

        DriveCommand target;
        uint32_t ageMs;
        bool have, armed;
        portENTER_CRITICAL(&cmdMux);
        target = latest;
        ageMs  = millis() - latestMs;
        have   = haveCommand;
        armed  = latestArmed;
        portEXIT_CRITICAL(&cmdMux);

        if (!have || ageMs > COMMAND_STALE_MS)
        {
            target = {0, 0};
        }

        curL = slew(curL, target.leftPWM);
        curR = slew(curR, target.rightPWM);
        outLeft = curL;
        outRight = curR;

        ControlPacket packet = {};
        packet.version  = PACKET_VERSION;
        packet.leftPWM  = curL;
        packet.rightPWM = curR;
        packet.buttons  = 0;
        packet.mode     = VISION_MODE_ID;
        // Disarmed (driver in standby) until the tracker has seen a tag once;
        // after that PWM 0 means "commanded stop" and the receiver's own
        // 200 ms failsafe is the disarm path.
        packet.armed    = armed ? 1 : 0;
        packet.packetCounter = counter++;

        lastOk = espnow_tx_send(packet);
        packetsSent = counter;
    }
}

bool command_sender_init()
{
    latest = {0, 0};
    latestMs = millis();
    haveCommand = false;
    latestArmed = false;

    BaseType_t ok = xTaskCreatePinnedToCore(sender_task, "cmd_tx", 4096, nullptr,
                                            2 /* above loop() */, nullptr, 0 /* Wi-Fi core */);
    return ok == pdPASS;
}

void command_sender_set(const DriveCommand &cmd, bool armed)
{
    portENTER_CRITICAL(&cmdMux);
    latest = cmd;
    latestMs = millis();
    haveCommand = true;
    latestArmed = armed;
    portEXIT_CRITICAL(&cmdMux);
}

uint32_t command_sender_packets_sent()
{
    return packetsSent;
}

bool command_sender_last_send_ok()
{
    return lastOk;
}

void command_sender_get_output(int16_t &left, int16_t &right)
{
    left = outLeft;
    right = outRight;
}
