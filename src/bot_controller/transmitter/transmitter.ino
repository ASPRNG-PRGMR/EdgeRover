// controller.ino
//
// Transmitter for a custom ESP-NOW RC link.
// Reads local inputs (inputs.*): a potentiometer for the overall speed
// ceiling and a rotary encoder for steering, mixes them into per-side
// PWM values, shows status on an ST7789 (display.*), and sends the
// result to the vehicle (espnow_tx.*) at ~50 Hz.

#include "inputs.h"
#include "espnow_tx.h"
#include "packet.h"
#include "display.h"

static const uint32_t SEND_INTERVAL_MS = 20; // 50 Hz

static uint32_t lastSendTime = 0;
static uint32_t packetCounter = 0;

void setup()
{
    Serial.begin(115200);

    inputs_init();
    display_init();

    if (!espnow_tx_init())
    {
        Serial.println("ESP-NOW init failed. Halting.");
        while (true) { delay(1000); }
    }

    Serial.println("Controller ready.");
}

void loop()
{
    uint32_t now = millis();

    if (now - lastSendTime >= SEND_INTERVAL_MS)
    {
        lastSendTime = now;

        InputState input;
        inputs_read(input);

        ControlPacket packet;
        packet.version = PACKET_VERSION;
        packet.leftPWM = input.leftPWM;
        packet.rightPWM = input.rightPWM;
        packet.buttons = input.buttons;
        packet.mode = input.mode;
        packet.armed = input.armed;
        packet.packetCounter = packetCounter++;

        espnow_tx_send(packet);

        uint8_t speedPercent = (uint8_t)(((uint16_t)input.speedPWM * 100) / 255);
        int8_t steerPercent  = (int8_t)(((int32_t)input.steerSteps * 100) / STEER_MAX_STEPS);
        display_update(input.leftPWM, input.rightPWM, speedPercent, steerPercent, input.armed != 0);

        // Uncomment for debugging - sending Serial output at 50Hz can
        // itself introduce jitter, so keep it off in normal operation.
        // Serial.printf("TX #%lu L=%u R=%u speed=%u%% steer=%d%% armed=%u ok=%d\n",
        //               packet.packetCounter, packet.leftPWM, packet.rightPWM,
        //               speedPercent, steerPercent, packet.armed,
        //               espnow_tx_last_send_ok());
    }
}
