// receiver.ino
//
// Vehicle-side receiver for a custom ESP-NOW RC link.
// Receives ControlPackets (espnow_rx.*), and drives the ESC/servo
// (outputs.*). Owns the failsafe timing decision itself, since that's
// link-health logic, not strictly "networking" or "output" logic.

#include "espnow_rx.h"
#include "outputs.h"
#include "packet.h"
#include <WiFi.h>
#include <esp_wifi.h>

static const uint32_t FAILSAFE_TIMEOUT_MS = 200;

static bool inFailsafe = true; // start in failsafe until we hear from the controller

void setup()
{
    Serial.begin(115200);

    outputs_init(); // safe state immediately, before radio is even up

    if (!espnow_rx_init())
    {
        Serial.println("ESP-NOW init failed. Halting in failsafe state.");
        while (true)
        {
            outputs_failsafe();
            delay(50);
        }
    }

    Serial.print("Receiver ready. MAC: ");
    uint8_t mac[6];
    esp_wifi_get_mac(WIFI_IF_STA, mac);
    char macStr[18];
    snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    Serial.println(macStr);
}

void loop()
{
    uint32_t age = espnow_rx_time_since_last_packet();

    if (age > FAILSAFE_TIMEOUT_MS)
    {
        if (!inFailsafe)
        {
            Serial.println("Link lost - entering failsafe.");
            inFailsafe = true;
        }
        outputs_failsafe();
    }
    else
    {
        if (inFailsafe)
        {
            Serial.println("Link restored - exiting failsafe.");
            inFailsafe = false;
        }

        ControlPacket packet;
        if (espnow_rx_get_latest(packet))
        {
            outputs_update(packet); // logs its own "armed=.. L=.. R=.." line
        }
        else
        {
            // Shouldn't happen (age check implies havePacket), but
            // fail safe rather than fail open if it ever does.
            outputs_failsafe();
        }
    }

    // Small delay to avoid pegging the CPU; outputs only need to update
    // as fast as packets arrive (~50Hz), and failsafe checking is cheap.
    delay(5);
}

