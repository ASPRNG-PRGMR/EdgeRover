#include "espnow_rx.h"
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

static volatile bool havePacket = false;
static volatile uint32_t lastReceiveMillis = 0;

// Double-buffered-ish via a simple mutex-free copy: the receive callback
// runs in a Wi-Fi driver task, not the Arduino loop() task, so we keep
// this critical section as short as possible (a single struct copy).
static ControlPacket latestPacket;

// NOTE: ESP32 Arduino core 3.x (IDF 5.x) changed this signature from
// (const uint8_t *mac_addr, ...) to (const esp_now_recv_info_t *recv_info, ...).
// If you're on an older core and this fails to compile, revert the
// parameter type back to `const uint8_t *mac_addr` and drop recv_info.
static void onDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{
    if (len != sizeof(ControlPacket))
    {
        // Wrong size packet - ignore it rather than risk reading garbage
        // into a differently-shaped struct.
        return;
    }

    memcpy((void *)&latestPacket, data, sizeof(ControlPacket));
    lastReceiveMillis = millis();
    havePacket = true;
}

bool espnow_rx_init()
{
    WiFi.mode(WIFI_STA);
    delay(100);   // let the Wi-Fi driver actually come up before anything
                  // (e.g. WiFi.macAddress() in receiver.ino) reads from it -
                  // reading too early returns 00:00:00:00:00:00 on some cores
    WiFi.disconnect();
    esp_wifi_set_ps(WIFI_PS_NONE);

    if (esp_now_init() != ESP_OK)
    {
        return false;
    }

    esp_now_register_recv_cb(onDataRecv);

    return true;
}

bool espnow_rx_get_latest(ControlPacket &outPacket)
{
    if (!havePacket)
    {
        return false;
    }

    memcpy(&outPacket, (const void *)&latestPacket, sizeof(ControlPacket));
    return true;
}

uint32_t espnow_rx_time_since_last_packet()
{
    if (!havePacket)
    {
        return UINT32_MAX;
    }

    return millis() - lastReceiveMillis;
}