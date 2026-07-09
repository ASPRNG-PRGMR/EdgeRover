#include "espnow_tx.h"
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

// Fill this in with your receiver's actual MAC address.
uint8_t RECEIVER_MAC[6] = {0x00, 0x70, 0x07, 0xE3, 0x17, 0xB8};

static volatile bool lastSendOk = false;

// ESP-NOW send callback - fires asynchronously after esp_now_send().
// Kept tiny on purpose; no business logic belongs here.
// NOTE: ESP32 Arduino core 3.x (IDF 5.x) changed this signature from
// (const uint8_t *mac_addr, ...) to (const wifi_tx_info_t *tx_info, ...).
// If you're on an older core and this fails to compile, revert the
// parameter type back to `const uint8_t *mac_addr`.
static void onDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status)
{
    lastSendOk = (status == ESP_NOW_SEND_SUCCESS);
}

bool espnow_tx_init()
{
    WiFi.mode(WIFI_STA);
    // Disconnect from anything Wi-Fi auto-tried to join, and don't
    // sleep the radio - ESP-NOW timing degrades badly with modem sleep.
    WiFi.disconnect();
    esp_wifi_set_ps(WIFI_PS_NONE);

    if (esp_now_init() != ESP_OK)
    {
        return false;
    }

    esp_now_register_send_cb(onDataSent);

    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, RECEIVER_MAC, 6);
    peerInfo.channel = 0;       // use current Wi-Fi channel
    peerInfo.encrypt = false;   // add encryption later if needed

    if (esp_now_add_peer(&peerInfo) != ESP_OK)
    {
        return false;
    }

    return true;
}

bool espnow_tx_send(const ControlPacket &packet)
{
    esp_err_t result = esp_now_send(RECEIVER_MAC,
                                     (const uint8_t *)&packet,
                                     sizeof(packet));
    return result == ESP_OK;
}

bool espnow_tx_last_send_ok()
{
    return lastSendOk;
}