#ifndef ESPNOW_TX_H
#define ESPNOW_TX_H

#include "packet.h"

// MAC address of the receiver ESP32. Replace with the actual address
// printed by the receiver sketch on boot (WiFi.macAddress()).
// Example: {0xA4, 0xCF, 0x12, 0x34, 0x56, 0x78}
extern uint8_t RECEIVER_MAC[6];

// Call once in setup(). Brings up Wi-Fi in STA mode (required by
// ESP-NOW, but no router/AP connection is made), initializes ESP-NOW,
// and registers the receiver as a peer.
// Returns true on success.
bool espnow_tx_init();

// Sends one ControlPacket to the registered receiver peer.
// Returns true if the packet was handed off successfully.
bool espnow_tx_send(const ControlPacket &packet);

// True if the last send was acknowledged by the ESP-NOW driver
// (delivery to the radio layer, not an application-level ACK).
bool espnow_tx_last_send_ok();

#endif // ESPNOW_TX_H
