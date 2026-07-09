#ifndef ESPNOW_RX_H
#define ESPNOW_RX_H

#include "packet.h"

// Call once in setup(). Brings up Wi-Fi in STA mode (required by
// ESP-NOW, but no router/AP connection is made) and initializes ESP-NOW
// reception. Returns true on success.
bool espnow_rx_init();

// Copies the most recently received packet into 'outPacket'.
// Returns true if at least one packet has ever been received.
// This function only copies data - it performs no failsafe logic and
// no output updates by design.
bool espnow_rx_get_latest(ControlPacket &outPacket);

// Milliseconds since the last packet was received (millis() based).
// Returns a very large value if no packet has ever been received yet,
// so callers naturally treat "never received" as "failsafe".
uint32_t espnow_rx_time_since_last_packet();

#endif // ESPNOW_RX_H
