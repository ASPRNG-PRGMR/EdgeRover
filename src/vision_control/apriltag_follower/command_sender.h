#ifndef COMMAND_SENDER_H
#define COMMAND_SENDER_H

#include <stdint.h>
#include "tracker_control.h"

// Dedicated 50 Hz ESP-NOW sender.
//
// The receiver (bot_controller/receiver) goes to failsafe 200 ms after
// the last packet. AprilTag detection can take longer than that per
// frame, so the vision loop must not be the thing pacing the radio.
// This module runs a FreeRTOS task pinned to core 0 (the Wi-Fi core;
// the Arduino loop() lives on core 1) that re-sends the most recent
// DriveCommand every SEND_INTERVAL_MS, ramps PWM changes at
// PWM_SLEW_UP/DOWN_PER_TICK, and zeroes the output if no fresh command
// has arrived for COMMAND_STALE_MS (vision loop wedged / camera died).

// Call once in setup(), after espnow_tx_init(). Returns true on success.
bool command_sender_init();

// Publish the latest command from the vision loop. Cheap; safe to call
// every frame. `armed` maps straight to ControlPacket.armed: while it is
// false the receiver holds the TB6612FNG in standby (driver disabled),
// which is where the rover should sit until the first tag has been seen.
void command_sender_set(const DriveCommand &cmd, bool armed);

// Diagnostics.
uint32_t command_sender_packets_sent();
bool     command_sender_last_send_ok();
void     command_sender_get_output(int16_t &left, int16_t &right);

#endif // COMMAND_SENDER_H
