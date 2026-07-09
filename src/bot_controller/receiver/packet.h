#ifndef PACKET_H
#define PACKET_H

#include <stdint.h>

// Bump this if you ever change the layout of ControlPacket.
// The receiver can use it to reject packets from a mismatched build.
// v2: throttle/steering (raw stick axes) replaced with leftPWM/rightPWM
// (final, ready-to-drive PWM duty per side) now that speed comes from
// a potentiometer and steering from a rotary encoder on the transmitter.
#define PACKET_VERSION 2

// Button bitmask positions (example layout, expand as needed)
#define BTN_A      (1 << 0)
#define BTN_B      (1 << 1)
#define BTN_C      (1 << 2)
#define BTN_D      (1 << 3)

// This struct is sent as-is over ESP-NOW, so it must be byte-identical
// on both the transmitter and receiver. __attribute__((packed)) removes
// any compiler padding so the layout is predictable.
struct __attribute__((packed)) ControlPacket
{
    uint8_t  version;        // PACKET_VERSION, used for sanity checking
    uint8_t  leftPWM;        // 0-255, final left motor PWM duty (forward only)
    uint8_t  rightPWM;       // 0-255, final right motor PWM duty (forward only)
    uint8_t  buttons;        // bitmask, see BTN_* defines above
    uint8_t  mode;           // flight/drive mode selector
    uint8_t  armed;          // 0 = disarmed, 1 = armed
    uint32_t packetCounter;  // increments every send, useful for loss stats
};

#endif // PACKET_H
