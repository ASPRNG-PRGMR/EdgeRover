#ifndef PACKET_H
#define PACKET_H

#include <stdint.h>

// Version 3: Supports signed int16_t PWM duties (-255 to +255) for
// through-zero tank turns, reverse-capable inner wheel mixing, and in-place pivots.
#define PACKET_VERSION 3

// Button bitmask positions
#define BTN_A      (1 << 0)
#define BTN_B      (1 << 1)
#define BTN_C      (1 << 2)
#define BTN_D      (1 << 3)

struct __attribute__((packed)) ControlPacket
{
    uint8_t  version;        // PACKET_VERSION (3)
    int16_t  leftPWM;        // -255 to +255 (signed: + = forward, - = reverse)
    int16_t  rightPWM;       // -255 to +255 (signed: + = forward, - = reverse)
    uint8_t  buttons;        // bitmask
    uint8_t  mode;           // 2 = vision/autonomous mode
    uint8_t  armed;          // 0 = disarmed, 1 = armed
    uint32_t packetCounter;  // increments every send
};

#endif // PACKET_H
