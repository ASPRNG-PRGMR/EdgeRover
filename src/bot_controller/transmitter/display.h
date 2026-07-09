#ifndef DISPLAY_H
#define DISPLAY_H

#include <stdint.h>

// ILI9225 status display: shows arm state, speed ceiling, and steering
// offset every time the transmitter sends a packet.
//
// Pins:  CLK(SCK)=5   SDA(MOSI)=15   RS=2   RST=4   CS=18
// The TFT_22_ILI9225 library only drives the panel over the ESP32's
// hardware SPI peripheral, so display_init() remaps HSPI's SCK/MOSI
// pins to the ones above via SPI.begin() before the panel is started.
// (No MISO/CIPO is used - the display is write-only.)
//
// display_update() only repaints the small regions that changed, so
// it's cheap enough to call every send (~50Hz).
//
// Requires the "TFT_22_ILI9225" library (Arduino Library Manager).
// Assumes a 176x220 panel - adjust TFT_WIDTH/TFT_HEIGHT in display.cpp
// if yours is a different size.

// Call once in setup().
void display_init();

// Call after each inputs_read(). speedPercent is 0-100, steerPercent
// is -100 (full left) .. +100 (full right).
void display_update(uint8_t leftPWM, uint8_t rightPWM,
                     uint8_t speedPercent, int8_t steerPercent,
                     bool armed);

#endif // DISPLAY_H