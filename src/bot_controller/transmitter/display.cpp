#include "display.h"
#include <Arduino.h>
#include <SPI.h>
#include <TFT_22_ILI9225.h>


#define TFT_CS   19
#define TFT_RS   21
#define TFT_MOSI 22
#define TFT_SCLK 18
#define TFT_RST  23
#define TFT_LED  -1      // No LED pin

// Native ILI9225 panel resolution (portrait). If your board is
// landscape-wired, call tft.setOrientation(1) in display_init()
// and swap these two.
#define TFT_WIDTH    176
#define TFT_HEIGHT   220

// TFT_22_ILI9225 only talks over the ESP32's hardware SPI peripheral
// (unlike Adafruit_ST7789's bit-banged software SPI), so the pins
// above get remapped onto the HSPI bus in display_init() via
// SPI.begin(). MISO is unused - pass -1, the display is write-only.
static TFT_22_ILI9225 tft(TFT_RST, TFT_RS, TFT_CS, TFT_LED);

// TFT_22_ILI9225 doesn't ship every 16-bit color name Adafruit_GFX
// does, so these are defined directly as RGB565 values.
#define COLOR_BLACK   0x0000
#define COLOR_WHITE   0xFFFF
#define COLOR_RED     0xF800
#define COLOR_GREEN   0x07E0
#define COLOR_YELLOW  0xFFE0
#define COLOR_CYAN    0x07FF

static const int16_t SPEED_BAR_X = 18, SPEED_BAR_Y = 86,  SPEED_BAR_W = 140, SPEED_BAR_H = 20;
static const int16_t STEER_BAR_X = 18, STEER_BAR_Y = 148, STEER_BAR_W = 140, STEER_BAR_H = 20;

// Dirty-tracking so display_update() only repaints what changed -
// keeps this fast enough over SPI to call every loop, and avoids
// the whole-screen flicker a full redraw would cause.
static int16_t lastSpeedFillW  = -1;
static int16_t lastSteerFillX  = -1, lastSteerFillW = -1;
static uint8_t lastLeftPWM = 255, lastRightPWM = 255; // impossible values force first draw
static int8_t  lastArmedState = -1;                    // -1 forces first draw

static void drawStaticChrome()
{
    tft.clear();
    tft.setBackgroundColor(COLOR_BLACK);

    tft.setFont(Terminal12x16);
    tft.drawText(58, 6, "ROVER", COLOR_WHITE);

    tft.setFont(Terminal6x8);
    tft.drawText(SPEED_BAR_X, SPEED_BAR_Y - 16, "SPEED", COLOR_WHITE);
    tft.drawRectangle(SPEED_BAR_X - 1, SPEED_BAR_Y - 1,
                       SPEED_BAR_X + SPEED_BAR_W, SPEED_BAR_Y + SPEED_BAR_H, COLOR_WHITE);

    tft.drawText(SPEED_BAR_X, STEER_BAR_Y - 16, "STEER   L <----> R", COLOR_WHITE);
    tft.drawRectangle(STEER_BAR_X - 1, STEER_BAR_Y - 1,
                       STEER_BAR_X + STEER_BAR_W, STEER_BAR_Y + STEER_BAR_H, COLOR_WHITE);
    tft.drawLine(STEER_BAR_X + STEER_BAR_W / 2, STEER_BAR_Y - 4,
                 STEER_BAR_X + STEER_BAR_W / 2, STEER_BAR_Y + STEER_BAR_H + 4, COLOR_WHITE);
}

static void updateArmedBanner(bool armed)
{
    int8_t state = armed ? 1 : 0;
    if (state == lastArmedState) return;
    lastArmedState = state;

    tft.fillRectangle(0, 30, TFT_WIDTH - 1, 53, COLOR_BLACK);
    tft.setFont(Terminal12x16);
    if (armed)
    {
        tft.drawText(58, 32, "ARMED", COLOR_GREEN);
    }
    else
    {
        tft.drawText(40, 32, "DISARMED", COLOR_RED);
    }
}

static void updateSpeedBar(uint8_t speedPercent)
{
    int16_t fillW = map(speedPercent, 0, 100, 0, SPEED_BAR_W);
    if (fillW == lastSpeedFillW) return;

    if (fillW > lastSpeedFillW)
    {
        int16_t from = SPEED_BAR_X + max(lastSpeedFillW, (int16_t)0);
        tft.fillRectangle(from, SPEED_BAR_Y,
                           SPEED_BAR_X + fillW - 1, SPEED_BAR_Y + SPEED_BAR_H - 1, COLOR_CYAN);
    }
    else
    {
        tft.fillRectangle(SPEED_BAR_X + fillW, SPEED_BAR_Y,
                           SPEED_BAR_X + lastSpeedFillW - 1, SPEED_BAR_Y + SPEED_BAR_H - 1, COLOR_BLACK);
    }
    lastSpeedFillW = fillW;

    tft.fillRectangle(SPEED_BAR_X, SPEED_BAR_Y + SPEED_BAR_H + 6,
                       SPEED_BAR_X + 60, SPEED_BAR_Y + SPEED_BAR_H + 20, COLOR_BLACK);
    tft.setFont(Terminal6x8);
    char buf[8];
    snprintf(buf, sizeof(buf), "%3d%%", speedPercent);
    tft.drawText(SPEED_BAR_X, SPEED_BAR_Y + SPEED_BAR_H + 6, buf, COLOR_WHITE);
}

static void updateSteerBar(int8_t steerPercent)
{
    int16_t halfW = STEER_BAR_W / 2;
    int16_t offset = map(steerPercent, -100, 100, -halfW, halfW);

    int16_t fillX, fillW;
    if (offset >= 0) { fillX = STEER_BAR_X + halfW; fillW = offset; }
    else              { fillX = STEER_BAR_X + halfW + offset; fillW = -offset; }

    if (fillX == lastSteerFillX && fillW == lastSteerFillW) return;

    // Clear the whole bar interior, then redraw the center tick and
    // the new fill - simplest way to handle the fill moving from one
    // side to the other without leaving stray pixels behind.
    tft.fillRectangle(STEER_BAR_X, STEER_BAR_Y,
                       STEER_BAR_X + STEER_BAR_W - 1, STEER_BAR_Y + STEER_BAR_H - 1, COLOR_BLACK);
    tft.drawLine(STEER_BAR_X + halfW, STEER_BAR_Y - 4,
                 STEER_BAR_X + halfW, STEER_BAR_Y + STEER_BAR_H + 4, COLOR_WHITE);
    if (fillW > 0)
    {
        tft.fillRectangle(fillX, STEER_BAR_Y,
                           fillX + fillW - 1, STEER_BAR_Y + STEER_BAR_H - 1, COLOR_YELLOW);
    }
    lastSteerFillX = fillX;
    lastSteerFillW = fillW;

    tft.fillRectangle(STEER_BAR_X, STEER_BAR_Y + STEER_BAR_H + 6,
                       STEER_BAR_X + 90, STEER_BAR_Y + STEER_BAR_H + 20, COLOR_BLACK);
    tft.setFont(Terminal6x8);
    char buf[10];
    if (steerPercent > 2)       snprintf(buf, sizeof(buf), "R %3d%%", steerPercent);
    else if (steerPercent < -2) snprintf(buf, sizeof(buf), "L %3d%%", -steerPercent);
    else                        snprintf(buf, sizeof(buf), "CENTER");
    tft.drawText(STEER_BAR_X, STEER_BAR_Y + STEER_BAR_H + 6, buf, COLOR_WHITE);
}

static void updateLRReadout(uint8_t leftPWM, uint8_t rightPWM)
{
    if (leftPWM == lastLeftPWM && rightPWM == lastRightPWM) return;
    lastLeftPWM = leftPWM;
    lastRightPWM = rightPWM;

    tft.fillRectangle(0, TFT_HEIGHT - 20, TFT_WIDTH - 1, TFT_HEIGHT - 1, COLOR_BLACK);
    tft.setFont(Terminal6x8);
    char buf[24];
    snprintf(buf, sizeof(buf), "L PWM: %3d  R PWM: %3d", leftPWM, rightPWM);
    tft.drawText(6, TFT_HEIGHT - 18, buf, COLOR_WHITE);
}

void display_init()
{
    // Remap ESP32 hardware SPI (HSPI) onto the pins the panel is
    // wired to. TFT_22_ILI9225 always drives whatever bus SPI.begin()
    // set up, unlike Adafruit_ST7789's software-SPI constructor which
    // could take arbitrary pins directly.
    SPI.begin(TFT_SCLK, -1 /* MISO unused */, TFT_MOSI, TFT_CS);

    tft.begin();
    tft.setOrientation(0);
    drawStaticChrome();
}

void display_update(uint8_t leftPWM, uint8_t rightPWM,
                     uint8_t speedPercent, int8_t steerPercent,
                     bool armed)
{
    updateArmedBanner(armed);
    updateSpeedBar(speedPercent);
    updateSteerBar(steerPercent);
    updateLRReadout(leftPWM, rightPWM);
}