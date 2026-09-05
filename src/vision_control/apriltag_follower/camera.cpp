#include "esp_camera.h"   // must come before constants.h (CAM_FRAMESIZE uses framesize_t)
#include "camera.h"
#include "constants.h"
#include <Arduino.h>

// ---------------------------------------------------------------------
// Pin configuration — ESP32-S3-CAM boards built around the ESP32-S3-
// WROOM-1 N16R8 module (Goouuu / Keyestudio MB0184 / Freenove S3 CAM
// and most AliExpress "ESP32-S3-CAM OV2640/OV3660" boards) all use the
// CAMERA_MODEL_ESP32S3_EYE map below, verified against the Arduino
// CameraWebServer example's camera_pins.h and the Keyestudio MB0184
// pin table. A wrong map fails loudly (esp_camera_init error or a
// black frame), not subtly.
// ---------------------------------------------------------------------
#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM     15
#define SIOD_GPIO_NUM      4
#define SIOC_GPIO_NUM      5

#define Y9_GPIO_NUM       16
#define Y8_GPIO_NUM       17
#define Y7_GPIO_NUM       18
#define Y6_GPIO_NUM       12
#define Y5_GPIO_NUM       10
#define Y4_GPIO_NUM        8
#define Y3_GPIO_NUM        9
#define Y2_GPIO_NUM       11
#define VSYNC_GPIO_NUM     6
#define HREF_GPIO_NUM      7
#define PCLK_GPIO_NUM     13

bool camera_init()
{
    camera_config_t config = {};
    // XCLK is generated with LEDC timer 0 / channel 0. tilt_servo.cpp
    // deliberately uses timer 1 / channel 2 through the IDF driver so
    // the two never collide (the Arduino ledcAttach() allocator does
    // not know about the camera's timer and could otherwise grab it).
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer   = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sccb_sda = SIOD_GPIO_NUM;
    config.pin_sccb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;

    // Grayscale straight from the sensor — the detector wants exactly
    // this, so no per-frame conversion. The frame size is fixed here and
    // never changed at runtime: calling set_framesize() later in
    // grayscale mode is a known way to get "frame buffer size mismatch"
    // capture failures with esp32-camera.
    config.pixel_format = PIXFORMAT_GRAYSCALE;
    config.frame_size   = CAM_FRAMESIZE;
    config.fb_location  = CAMERA_FB_IN_PSRAM;
    config.fb_count     = 2;                   // double-buffer (needs PSRAM)
    config.grab_mode    = CAMERA_GRAB_LATEST;  // always process the newest frame

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK)
    {
        Serial.printf("[camera] esp_camera_init failed: 0x%x\n", (unsigned)err);
        return false;
    }

    sensor_t *s = esp_camera_sensor_get();
    if (s == nullptr)
    {
        return false;
    }

    if (s->id.PID == OV3660_PID)
    {
        // Same tweaks the stock CameraWebServer applies to this sensor.
        s->set_brightness(s, 1);
    }
    else
    {
        Serial.printf("[camera] note: sensor PID 0x%x is not an OV3660; orientation "
                      "constants in constants.h may need flipping\n", (unsigned)s->id.PID);
    }
    s->set_vflip(s, CAMERA_VFLIP);
    s->set_hmirror(s, CAMERA_HMIRROR);

    // Sanity-check the actual frame size matches what constants.h assumes,
    // since a mismatch silently breaks every pixel-based calculation
    // downstream (bearing, elevation, range, deadzones).
    camera_fb_t *fb = esp_camera_fb_get();
    if (fb == nullptr)
    {
        Serial.println("[camera] first capture failed");
        return false;
    }
    bool sizeOk = (fb->width == FRAME_WIDTH) && (fb->height == FRAME_HEIGHT);
    if (!sizeOk)
    {
        Serial.printf("[camera] frame is %ux%u but constants.h says %dx%d\n",
                      (unsigned)fb->width, (unsigned)fb->height, FRAME_WIDTH, FRAME_HEIGHT);
    }
    esp_camera_fb_return(fb);
    return sizeOk;
}

bool camera_capture(GrayFrame &out)
{
    camera_fb_t *fb = esp_camera_fb_get();
    if (fb == nullptr)
    {
        return false;
    }

    if (fb->format != PIXFORMAT_GRAYSCALE)
    {
        // Shouldn't happen given the init config above, but don't hand
        // back a frame the detector will misinterpret as raw grayscale.
        esp_camera_fb_return(fb);
        return false;
    }

    out.width  = (int32_t)fb->width;
    out.height = (int32_t)fb->height;
    out.stride = (int32_t)fb->width;  // tightly packed, 1 byte/pixel
    out.buf    = fb->buf;
    out._fb    = (void *)fb;

    return true;
}

void camera_release(GrayFrame &frame)
{
    if (frame._fb != nullptr)
    {
        esp_camera_fb_return((camera_fb_t *)frame._fb);
        frame._fb = nullptr;
        frame.buf = nullptr;
    }
}
