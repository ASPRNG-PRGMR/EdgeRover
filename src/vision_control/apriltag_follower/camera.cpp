#include "camera.h"
#include "constants.h"
#include "esp_camera.h"

// ---------------------------------------------------------------------
// Pin configuration — this is the single most board-specific part of
// this file. The values below are a common ESP32-S3-CAM layout, but
// CONFIRM against your exact board's schematic/silkscreen before
// trusting this. Wrong pins here just gives a black frame or an init
// failure, not a subtle bug — so it fails loud, which is something.
// ---------------------------------------------------------------------
#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM     15
#define SIOD_GPIO_NUM      4
#define SIOC_GPIO_NUM      5

#define Y9_GPIO_NUM        16
#define Y8_GPIO_NUM        17
#define Y7_GPIO_NUM        18
#define Y6_GPIO_NUM        12
#define Y5_GPIO_NUM        10
#define Y4_GPIO_NUM         8
#define Y3_GPIO_NUM         9
#define Y2_GPIO_NUM        11
#define VSYNC_GPIO_NUM      6
#define HREF_GPIO_NUM       7
#define PCLK_GPIO_NUM      13

bool camera_init()
{
    camera_config_t config = {};
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

    // Grayscale straight from the sensor — no need to capture RGB/JPEG
    // and convert. Saves cycles and RAM on every single frame.
    config.pixel_format = PIXFORMAT_GRAYSCALE;
    config.frame_size    = FRAMESIZE_QVGA;   // 320x240, matches constants.h
    config.fb_location   = CAMERA_FB_IN_PSRAM;
    config.fb_count      = 2;                // double-buffer; needs PSRAM
    config.grab_mode     = CAMERA_GRAB_LATEST;

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK)
    {
        return false;
    }

    // Sanity-check the actual frame size matches what constants.h assumes,
    // since a mismatch here silently breaks every pixel-based calculation
    // downstream (steering error, distance estimate, deadzone, etc.).
    sensor_t *s = esp_camera_sensor_get();
    if (s != nullptr)
    {
        s->set_framesize(s, FRAMESIZE_QVGA);
    }

    return true;
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
