#ifndef CAMERA_H
#define CAMERA_H

#include <stdint.h>

// Minimal grayscale image handle passed to the tag detector.
// Mirrors apriltag's image_u8_t field layout so tag_detector.cpp can
// wrap it directly without a copy.
struct GrayFrame
{
    int32_t  width;
    int32_t  height;
    int32_t  stride;   // bytes per row; equals width for tightly packed grayscale
    uint8_t *buf;      // points into the camera driver's own frame buffer
    void    *_fb;      // opaque camera_fb_t*, needed to release the buffer
};

// Call once in setup(). Configures the OV3660 in 8-bit grayscale at
// CAM_FRAMESIZE (see constants.h), applies the OV3660 orientation fix,
// and double-buffers in PSRAM. Returns true on success.
bool camera_init();

// Blocking capture of one frame. Returns true and fills `out` on
// success. The underlying buffer is only valid until camera_release()
// is called — call that as soon as you're done reading `out.buf`.
bool camera_capture(GrayFrame &out);

// Releases the frame buffer obtained from camera_capture(). Must be
// called exactly once per successful camera_capture() before the next
// capture, or the camera driver will stall waiting for a free buffer.
void camera_release(GrayFrame &frame);

#endif // CAMERA_H
