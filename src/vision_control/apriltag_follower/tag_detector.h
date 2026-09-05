#ifndef TAG_DETECTOR_H
#define TAG_DETECTOR_H

#include <stdint.h>
#include "camera.h"

// Result of one detection pass — deliberately flat/simple so the main
// loop and control math never have to touch apriltag's own types.
struct TagDetection
{
    bool  found;
    int   id;
    int   hamming;
    float cx;               // tag centre, pixels, frame coordinates (x right, y DOWN)
    float cy;
    float size_px;          // apparent side length, pixels — see tag_detector.cpp for how
    float decision_margin;  // detector confidence; higher = more confident
    float corners[4][2];    // outer black-border corners, image pixels

    // Only filled when USE_POSE_ESTIMATE == 1 (constants.h): tag centre in
    // the camera frame, same units as REAL_TAG_SIZE_CM. x right, y down,
    // z forward along the optical axis.
    bool  pose_valid;
    float tx, ty, tz;

    uint32_t detect_ms;     // wall time spent inside the detector for this frame
};

// Call once in setup(). Creates the AprilTag detector and registers
// the tag36h11 family (change here if you print a different family).
// Returns true on success.
bool tag_detector_init();

// Runs detection on one grayscale frame and returns the best candidate
// that passes the ID / hamming / decision-margin filters in constants.h,
// or found=false if none did. "Best" = largest apparent size, i.e. the
// closest tag, which is the right choice for a follower (the library's
// decision_margin is explicitly not meaningful for large tags).
TagDetection tag_detector_detect(const GrayFrame &frame);

#endif // TAG_DETECTOR_H
