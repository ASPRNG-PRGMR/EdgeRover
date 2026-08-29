#ifndef TAG_DETECTOR_H
#define TAG_DETECTOR_H

#include "camera.h"

// Result of one detection pass — deliberately flat/simple so the main
// loop and control math never have to touch apriltag's own types.
struct TagDetection
{
    bool  found;
    int   id;
    float cx;         // tag center, pixels, frame coordinates
    float cy;
    float size_px;    // apparent tag size, pixels (avg of 4 side lengths)
    float decision_margin;  // detector confidence; higher = more confident
};

// Call once in setup(). Creates the AprilTag detector and registers
// the tag36h11 family (change here if you print a different family).
// Returns true on success.
bool tag_detector_init();

// Runs detection on one grayscale frame and returns the single best
// detection (highest decision_margin among all tags found), or
// found=false if no tag was detected.
//
// NOTE: if you print multiple tags with different IDs for different
// purposes later, this is the function to extend — right now it
// deliberately collapses everything to "best single tag" since v1
// only follows one target.
TagDetection tag_detector_detect(const GrayFrame &frame);

#endif // TAG_DETECTOR_H
