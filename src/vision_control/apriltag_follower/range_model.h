#ifndef RANGE_MODEL_H
#define RANGE_MODEL_H

#include "tag_detector.h"

// Everything the controller needs to know about where the tag is,
// expressed in the ROVER frame (x right, y up, z forward, origin at the
// camera). Angles in degrees, distances in the units of REAL_TAG_SIZE_CM.
struct RangeEstimate
{
    bool  valid;

    float in_frame_bearing_deg;    // tag offset from frame centre, horizontal (+ = right)
    float in_frame_elev_deg;       // tag offset from frame centre, vertical   (+ = above)

    float slant_cm;                // straight-line camera → tag (the pinhole/pose range)
    float bearing_deg;             // heading error the rover has to steer out (+ = right)
    float elevation_deg;           // line-of-sight angle above horizontal, tilt included
    float height_cm;               // vertical leg  h = r·sin(elevation)
    float ground_cm;               // horizontal leg d = sqrt(r² − h²)   ← what we follow on
    float ground_filtered_cm;      // EMA of ground_cm (RANGE_EMA_ALPHA)
};

void range_model_init();

// Convert one detection + the current camera tilt into a RangeEstimate.
// Returns valid=false when the detection is empty or the geometry is
// degenerate. The EMA keeps its state across calls; call
// range_model_reset() after a long loss so the filter doesn't drag an
// old distance into the re-acquire.
RangeEstimate range_model_update(const TagDetection &det, float tilt_deg);
void range_model_reset();

#endif // RANGE_MODEL_H
