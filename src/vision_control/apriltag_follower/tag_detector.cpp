#include "tag_detector.h"

// ---------------------------------------------------------------------
// Requires the UMich AprilTag C library vendored into this project
// (https://github.com/AprilRobotics/apriltag). It's plain, portable C
// with no hard OS dependency, but it is NOT an Arduino library out of
// the box — per the roadmap's Phase 3 checklist, get it building
// standalone against your ESP-IDF/Arduino-ESP32 toolchain BEFORE
// wiring it into this file. Two common ways to bring it in:
//   1. As an ESP-IDF component (add a component.mk / CMakeLists.txt
//      that just globs the library's .c files), or
//   2. Vendored as an Arduino library under your sketchbook's
//      libraries/apriltag/ with a src/ subfolder, since the Arduino
//      builder only auto-compiles .c/.cpp files that live inside a
//      proper library structure, not arbitrary subfolders of a sketch.
// Either way, only common/, apriltag.c, apriltag_quad_thresh.c, and
// tag36h11.c are needed for this single-family, single-thread setup —
// you don't need the whole repo (e.g. skip the other tag families,
// apps/, and the OpenCV-dependent bits).
// ---------------------------------------------------------------------
extern "C" {
#include "apriltag.h"
#include "tag36h11.h"
#include "common/image_u8.h"
}

static apriltag_family_t   *tagFamily   = nullptr;
static apriltag_detector_t *tagDetector = nullptr;

bool tag_detector_init()
{
    tagFamily = tag36h11_create();
    if (tagFamily == nullptr)
    {
        return false;
    }

    tagDetector = apriltag_detector_create();
    if (tagDetector == nullptr)
    {
        return false;
    }

    apriltag_detector_add_family(tagDetector, tagFamily);

    // Tuned for an MCU, not a desktop. quad_decimate > 1 trades
    // detection range for speed — this is very likely the first knob
    // you'll want to turn once you measure real detection FPS in
    // Phase 3. Start conservative (correctness first), speed up later.
    tagDetector->quad_decimate   = 2.0f;
    tagDetector->quad_sigma      = 0.0f;
    tagDetector->nthreads        = 1;     // single core budget assumed
    tagDetector->refine_edges    = 1;
    tagDetector->decode_sharpening = 0.25;

    return true;
}

// Apparent tag size = average of the 4 side lengths, in pixels.
// Using the average (not just one side) makes this more robust to
// the tag being viewed at a slight angle rather than dead-on.
static float compute_apparent_size(const apriltag_detection_t *det)
{
    float total = 0.0f;
    for (int i = 0; i < 4; i++)
    {
        int j = (i + 1) % 4;
        float dx = (float)(det->p[j][0] - det->p[i][0]);
        float dy = (float)(det->p[j][1] - det->p[i][1]);
        total += sqrtf(dx * dx + dy * dy);
    }
    return total / 4.0f;
}

TagDetection tag_detector_detect(const GrayFrame &frame)
{
    TagDetection result = {};
    result.found = false;

    image_u8_t im = {
        .width  = frame.width,
        .height = frame.height,
        .stride = frame.stride,
        .buf    = frame.buf,
    };

    zarray_t *detections = apriltag_detector_detect(tagDetector, &im);
    if (detections == nullptr)
    {
        return result;
    }

    // Pick the single best detection by decision_margin (detector's own
    // confidence score) rather than just "first in array" or "largest",
    // since a large-but-marginal false-ish detection is worse than a
    // smaller, confident one.
    apriltag_detection_t *best = nullptr;
    int n = zarray_size(detections);
    for (int i = 0; i < n; i++)
    {
        apriltag_detection_t *det;
        zarray_get(detections, i, &det);
        if (best == nullptr || det->decision_margin > best->decision_margin)
        {
            best = det;
        }
    }

    if (best != nullptr)
    {
        result.found = true;
        result.id = best->id;
        result.cx = (float)best->c[0];
        result.cy = (float)best->c[1];
        result.size_px = compute_apparent_size(best);
        result.decision_margin = best->decision_margin;
    }

    apriltag_detections_destroy(detections);
    return result;
}
