#include "tag_detector.h"
#include "constants.h"
#include <Arduino.h>
#include <math.h>

// ---------------------------------------------------------------------
// Library: raspiduino/apriltag-esp32 — the UMich AprilTag 3 library
// repackaged as an Arduino library, with doubles turned into floats and
// tag36h11 trimmed to 35 codes so the lookup tables fit an ESP32.
//   https://github.com/raspiduino/apriltag-esp32
// Install: Sketch > Include Library > Add .ZIP Library (or clone it into
// ~/Arduino/libraries/). Board settings that matter: "PSRAM: OPI PSRAM"
// (the N16R8 module has octal PSRAM; without it the detector's working
// buffers will not fit) and any partition scheme with >= 1.5 MB app.
// ---------------------------------------------------------------------
extern "C" {
#include "apriltag.h"
#include "tag36h11.h"
#include "common/image_u8.h"
#if USE_POSE_ESTIMATE
#include "apriltag_pose.h"
#include "common/matd.h"
#endif
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

    tagDetector->quad_decimate     = QUAD_DECIMATE;
    tagDetector->quad_sigma        = QUAD_SIGMA;
    tagDetector->nthreads          = 1;     // core 0 is busy with Wi-Fi + the sender task
    tagDetector->refine_edges      = REFINE_EDGES;
    tagDetector->decode_sharpening = DECODE_SHARPENING;

    return true;
}

// Apparent tag size in pixels.
//
// Why not just average all four sides: the camera looks UP at a tag
// that hangs vertically on a pocket. Near the rover the line of sight
// is steep, so the tag is foreshortened vertically — its two vertical
// edges shrink while the two horizontal edges keep (to first order)
// the length f·S/r that the pinhole model wants. Averaging all four
// would overestimate range exactly where the 15 cm stop matters most.
// The same happens sideways when the person turns. So: group the edges
// by orientation, average each group, and take the larger of the two —
// foreshortening only ever shrinks an edge.
static float compute_apparent_size(const apriltag_detection_t *det)
{
    float sumH = 0.0f, sumV = 0.0f, sumAll = 0.0f;
    int   nH = 0, nV = 0;
    for (int i = 0; i < 4; i++)
    {
        int j = (i + 1) % 4;
        float dx  = (float)(det->p[j][0] - det->p[i][0]);
        float dy  = (float)(det->p[j][1] - det->p[i][1]);
        float len = sqrtf(dx * dx + dy * dy);
        sumAll += len;
        if (fabsf(dx) >= fabsf(dy)) { sumH += len; nH++; }
        else                        { sumV += len; nV++; }
    }
    float meanH = (nH > 0) ? sumH / nH : 0.0f;
    float meanV = (nV > 0) ? sumV / nV : 0.0f;
    float best  = (meanH > meanV) ? meanH : meanV;
    // A tag rotated ~45° in the image puts all four edges in one bucket;
    // fall back to the plain mean in that case rather than a 1-edge guess.
    if (nH < 2 || nV < 2)
    {
        best = sumAll / 4.0f;
    }
    return best;
}

static bool passes_filters(const apriltag_detection_t *det)
{
    if (TARGET_TAG_ID >= 0 && det->id != TARGET_TAG_ID) return false;
    if (det->hamming > MAX_HAMMING)                    return false;
    if (det->decision_margin < MIN_DECISION_MARGIN)    return false;
    return true;
}

TagDetection tag_detector_detect(const GrayFrame &frame)
{
    TagDetection result = {};
    result.found = false;

    uint32_t t0 = millis();

    // Positional aggregate init: image_u8_t's first three fields are const.
    image_u8_t im = { frame.width, frame.height, frame.stride, frame.buf };

    zarray_t *detections = apriltag_detector_detect(tagDetector, &im);
    if (detections == nullptr)
    {
        result.detect_ms = millis() - t0;
        return result;
    }

    apriltag_detection_t *best = nullptr;
    float bestSize = 0.0f;
    int n = zarray_size(detections);
    for (int i = 0; i < n; i++)
    {
        apriltag_detection_t *det;
        zarray_get(detections, i, &det);
        if (!passes_filters(det)) continue;

        float size = compute_apparent_size(det);
        if (best == nullptr || size > bestSize)
        {
            best = det;
            bestSize = size;
        }
    }

    if (best != nullptr)
    {
        result.found   = true;
        result.id      = best->id;
        result.hamming = best->hamming;
        result.cx      = (float)best->c[0];
        result.cy      = (float)best->c[1];
        result.size_px = bestSize;
        result.decision_margin = best->decision_margin;
        for (int k = 0; k < 4; k++)
        {
            result.corners[k][0] = (float)best->p[k][0];
            result.corners[k][1] = (float)best->p[k][1];
        }

#if USE_POSE_ESTIMATE
        // Homography → pose. Units follow tagsize, so passing cm gives cm.
        apriltag_detection_info_t info;
        info.det     = best;
        info.tagsize = REAL_TAG_SIZE_CM;
        info.fx      = FOCAL_LENGTH_PX;
        info.fy      = FOCAL_LENGTH_PX;
        info.cx      = PRINCIPAL_X_PX;
        info.cy      = PRINCIPAL_Y_PX;

        apriltag_pose_t pose = {};
        estimate_tag_pose(&info, &pose);
        if (pose.t != nullptr)
        {
            result.tx = (float)MATD_EL(pose.t, 0, 0);
            result.ty = (float)MATD_EL(pose.t, 1, 0);
            result.tz = (float)MATD_EL(pose.t, 2, 0);
            result.pose_valid = (result.tz > 0.0f);
        }
        if (pose.R != nullptr) matd_destroy(pose.R);
        if (pose.t != nullptr) matd_destroy(pose.t);
#endif
    }

    apriltag_detections_destroy(detections);
    result.detect_ms = millis() - t0;
    return result;
}
