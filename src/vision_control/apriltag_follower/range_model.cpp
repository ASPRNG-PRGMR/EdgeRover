#include "range_model.h"
#include "constants.h"
#include <math.h>

// ---------------------------------------------------------------------
// Camera frame: x right, y DOWN, z forward (image / apriltag convention).
// Rover frame:  x right, y UP,   z forward, camera pitched up by `tilt`.
//
//   1. Get a vector to the tag in the camera frame.
//        size model: direction (dx, dy, f) scaled to slant r = f·S / size_px
//        pose model: (tx, ty, tz) straight from the homography solver
//   2. Rotate it about the x axis by the servo tilt into the rover frame.
//   3. Pythagoras:  r² = ground² + height²
//        height = y_rover,  ground = sqrt(x_rover² + z_rover²)
//      bearing = atan2(x_rover, z_rover), elevation = atan2(height, ground)
// ---------------------------------------------------------------------

static const float DEG = 180.0f / (float)M_PI;

static bool  emaValid = false;
static float emaGround = 0.0f;

void range_model_init()
{
    range_model_reset();
}

void range_model_reset()
{
    emaValid = false;
    emaGround = 0.0f;
}

RangeEstimate range_model_update(const TagDetection &det, float tilt_deg)
{
    RangeEstimate r = {};
    r.valid = false;

    if (!det.found)
    {
        return r;
    }

    const float f  = FOCAL_LENGTH_PX;
    const float dx = det.cx - PRINCIPAL_X_PX;   // + = right of centre
    const float dy = det.cy - PRINCIPAL_Y_PX;   // + = BELOW centre (image y is down)

    // In-frame angular offsets — these drive the tilt servo directly and
    // are valid regardless of calibration.
    r.in_frame_bearing_deg = atan2f( dx, f) * DEG;
    r.in_frame_elev_deg    = atan2f(-dy, f) * DEG;

    // --- 1. vector to the tag in the camera frame -------------------------
    float xc, yc_down, zc;
#if USE_POSE_ESTIMATE
    if (det.pose_valid)
    {
        xc = det.tx; yc_down = det.ty; zc = det.tz;
    }
    else
#endif
    {
        if (det.size_px < 1.0f || f <= 0.0f || REAL_TAG_SIZE_CM <= 0.0f)
        {
            return r;
        }
        float slant = (REAL_TAG_SIZE_CM * f) / det.size_px;
        float norm  = sqrtf(dx * dx + dy * dy + f * f);
        xc      = slant * dx / norm;
        yc_down = slant * dy / norm;
        zc      = slant * f  / norm;
    }

    // --- 2. rotate into the rover frame (camera pitched UP by tilt) --------
    float yc_up = -yc_down;
    float t  = tilt_deg / DEG;
    float st = sinf(t), ct = cosf(t);
    float xr = xc;
    float yr = yc_up * ct + zc * st;
    float zr = -yc_up * st + zc * ct;

    // --- 3. Pythagoras ----------------------------------------------------
    float slant  = sqrtf(xr * xr + yr * yr + zr * zr);
    float ground = sqrtf(xr * xr + zr * zr);          // == sqrt(slant² − yr²)

    if (zr <= 0.0f || slant < 1.0f)
    {
        // Tag "behind" the rover or numerically silly — refuse rather than
        // feed the controller a negative distance.
        return r;
    }

    r.slant_cm      = slant;
    r.height_cm     = yr;
    r.ground_cm     = ground;
    r.bearing_deg   = atan2f(xr, zr) * DEG;
    r.elevation_deg = atan2f(yr, ground) * DEG;

    if (!emaValid)
    {
        emaGround = ground;
        emaValid = true;
    }
    else
    {
        emaGround += RANGE_EMA_ALPHA * (ground - emaGround);
    }
    r.ground_filtered_cm = emaGround;

    r.valid = true;
    return r;
}
