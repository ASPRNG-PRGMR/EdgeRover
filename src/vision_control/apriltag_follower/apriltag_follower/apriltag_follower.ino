// EdgeRover — AprilTag Follower
//
// Runs on the ESP32-S3-CAM. Detects a printed AprilTag, computes
// steering + distance, and sends the result as a ControlPacket to the
// SAME receiver used by the original manual transmitter — the
// receiver.ino code is unchanged. It only understands leftPWM/rightPWM;
// it has no idea (and doesn't need to know) these numbers now come
// from a camera instead of a rotary encoder.
//
// Before trusting anything this sketch drives: work through the
// roadmap's Phase 0-2 checklists first (power validation, mechanical
// tests, camera bring-up in isolation). This sketch assumes all of
// that already passed.

#include "constants.h"
#include "camera.h"
#include "tag_detector.h"
#include "tracker_control.h"
#include "espnow_tx.h"
#include "packet.h"

static uint32_t packetCounter = 0;

void setup()
{
    Serial.begin(115200);
    delay(200);

    Serial.println("[apriltag_follower] starting up...");

    if (!camera_init())
    {
        Serial.println("[apriltag_follower] FATAL: camera_init failed");
        while (true) { delay(1000); }
    }
    Serial.println("[apriltag_follower] camera OK");

    if (!tag_detector_init())
    {
        Serial.println("[apriltag_follower] FATAL: tag_detector_init failed");
        while (true) { delay(1000); }
    }
    Serial.println("[apriltag_follower] detector OK");

    tracker_control_init();

    if (!espnow_tx_init())
    {
        Serial.println("[apriltag_follower] FATAL: espnow_tx_init failed");
        while (true) { delay(1000); }
    }
    Serial.println("[apriltag_follower] ESP-NOW OK, receiver peer registered");

    if (FOCAL_LENGTH_PX <= 0.0f || REAL_TAG_SIZE_CM <= 0.0f)
    {
        Serial.println("[apriltag_follower] WARNING: FOCAL_LENGTH_PX / REAL_TAG_SIZE_CM "
                        "not calibrated yet (constants.h) — distance estimate will be "
                        "invalid until these are set. Steering will still work.");
    }
}

void loop()
{
    GrayFrame frame;
    if (!camera_capture(frame))
    {
        Serial.println("[apriltag_follower] camera_capture failed, skipping frame");
        delay(10);
        return;
    }

    TagDetection det = tag_detector_detect(frame);
    camera_release(frame);   // done with the frame buffer as soon as detection is over

    DriveCommand cmd = tracker_control_update(det);

    ControlPacket packet = {};
    packet.version = PACKET_VERSION;
    packet.leftPWM  = cmd.leftPWM;
    packet.rightPWM = cmd.rightPWM;
    packet.buttons  = 0;
    packet.mode     = 2;   // 2 = vision/autonomous mode (1 reserved for manual, adjust to taste)
    packet.armed    = 1;   // rover is autonomy-armed; PWM 0 = commanded stop, not disarm
    packet.packetCounter = packetCounter++;

    bool sent = espnow_tx_send(packet);

    // Lightweight telemetry — cheap enough to leave in, useful for
    // Phase 3/4 bench tuning without needing a separate debug build.
    if (det.found)
    {
        Serial.printf("[tag id=%d] cx=%.1f cy=%.1f size=%.1f margin=%.1f -> L=%u R=%u %s\n",
                      det.id, det.cx, det.cy, det.size_px, det.decision_margin,
                      cmd.leftPWM, cmd.rightPWM, sent ? "" : "(SEND FAILED)");
    }
    else
    {
        Serial.printf("[no tag] -> L=%u R=%u %s\n",
                      cmd.leftPWM, cmd.rightPWM, sent ? "" : "(SEND FAILED)");
    }

#if SEND_INTERVAL_MS > 0
    delay(SEND_INTERVAL_MS);
#endif
}
