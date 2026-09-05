// EdgeRover — AprilTag Follower
//
// Runs on the ESP32-S3-CAM (N16R8 + OV3660). Detects a printed AprilTag
// worn on the person's back pocket, keeps it centred with an SG90 tilt
// servo, works out the true ground distance from the pinhole range and
// the servo angle (Pythagoras — see constants.h / range_model.cpp), and
// sends left/right PWM as ControlPacket v3 to the SAME, unmodified
// bot_controller/receiver that the handheld transmitter talks to. The
// receiver neither knows nor cares that the numbers now come from a
// camera; the rover steers by tank-turn / in-place pivot exactly as it
// does under manual control.
//
// Pipeline per frame (core 1, this loop):
//   capture → detect → range model → tracker → publish command → tilt servo
// Radio (core 0, command_sender.cpp): 50 Hz re-send of the latest command
// with slew limiting and a stale-command cut-off, so the receiver's 200 ms
// failsafe is never tripped by a slow detection.
//
// Before first run: work through the notes at the top of constants.h
// (tag size, focal length calibration, servo level angle, orientation
// flags) and the README in ../model/.

#include <Arduino.h>
#include "constants.h"
#include "camera.h"
#include "tag_detector.h"
#include "tilt_servo.h"
#include "range_model.h"
#include "tracker_control.h"
#include "espnow_tx.h"
#include "command_sender.h"
#include "packet.h"

static void halt(const char *why)
{
    Serial.printf("[apriltag_follower] FATAL: %s\n", why);
    // command_sender is never started on a fatal init error, so the
    // receiver stays in failsafe with the motor driver disabled.
    while (true) { delay(1000); }
}

void setup()
{
    Serial.begin(115200);
    delay(300);
    Serial.println();
    Serial.println("[apriltag_follower] starting up...");

    if (!psramFound())
    {
        Serial.println("[apriltag_follower] WARNING: no PSRAM detected — select "
                       "\"PSRAM: OPI PSRAM\" in the board menu (N16R8 has octal PSRAM). "
                       "Camera + detector buffers will not fit without it.");
    }

    // Servo first: it claims LEDC timer 1 explicitly, the camera then takes timer 0.
    if (!tilt_servo_init())   halt("tilt_servo_init failed");
    Serial.printf("[apriltag_follower] servo OK on GPIO%d, search tilt %.1f deg\n",
                  SERVO_PIN, tilt_servo_search_tilt());

    if (!camera_init())       halt("camera_init failed (check pins / PSRAM / frame size)");
    Serial.printf("[apriltag_follower] camera OK %dx%d grayscale\n", FRAME_WIDTH, FRAME_HEIGHT);

    if (!tag_detector_init()) halt("tag_detector_init failed");
    Serial.println("[apriltag_follower] detector OK (tag36h11)");

    range_model_init();
    tracker_control_init();

    if (!espnow_tx_init())    halt("espnow_tx_init failed");
    Serial.println("[apriltag_follower] ESP-NOW OK, receiver peer registered");

    if (!command_sender_init()) halt("command_sender_init failed");
    Serial.printf("[apriltag_follower] sender task running at %d Hz\n", 1000 / SEND_INTERVAL_MS);

#if !FOCAL_IS_CALIBRATED
    Serial.printf("[apriltag_follower] NOTE: FOCAL_LENGTH_PX=%.0f is an estimate, not calibrated. "
                  "Forward PWM capped at %d until FOCAL_IS_CALIBRATED=1 (see model/README.md).\n",
                  (double)FOCAL_LENGTH_PX, UNCALIBRATED_PWM_CAP);
#endif
    Serial.printf("[apriltag_follower] tag %.1f cm, stop %.0f cm / resume %.0f cm / slow %.0f cm\n",
                  (double)REAL_TAG_SIZE_CM, (double)MIN_FOLLOW_DISTANCE_CM,
                  (double)RESUME_DISTANCE_CM, (double)SLOW_DISTANCE_CM);
}

void loop()
{
    static uint32_t lastTelemetryMs = 0;
    static uint32_t frames = 0;

    GrayFrame frame;
    if (!camera_capture(frame))
    {
        Serial.println("[apriltag_follower] camera_capture failed, skipping frame");
        delay(10);
        return;
    }

    TagDetection det = tag_detector_detect(frame);
    camera_release(frame);   // hand the buffer back as soon as detection is over
    frames++;

    float tilt = tilt_servo_get_tilt();
    RangeEstimate range = range_model_update(det, tilt);
    DriveCommand cmd = tracker_control_update(det, range);
    // Motor driver stays in standby until the first tag is ever seen.
    command_sender_set(cmd, tracker_control_state() != TRACKER_IDLE);

    // --- Tilt servo: keep the tag vertically centred -----------------------
    if (det.found)
    {
        float err = range.in_frame_elev_deg;             // + = tag above centre
        if (fabsf(err) > TILT_DEADZONE_DEG)
        {
            tilt_servo_set_target(tilt + TILT_KP * err);
        }
    }
    else if (tracker_control_state() == TRACKER_LOST_STOP ||
             tracker_control_state() == TRACKER_IDLE)
    {
        tilt_servo_set_target(tilt_servo_search_tilt());
    }
    tilt_servo_update();

    // --- Telemetry ----------------------------------------------------------
    uint32_t now = millis();
    if (now - lastTelemetryMs >= TELEMETRY_INTERVAL_MS)
    {
        lastTelemetryMs = now;
        int16_t outL, outR;
        command_sender_get_output(outL, outR);

        if (det.found)
        {
            Serial.printf("[tag %d] px=(%.0f,%.0f) size=%.1f margin=%.0f | "
                          "slant=%.0fcm bear=%+.1f elev=%+.1f tilt=%.1f ground=%.0fcm(f=%.0f) h=%.0fcm | "
                          "%s cmd L=%d R=%d out L=%d R=%d %s | det=%lums\n",
                          det.id, (double)det.cx, (double)det.cy, (double)det.size_px,
                          (double)det.decision_margin,
                          (double)range.slant_cm, (double)range.bearing_deg,
                          (double)range.elevation_deg, (double)tilt,
                          (double)range.ground_cm, (double)range.ground_filtered_cm,
                          (double)range.height_cm,
                          tracker_control_state_name(),
                          (int)cmd.leftPWM, (int)cmd.rightPWM, (int)outL, (int)outR,
                          command_sender_last_send_ok() ? "" : "(SEND FAILED)",
                          (unsigned long)det.detect_ms);
        }
        else
        {
            uint32_t lostMs = tracker_control_lost_for_ms();
            Serial.printf("[no tag] lost=%s%lums tilt=%.1f | %s cmd L=%d R=%d out L=%d R=%d %s | det=%lums\n",
                          (lostMs == UINT32_MAX) ? "never seen, " : "",
                          (unsigned long)((lostMs == UINT32_MAX) ? 0 : lostMs), (double)tilt,
                          tracker_control_state_name(),
                          (int)cmd.leftPWM, (int)cmd.rightPWM, (int)outL, (int)outR,
                          command_sender_last_send_ok() ? "" : "(SEND FAILED)",
                          (unsigned long)det.detect_ms);
        }
    }

#if CALIBRATION_LOG
    if (det.found)
    {
        // Paste these into a CSV next to the measured distance — see
        // model/fit_camera_model.py.
        Serial.printf("CAL,%.2f,%.1f,%.1f,%.1f\n",
                      (double)det.size_px, (double)det.cx, (double)det.cy, (double)tilt);
    }
#endif
}
