#include "outputs.h"
#include <Arduino.h>

void outputs_init()
{
    pinMode(PIN_AIN1, OUTPUT);
    pinMode(PIN_AIN2, OUTPUT);
    pinMode(PIN_BIN1, OUTPUT);
    pinMode(PIN_BIN2, OUTPUT);
    pinMode(PIN_STBY, OUTPUT);

    ledcAttach(PIN_PWMA, PWM_FREQ_HZ, PWM_RESOLUTION);
    ledcAttach(PIN_PWMB, PWM_FREQ_HZ, PWM_RESOLUTION);

    // Go safe on boot - driver disabled, direction pins low, PWM at 0,
    // before anything ever gets a chance to drive an arbitrary value.
    outputs_failsafe();
}

void outputs_update(const ControlPacket &packet)
{
    // Disarmed always means stopped, regardless of link health. This
    // drops STBY - the driver chip itself goes to standby, not just
    // the PWM duty.
    if (!packet.armed)
    {
        outputs_failsafe();
        Serial.println("armed=0 L=0 R=0");
        return;
    }

    digitalWrite(PIN_STBY, HIGH);   // enable driver

    // Forward-only: direction pins fixed, PWM carries the speed.
    digitalWrite(PIN_AIN1, HIGH);
    digitalWrite(PIN_AIN2, LOW);
    digitalWrite(PIN_BIN1, HIGH);
    digitalWrite(PIN_BIN2, LOW);

    ledcWrite(PIN_PWMA, packet.leftPWM);
    ledcWrite(PIN_PWMB, packet.rightPWM);

    Serial.printf("armed=1 L=%3u R=%3u\n", packet.leftPWM, packet.rightPWM);
}

void outputs_failsafe()
{
    digitalWrite(PIN_STBY, LOW);    // disable driver chip entirely
    digitalWrite(PIN_AIN1, LOW);
    digitalWrite(PIN_AIN2, LOW);
    digitalWrite(PIN_BIN1, LOW);
    digitalWrite(PIN_BIN2, LOW);
    ledcWrite(PIN_PWMA, 0);
    ledcWrite(PIN_PWMB, 0);
}
