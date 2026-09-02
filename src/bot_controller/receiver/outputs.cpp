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
    if (!packet.armed)
    {
        outputs_failsafe();
        Serial.println("armed=0 L=0 R=0");
        return;
    }

    digitalWrite(PIN_STBY, HIGH);   // enable driver

    int16_t left  = packet.leftPWM;
    int16_t right = packet.rightPWM;

    digitalWrite(PIN_AIN1, left  >= 0 ? HIGH : LOW);
    digitalWrite(PIN_AIN2, left  >= 0 ? LOW  : HIGH);
    digitalWrite(PIN_BIN1, right >= 0 ? HIGH : LOW);
    digitalWrite(PIN_BIN2, right >= 0 ? LOW  : HIGH);

    uint8_t leftDuty  = (uint8_t)min((int)abs(left),  255);
    uint8_t rightDuty = (uint8_t)min((int)abs(right), 255);

    ledcWrite(PIN_PWMA, leftDuty);
    ledcWrite(PIN_PWMB, rightDuty);

    Serial.printf("armed=1 L=%4d R=%4d\n", (int)left, (int)right);
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
