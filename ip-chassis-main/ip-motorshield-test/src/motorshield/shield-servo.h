#pragma once
#include <Arduino.h>
#include "../drivers/pca9685.h"

// One hobby servo on a channel of the 50 Hz servo PCA9685.
//
// The pulse range (default 500-2500 us) is mapped linearly onto the servo's
// mechanical travel, which defaults to 0..270 degrees. For servos with a
// different travel, change the mapping with setAngleRange(), e.g.
// setAngleRange(180) for a standard 180-degree servo.
class ShieldServo
{
public:
    ShieldServo(Pca9685 &pwm, uint8_t channel);

    // Mechanical travel in degrees that the full pulse range corresponds to.
    void setAngleRange(float maxDegrees);

    // Pulse widths mapped to 0 degrees / full travel. Narrow the range if a
    // servo buzzes against its mechanical endstops.
    void setPulseRange(uint16_t minUs, uint16_t maxUs);

    void setAngle(float degrees);  // 0..maxDegrees (default 0..270)
    void setPercent(float percent); // 0..100% of the full travel
    void writeMicroseconds(uint16_t pulseUs); // clamped to the pulse range
    void disable(); // stop sending pulses; servo goes limp

private:
    Pca9685 &_pwm;
    uint8_t _channel;
    uint16_t _minUs;
    uint16_t _maxUs;
    float _maxAngleDeg;
};
