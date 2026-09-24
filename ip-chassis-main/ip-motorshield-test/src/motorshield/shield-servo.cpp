#include "shield-servo.h"
#include "../../config.h"

ShieldServo::ShieldServo(Pca9685 &pwm, uint8_t channel)
    : _pwm(pwm), _channel(channel),
      _minUs(servoDefaultMinUs), _maxUs(servoDefaultMaxUs),
      _maxAngleDeg(servoDefaultMaxAngleDeg) {}

void ShieldServo::setAngleRange(float maxDegrees)
{
    if (maxDegrees > 0.0f)
    {
        _maxAngleDeg = maxDegrees;
    }
}

void ShieldServo::setPulseRange(uint16_t minUs, uint16_t maxUs)
{
    _minUs = minUs;
    _maxUs = maxUs;
}

void ShieldServo::setAngle(float degrees)
{
    setPercent(degrees / _maxAngleDeg * 100.0f);
}

void ShieldServo::setPercent(float percent)
{
    percent = constrain(percent, 0.0f, 100.0f);
    writeMicroseconds(_minUs + (uint16_t)lroundf(percent / 100.0f * (_maxUs - _minUs)));
}

void ShieldServo::writeMicroseconds(uint16_t pulseUs)
{
    _pwm.setMicroseconds(_channel, constrain(pulseUs, _minUs, _maxUs));
}

void ShieldServo::disable()
{
    _pwm.setDuty(_channel, 0);
}
