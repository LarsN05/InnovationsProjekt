#include "motor.h"
#include "../../config.h"

static_assert(PWM_MAX_VALUE == Motor::kMaxSpeed, "Motor speed range expects 12-bit PWM resolution");

void Motor::setSpeed(int speedSigned)
{
    _speed = constrain(speedSigned, -kMaxSpeed, kMaxSpeed);
    uint16_t duty = abs(_speed);

    if (_speed == 0)
    {
        writePins(0, 0);
    }
    else if (_decayMode == DecayMode::Brake)
    {
        // Slow decay: active input held high, other input carries inverted duty,
        // so the off-time shorts the windings (both inputs high = brake).
        if (_speed > 0)
        {
            writePins(kMaxSpeed, kMaxSpeed - duty);
        }
        else
        {
            writePins(kMaxSpeed - duty, kMaxSpeed);
        }
    }
    else
    {
        // Fast decay: active input carries the duty, other input held low,
        // so the off-time leaves the outputs Hi-Z (both inputs low = coast).
        if (_speed > 0)
        {
            writePins(duty, 0);
        }
        else
        {
            writePins(0, duty);
        }
    }
}

void Motor::stop()
{
    _speed = 0;
    writePins(0, 0);
}

void Motor::brake()
{
    _speed = 0;
    writePins(kMaxSpeed, kMaxSpeed);
}

void Motor::attachCurrentSense(Ads1015 &adc, uint8_t adcChannel)
{
    _senseAdc = &adc;
    _senseChannel = adcChannel;
}

float Motor::readCurrentAmps()
{
    if (_senseAdc == nullptr)
    {
        return NAN;
    }
    return _senseAdc->readVoltage(_senseChannel) / motorIsenseVoltsPerAmp;
}

GpioMotor::GpioMotor(uint8_t pinA, uint8_t pinB) : _pinA(pinA), _pinB(pinB) {}

void GpioMotor::begin()
{
    ledcAttach(_pinA, pwmFreq, pwmResolution);
    ledcAttach(_pinB, pwmFreq, pwmResolution);
}

void GpioMotor::writePins(uint16_t dutyA, uint16_t dutyB)
{
    ledcWrite(_pinA, dutyA);
    ledcWrite(_pinB, dutyB);
}

Pca9685Motor::Pca9685Motor(Pca9685 &pwm, uint8_t channelA, uint8_t channelB)
    : _pwm(pwm), _chA(channelA), _chB(channelB) {}

void Pca9685Motor::writePins(uint16_t dutyA, uint16_t dutyB)
{
    _pwm.setDuty(_chA, dutyA);
    _pwm.setDuty(_chB, dutyB);
}
