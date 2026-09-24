#include "pca9685.h"

namespace
{
    constexpr uint8_t REG_MODE1 = 0x00;
    constexpr uint8_t REG_LED0_ON_L = 0x06; // channel n: 0x06 + 4*n
    constexpr uint8_t REG_PRESCALE = 0xFE;

    constexpr uint8_t MODE1_RESTART = 0x80;
    constexpr uint8_t MODE1_AI = 0x20; // register auto-increment
    constexpr uint8_t MODE1_SLEEP = 0x10;

    constexpr float OSC_FREQ_HZ = 25000000.0f;
    constexpr uint16_t FULL_ON_OFF_BIT = 0x1000; // bit 4 of LEDn_ON_H / LEDn_OFF_H
}

Pca9685::Pca9685(uint8_t i2cAddr, TwoWire &wire) : _addr(i2cAddr), _wire(wire) {}

bool Pca9685::begin(float pwmFreqHz)
{
    _wire.beginTransmission(_addr);
    if (_wire.endTransmission() != 0)
    {
        return false;
    }

    reset(); // disable all PWM outputs at the beginning.
    write8(REG_MODE1, 0x00); // wake from POR sleep, defaults otherwise
    setPwmFreqHz(pwmFreqHz);
    return true;
}

void Pca9685::reset()
{
    for (uint8_t channel = 0; channel < 16; channel++)
    {
        setPwm(channel, 0, 0);
    }
}


void Pca9685::setPwmFreqHz(float freqHz)
{
    long prescale = lroundf(OSC_FREQ_HZ / (4096.0f * freqHz)) - 1;
    _prescale = constrain(prescale, 3L, 255L);

    // The prescaler is only writable while the oscillator is in sleep.
    uint8_t oldMode = read8(REG_MODE1);
    write8(REG_MODE1, (oldMode & ~MODE1_RESTART) | MODE1_SLEEP);
    write8(REG_PRESCALE, _prescale);
    write8(REG_MODE1, oldMode & ~MODE1_SLEEP);
    delayMicroseconds(500); // oscillator startup, required before RESTART
    write8(REG_MODE1, (oldMode & ~MODE1_SLEEP) | MODE1_RESTART | MODE1_AI);
}

float Pca9685::actualFreqHz() const
{
    return OSC_FREQ_HZ / (4096.0f * (_prescale + 1));
}

void Pca9685::setPwm(uint8_t channel, uint16_t on, uint16_t off)
{
    _wire.beginTransmission(_addr);
    _wire.write(REG_LED0_ON_L + 4 * channel);
    _wire.write(on & 0xFF);
    _wire.write(on >> 8);
    _wire.write(off & 0xFF);
    _wire.write(off >> 8);
    _wire.endTransmission();
}

void Pca9685::setDuty(uint8_t channel, uint16_t duty)
{
    if (duty == 0)
    {
        setPwm(channel, 0, FULL_ON_OFF_BIT);
    }
    else if (duty >= kMaxDuty)
    {
        setPwm(channel, FULL_ON_OFF_BIT, 0);
    }
    else
    {
        setPwm(channel, 0, duty);
    }
}

void Pca9685::setMicroseconds(uint8_t channel, uint16_t pulseUs)
{
    float ticks = pulseUs * actualFreqHz() * 4096.0f / 1000000.0f;
    setDuty(channel, (uint16_t)constrain(lroundf(ticks), 0L, (long)kMaxDuty));
}

void Pca9685::write8(uint8_t reg, uint8_t value)
{
    _wire.beginTransmission(_addr);
    _wire.write(reg);
    _wire.write(value);
    _wire.endTransmission();
}

uint8_t Pca9685::read8(uint8_t reg)
{
    _wire.beginTransmission(_addr);
    _wire.write(reg);
    _wire.endTransmission();
    _wire.requestFrom(_addr, (uint8_t)1);
    return _wire.read();
}
