#pragma once
#include <Arduino.h>
#include <Wire.h>

// Minimal driver for the NXP PCA9685 16-channel 12-bit PWM expander.
// Each instance can run its own PWM frequency (24 Hz .. ~1526 Hz, limited by
// the internal 25 MHz oscillator and the 8-bit prescaler).
class Pca9685
{
public:
    static constexpr uint16_t kMaxDuty = 4095;

    explicit Pca9685(uint8_t i2cAddr, TwoWire &wire = Wire);

    // Probes the chip and programs the PWM frequency. Returns false if the
    // chip does not ACK its address.
    bool begin(float pwmFreqHz);

    // Resets the PCA. All channels are set to 0% duty cycle.
    void reset();

    void setPwmFreqHz(float freqHz);

    // Frequency resulting from the programmed prescaler; used for exact
    // microsecond-to-tick conversion.
    float actualFreqHz() const;

    // Raw write of the LEDn_ON/LEDn_OFF registers (single burst transaction).
    void setPwm(uint8_t channel, uint16_t on, uint16_t off);

    // Duty 0 uses the full-OFF special bit, >= 4095 the full-ON special bit,
    // so the output is a clean DC level instead of a 1-tick glitch.
    void setDuty(uint8_t channel, uint16_t duty);

    void setMicroseconds(uint8_t channel, uint16_t pulseUs);

private:
    void write8(uint8_t reg, uint8_t value);
    uint8_t read8(uint8_t reg);

    uint8_t _addr;
    TwoWire &_wire;
    uint8_t _prescale = 0;
};
