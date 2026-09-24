#pragma once
#include <Arduino.h>
#include <Wire.h>

// Minimal driver for the TI ADS1015 12-bit I2C ADC.
// Blocking single-ended reads at 3300 SPS and the fixed PGA range of
// +/-4.096 V (LSB = 2 mV), which fits all signals on this board.
class Ads1015
{
public:
    explicit Ads1015(uint8_t i2cAddr, TwoWire &wire = Wire);

    // Configures the chip in single-shot power-down mode and verifies the
    // configuration by reading it back. Returns false on an I2C error or if
    // the register does not contain the expected value.
    bool begin();

    // Starts a one-shot conversion on AINx vs GND and waits for it.
    // Returns 0..2047 counts, or -1 on I2C error / timeout.
    int16_t readSingleEnded(uint8_t channel);

    // One-shot voltage at the ADC input pin (counts * 2 mV).
    // Returns NAN on error.
    float readVoltage(uint8_t channel);

    // Average of several samples from AINx vs GND. For sampleCount > 1 the
    // ADC runs continuously while this blocking call collects the samples,
    // then returns to single-shot power-down mode. sampleCount <= 1 uses one
    // single-shot conversion. Returns NAN on error.
    float readAverageVoltage(uint8_t channel, uint16_t sampleCount);

private:
    bool powerDown(uint8_t channel);
    bool writeReg16(uint8_t reg, uint16_t value);
    bool readReg16(uint8_t reg, uint16_t &value);

    uint8_t _addr;
    TwoWire &_wire;
};
