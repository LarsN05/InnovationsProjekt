#pragma once
#include <Arduino.h>
#include <Wire.h>

// Minimal driver for the TI ADS1015 12-bit I2C ADC.
// Blocking single-shot single-ended reads at the fixed PGA range of
// +/-4.096 V (LSB = 2 mV), which fits all signals on this board.
class Ads1015
{
public:
    explicit Ads1015(uint8_t i2cAddr, TwoWire &wire = Wire);

    // Probes the chip. Returns false if it does not ACK its address.
    bool begin();

    // Starts a conversion on AINx vs GND and waits for it (~1 ms at 1600 SPS).
    // Returns 0..2047 counts, or -1 on I2C error / timeout.
    int16_t readSingleEnded(uint8_t channel);

    // Voltage at the ADC input pin (counts * 2 mV). Returns NAN on error.
    float readVoltage(uint8_t channel);

private:
    bool writeReg16(uint8_t reg, uint16_t value);
    uint16_t readReg16(uint8_t reg);

    uint8_t _addr;
    TwoWire &_wire;
};
