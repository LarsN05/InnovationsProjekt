#pragma once
#include <Arduino.h>
#include <Wire.h>

// Minimal driver for the Microchip MCP23017 16-bit I2C GPIO expander.
// On PCB V3 only GPB0-6 are wired to the user header. GPA0-6 are not
// connected, and GPA7/GPB7 are not bonded out on this die revision and must
// be configured as outputs (done unconditionally in begin()).
class Mcp23017
{
public:
    explicit Mcp23017(uint8_t i2cAddr, TwoWire &wire = Wire);

    // Probes the chip and configures safe directions (GPA all output,
    // GPB0-6 input, GPB7 output). Returns false if the chip does not ACK.
    bool begin();

    // pin = 0..6, mapped to GPB0..GPB6. mode: INPUT, INPUT_PULLUP or OUTPUT.
    void pinMode(uint8_t pin, uint8_t mode);
    void digitalWrite(uint8_t pin, uint8_t value);
    int digitalRead(uint8_t pin);

private:
    void write8(uint8_t reg, uint8_t value);
    uint8_t read8(uint8_t reg);

    uint8_t _addr;
    TwoWire &_wire;
    // Register shadows so read-modify-write needs no I2C reads.
    uint8_t _iodirB = 0x7F;
    uint8_t _gppuB = 0x00;
    uint8_t _olatB = 0x00;
};
