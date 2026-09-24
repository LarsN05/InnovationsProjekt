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
    // User pins, mapped to GPB0..GPB6.
    enum class Pin : uint8_t
    {
        B0 = 0,
        B1 = 1,
        B2 = 2,
        B3 = 3,
        B4 = 4,
        B5 = 5,
        B6 = 6,
    };

    static constexpr uint8_t pinMask(Pin pin) { return 1 << static_cast<uint8_t>(pin); }

    explicit Mcp23017(uint8_t i2cAddr, TwoWire &wire = Wire);

    // Configures safe directions (GPA all output, GPB0-6 input, GPB7 output)
    // and reads the configuration back. Returns false on an I2C error or if
    // any register does not contain the expected value.
    bool begin();

    // pin: B0..B6, mapped to GPB0..GPB6. mode: INPUT, INPUT_PULLUP or OUTPUT.
    void pinMode(Pin pin, uint8_t mode);
    void digitalWrite(Pin pin, uint8_t value);
    int digitalRead(Pin pin);

    // Configures global interrupt behavior via IOCON bits.
    // mirror: true = INTB mirrors INTA, false = INTA and INTB are independent.
    // openDrain: true = interrupt outputs are open-drain.
    // polarity: HIGH = active-high, LOW = active-low.
    // Defaults: mirror=true, openDrain=false, polarity=LOW.
    void setupInterrupts(bool mirror = true, bool openDrain = false, uint8_t polarity = LOW);

    // Configures one user pin (B0..B6 => GPB0..GPB6) as interrupt source.
    // mode: CHANGE, LOW or HIGH. Default: CHANGE.
    void setupInterruptPin(Pin pin, uint8_t mode = CHANGE);

    // Reads captured GPIOB value (INTCAPB), which also clears interrupt state.
    uint8_t getCapturedInterrupt();

    // Reads active interrupt flags for GPIOB (INTFB).
    uint8_t getInterruptFlags();

private:
    bool write8(uint8_t reg, uint8_t value);
    bool read8(uint8_t reg, uint8_t &value);

    uint8_t _addr;
    TwoWire &_wire;
    // Register shadows so read-modify-write needs no I2C reads.
    uint8_t _iodirB = 0x7F;
    uint8_t _gppuB = 0x00;
    uint8_t _olatB = 0x00;
    uint8_t _gpintenB = 0x00;
    uint8_t _defvalB = 0x00;
    uint8_t _intconB = 0x00;
    uint8_t _iocon = 0x00;
};
