#include "ads1015.h"

namespace
{
    constexpr uint8_t REG_CONVERSION = 0x00;
    constexpr uint8_t REG_CONFIG = 0x01;

    constexpr uint16_t CFG_OS_START = 0x8000;   // write: start single conversion; read: 1 = idle
    constexpr uint16_t CFG_MUX_SINGLE = 0x4000; // AINx vs GND, channel in bits [13:12]
    constexpr uint16_t CFG_PGA_4_096V = 0x0200; // LSB = 2 mV
    constexpr uint16_t CFG_MODE_SINGLE = 0x0100;
    constexpr uint16_t CFG_DR_1600SPS = 0x0080; // ~625 us conversion
    constexpr uint16_t CFG_COMP_DISABLE = 0x0003;

    constexpr float LSB_VOLTS = 4.096f / 2048.0f;
    constexpr uint32_t CONVERSION_TIMEOUT_US = 5000;
}

Ads1015::Ads1015(uint8_t i2cAddr, TwoWire &wire) : _addr(i2cAddr), _wire(wire) {}

bool Ads1015::begin()
{
    _wire.beginTransmission(_addr);
    return _wire.endTransmission() == 0;
}

int16_t Ads1015::readSingleEnded(uint8_t channel)
{
    uint16_t config = CFG_OS_START | CFG_MUX_SINGLE | ((uint16_t)(channel & 0x03) << 12) |
                      CFG_PGA_4_096V | CFG_MODE_SINGLE | CFG_DR_1600SPS | CFG_COMP_DISABLE;
    if (!writeReg16(REG_CONFIG, config))
    {
        return -1;
    }

    uint32_t start = micros();
    while ((readReg16(REG_CONFIG) & CFG_OS_START) == 0)
    {
        if (micros() - start > CONVERSION_TIMEOUT_US)
        {
            return -1;
        }
        delayMicroseconds(100);
    }

    // Conversion result is left-justified 12-bit; single-ended is 0..2047.
    return (int16_t)readReg16(REG_CONVERSION) >> 4;
}

float Ads1015::readVoltage(uint8_t channel)
{
    int16_t counts = readSingleEnded(channel);
    if (counts < 0)
    {
        return NAN;
    }
    return counts * LSB_VOLTS;
}

bool Ads1015::writeReg16(uint8_t reg, uint16_t value)
{
    _wire.beginTransmission(_addr);
    _wire.write(reg);
    _wire.write(value >> 8);
    _wire.write(value & 0xFF);
    return _wire.endTransmission() == 0;
}

uint16_t Ads1015::readReg16(uint8_t reg)
{
    _wire.beginTransmission(_addr);
    _wire.write(reg);
    _wire.endTransmission();
    _wire.requestFrom(_addr, (uint8_t)2);
    uint16_t value = (uint16_t)_wire.read() << 8;
    value |= _wire.read();
    return value;
}
