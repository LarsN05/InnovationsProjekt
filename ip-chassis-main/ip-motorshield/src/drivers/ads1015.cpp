#include "ads1015.h"

namespace
{
    constexpr uint8_t REG_CONVERSION = 0x00;
    constexpr uint8_t REG_CONFIG = 0x01;

    constexpr uint16_t CFG_OS_START = 0x8000;   // write: start single conversion; read: 1 = idle
    constexpr uint16_t CFG_READBACK_MASK = 0x7FFF; // OS reads as live conversion status
    constexpr uint16_t CFG_MUX_SINGLE = 0x4000; // AINx vs GND, channel in bits [13:12]
    constexpr uint16_t CFG_PGA_4_096V = 0x0200; // LSB = 2 mV
    constexpr uint16_t CFG_MODE_SINGLE = 0x0100;
    constexpr uint16_t CFG_DR_3300SPS = 0x00C0; // DR[2:0] = 110, ~303 us conversion
    constexpr uint16_t CFG_COMP_DISABLE = 0x0003;

    constexpr uint16_t BASE_CONFIG = CFG_MUX_SINGLE | CFG_PGA_4_096V |
                                     CFG_DR_3300SPS | CFG_COMP_DISABLE;
    constexpr uint16_t SINGLE_SHOT_CONFIG = BASE_CONFIG | CFG_MODE_SINGLE;

    constexpr float LSB_VOLTS = 4.096f / 2048.0f;
    constexpr uint32_t CONVERSION_TIMEOUT_US = 5000;
    constexpr uint32_t CONVERSION_PERIOD_US = 304; // ceil(1 s / 3300 samples)

    uint16_t channelConfig(uint8_t channel)
    {
        return static_cast<uint16_t>((channel & 0x03) << 12);
    }
}

Ads1015::Ads1015(uint8_t i2cAddr, TwoWire &wire) : _addr(i2cAddr), _wire(wire) {}

bool Ads1015::begin()
{
    // Keep OS clear when writing so initialization does not start a conversion.
    // On reads OS reports live conversion status, so it is excluded from the
    // read-back comparison. In single-shot mode an idle device reads OS as 1.
    if (!writeReg16(REG_CONFIG, SINGLE_SHOT_CONFIG))
    {
        return false;
    }

    uint16_t config = 0;
    return readReg16(REG_CONFIG, config) &&
           (config & CFG_READBACK_MASK) == (SINGLE_SHOT_CONFIG & CFG_READBACK_MASK);
}

int16_t Ads1015::readSingleEnded(uint8_t channel)
{
    uint16_t config = CFG_OS_START | SINGLE_SHOT_CONFIG | channelConfig(channel);
    if (!writeReg16(REG_CONFIG, config))
    {
        return -1;
    }

    uint32_t start = micros();
    while (true)
    {
        uint16_t status = 0;
        if (!readReg16(REG_CONFIG, status))
        {
            return -1;
        }
        if (status & CFG_OS_START)
        {
            break;
        }
        if (micros() - start > CONVERSION_TIMEOUT_US)
        {
            return -1;
        }
        delayMicroseconds(100);
    }

    // Conversion result is left-justified 12-bit; single-ended is 0..2047.
    uint16_t conversion = 0;
    if (!readReg16(REG_CONVERSION, conversion))
    {
        return -1;
    }
    return (int16_t)conversion >> 4;
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

float Ads1015::readAverageVoltage(uint8_t channel, uint16_t sampleCount)
{
    if (sampleCount <= 1)
    {
        return readVoltage(channel);
    }

    // MODE = 0 starts continuous conversions. Wait one complete conversion
    // before every read so each value comes from a distinct ADC sample.
    uint16_t config = BASE_CONFIG | channelConfig(channel);
    if (!writeReg16(REG_CONFIG, config))
    {
        return NAN;
    }

    uint32_t sum = 0;
    bool readOk = true;
    uint32_t nextSampleAt = micros() + CONVERSION_PERIOD_US;
    for (uint16_t i = 0; i < sampleCount; ++i)
    {
        int32_t waitUs = static_cast<int32_t>(nextSampleAt - micros());
        if (waitUs > 0)
        {
            delayMicroseconds(static_cast<uint32_t>(waitUs));
        }

        uint16_t conversion = 0;
        if (!readReg16(REG_CONVERSION, conversion))
        {
            readOk = false;
            break;
        }

        int16_t counts = static_cast<int16_t>(conversion) >> 4;
        sum += static_cast<uint16_t>(counts > 0 ? counts : 0);
        nextSampleAt += CONVERSION_PERIOD_US;
    }

    // MODE = 1 returns the ADC to its low-power single-shot idle state.
    bool poweredDown = powerDown(channel);
    if (!readOk || !poweredDown)
    {
        return NAN;
    }

    return (static_cast<float>(sum) / sampleCount) * LSB_VOLTS;
}

bool Ads1015::powerDown(uint8_t channel)
{
    return writeReg16(REG_CONFIG, SINGLE_SHOT_CONFIG | channelConfig(channel));
}

bool Ads1015::writeReg16(uint8_t reg, uint16_t value)
{
    _wire.beginTransmission(_addr);
    _wire.write(reg);
    _wire.write(value >> 8);
    _wire.write(value & 0xFF);
    return _wire.endTransmission() == 0;
}

bool Ads1015::readReg16(uint8_t reg, uint16_t &value)
{
    _wire.beginTransmission(_addr);
    _wire.write(reg);
    if (_wire.endTransmission(false) != 0)
    {
        return false;
    }

    if (_wire.requestFrom(_addr, static_cast<uint8_t>(2)) != 2 || _wire.available() < 2)
    {
        return false;
    }

    int high = _wire.read();
    int low = _wire.read();
    if (high < 0 || low < 0)
    {
        return false;
    }

    value = (static_cast<uint16_t>(high) << 8) | static_cast<uint8_t>(low);
    return true;
}
