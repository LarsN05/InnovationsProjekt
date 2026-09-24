#include "mcp23017.h"

namespace
{
    // BANK=0 register map (POR default)
    constexpr uint8_t REG_IODIRA = 0x00;
    constexpr uint8_t REG_IODIRB = 0x01;
    constexpr uint8_t REG_GPPUB = 0x0D;
    constexpr uint8_t REG_GPIOB = 0x13;
    constexpr uint8_t REG_OLATA = 0x14;
    constexpr uint8_t REG_OLATB = 0x15;

    constexpr uint8_t USER_PIN_MASK = 0x7F; // GPB0-6; bit 7 is never user-controlled
}

Mcp23017::Mcp23017(uint8_t i2cAddr, TwoWire &wire) : _addr(i2cAddr), _wire(wire) {}

bool Mcp23017::begin()
{
    _wire.beginTransmission(_addr);
    if (_wire.endTransmission() != 0)
    {
        return false;
    }

    write8(REG_OLATA, 0x00);
    write8(REG_OLATB, _olatB);
    write8(REG_IODIRA, 0x00);    // GPA7 unbonded -> all of port A as output
    write8(REG_IODIRB, _iodirB); // GPB0-6 input, GPB7 unbonded -> output
    write8(REG_GPPUB, _gppuB);
    return true;
}

void Mcp23017::pinMode(uint8_t pin, uint8_t mode)
{
    uint8_t bit = (1 << pin) & USER_PIN_MASK;
    if (bit == 0)
    {
        return;
    }

    if (mode == OUTPUT)
    {
        _iodirB &= ~bit;
    }
    else
    {
        _iodirB |= bit;
    }

    if (mode == INPUT_PULLUP)
    {
        _gppuB |= bit;
    }
    else
    {
        _gppuB &= ~bit;
    }

    write8(REG_IODIRB, _iodirB);
    write8(REG_GPPUB, _gppuB);
}

void Mcp23017::digitalWrite(uint8_t pin, uint8_t value)
{
    uint8_t bit = (1 << pin) & USER_PIN_MASK;
    if (bit == 0)
    {
        return;
    }

    if (value)
    {
        _olatB |= bit;
    }
    else
    {
        _olatB &= ~bit;
    }
    write8(REG_OLATB, _olatB);
}

int Mcp23017::digitalRead(uint8_t pin)
{
    uint8_t bit = (1 << pin) & USER_PIN_MASK;
    if (bit == 0)
    {
        return LOW;
    }
    return (read8(REG_GPIOB) & bit) ? HIGH : LOW;
}

void Mcp23017::write8(uint8_t reg, uint8_t value)
{
    _wire.beginTransmission(_addr);
    _wire.write(reg);
    _wire.write(value);
    _wire.endTransmission();
}

uint8_t Mcp23017::read8(uint8_t reg)
{
    _wire.beginTransmission(_addr);
    _wire.write(reg);
    _wire.endTransmission();
    _wire.requestFrom(_addr, (uint8_t)1);
    return _wire.read();
}
