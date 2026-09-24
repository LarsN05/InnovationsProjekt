#include "mcp23017.h"

namespace
{
    // BANK=0 register map (POR default)
    constexpr uint8_t REG_IODIRA = 0x00;
    constexpr uint8_t REG_IODIRB = 0x01;
    constexpr uint8_t REG_GPINTENB = 0x05;
    constexpr uint8_t REG_DEFVALB = 0x07;
    constexpr uint8_t REG_INTCONB = 0x09;
    constexpr uint8_t REG_IOCONA = 0x0A;
    constexpr uint8_t REG_IOCONB = 0x0B;
    constexpr uint8_t REG_GPPUB = 0x0D;
    constexpr uint8_t REG_INTFB = 0x0F;
    constexpr uint8_t REG_INTCAPB = 0x11;
    constexpr uint8_t REG_GPIOB = 0x13;
    constexpr uint8_t REG_OLATA = 0x14;
    constexpr uint8_t REG_OLATB = 0x15;

    constexpr uint8_t USER_PIN_MASK = 0x7F; // GPB0-6; bit 7 is never user-controlled
}

Mcp23017::Mcp23017(uint8_t i2cAddr, TwoWire &wire) : _addr(i2cAddr), _wire(wire) {}

bool Mcp23017::begin()
{
    struct RegisterValue
    {
        uint8_t reg;
        uint8_t value;
    };

    const RegisterValue configuration[] = {
        {REG_OLATA, 0x00},
        {REG_OLATB, _olatB},
        {REG_IODIRA, 0x00}, // GPA7 unbonded -> all of port A as output
        {REG_IODIRB, _iodirB}, // GPB0-6 input, GPB7 unbonded -> output
        {REG_GPPUB, _gppuB},
        {REG_GPINTENB, _gpintenB},
        {REG_DEFVALB, _defvalB},
        {REG_INTCONB, _intconB},
        {REG_IOCONA, _iocon},
        {REG_IOCONB, _iocon},
    };

    for (const RegisterValue &entry : configuration)
    {
        if (!write8(entry.reg, entry.value))
        {
            return false;
        }
    }

    // verify all registers were written correctly, which should fail in case the device is not present or not responding
    for (const RegisterValue &entry : configuration)
    {
        uint8_t value = 0;
        if (!read8(entry.reg, value) || value != entry.value)
        {
            return false;
        }
    }

    return true;
}

void Mcp23017::pinMode(Pin pin, uint8_t mode)
{
    uint8_t bit = pinMask(pin) & USER_PIN_MASK;
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

void Mcp23017::digitalWrite(Pin pin, uint8_t value)
{
    uint8_t bit = pinMask(pin) & USER_PIN_MASK;
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

int Mcp23017::digitalRead(Pin pin)
{
    uint8_t bit = pinMask(pin) & USER_PIN_MASK;
    if (bit == 0)
    {
        return LOW;
    }
    uint8_t value = 0;
    return (read8(REG_GPIOB, value) && (value & bit)) ? HIGH : LOW;
}

void Mcp23017::setupInterrupts(bool mirror, bool openDrain, uint8_t polarity)
{
    constexpr uint8_t IOCON_MIRROR = 1 << 6;
    constexpr uint8_t IOCON_ODR = 1 << 2;
    constexpr uint8_t IOCON_INTPOL = 1 << 1;

    uint8_t iocon = _iocon & ~(IOCON_MIRROR | IOCON_ODR | IOCON_INTPOL);
    if (mirror)
    {
        iocon |= IOCON_MIRROR;
    }
    if (openDrain)
    {
        iocon |= IOCON_ODR;
    }
    if (polarity == HIGH)
    {
        iocon |= IOCON_INTPOL;
    }

    _iocon = iocon;
    write8(REG_IOCONA, _iocon);
    write8(REG_IOCONB, _iocon);
}

void Mcp23017::setupInterruptPin(Pin pin, uint8_t mode)
{
    uint8_t bit = pinMask(pin) & USER_PIN_MASK;
    if (bit == 0)
    {
        return;
    }

    // Interrupt-capable user pins are inputs.
    _iodirB |= bit;
    write8(REG_IODIRB, _iodirB);

    _gpintenB |= bit;

    if (mode == CHANGE)
    {
        // Compare against previous pin state.
        _intconB &= ~bit;
    }
    else
    {
        // Compare against DEFVALB.
        _intconB |= bit;
        if (mode == LOW)
        {
            // Trigger while pin is low: low differs from DEFVAL=1.
            _defvalB |= bit;
        }
        else
        {
            // Treat HIGH and unknown modes as high-level trigger.
            _defvalB &= ~bit;
        }
    }

    write8(REG_DEFVALB, _defvalB);
    write8(REG_INTCONB, _intconB);
    write8(REG_GPINTENB, _gpintenB);
}

uint8_t Mcp23017::getCapturedInterrupt()
{
    uint8_t value = 0;
    return read8(REG_INTCAPB, value) ? value & USER_PIN_MASK : 0;
}

uint8_t Mcp23017::getInterruptFlags()
{
    uint8_t value = 0;
    return read8(REG_INTFB, value) ? value & USER_PIN_MASK : 0;
}

bool Mcp23017::write8(uint8_t reg, uint8_t value)
{
    _wire.beginTransmission(_addr);
    _wire.write(reg);
    _wire.write(value);
    return _wire.endTransmission() == 0;
}

bool Mcp23017::read8(uint8_t reg, uint8_t &value)
{
    _wire.beginTransmission(_addr);
    _wire.write(reg);
    if (_wire.endTransmission(false) != 0)
    {
        return false;
    }

    if (_wire.requestFrom(_addr, static_cast<uint8_t>(1)) != 1 || !_wire.available())
    {
        return false;
    }

    int received = _wire.read();
    if (received < 0)
    {
        return false;
    }

    value = static_cast<uint8_t>(received);
    return true;
}
