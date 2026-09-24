#pragma once
#include <Arduino.h>
#include "../drivers/pca9685.h"
#include "../drivers/ads1015.h"

// PWM decay mode while the H-bridge is modulating, per DRV8251A datasheet Table 8-2:
// Brake = slow decay (inactive input held high), 
// Coast = fast decay (inactive input held low, outputs Hi-Z during the off-time).
enum class DecayMode
{
    Brake,
    Coast
};

// One DC motor behind a DRV8251A H-bridge. Speed is signed 12-bit:
// -4095 (full reverse) .. 4095 (full forward), 0 = coast.
// Subclasses only translate duty cycles to their PWM hardware.
class Motor
{
public:
    static constexpr int kMaxSpeed = 4095;

    virtual ~Motor() = default;

    void setSpeed(int speedSigned);
    void setDecayMode(DecayMode mode) { _decayMode = mode; }
    void stop();  // both inputs low -> outputs Hi-Z, motor coasts
    void brake(); // both inputs high -> motor windings shorted, active brake
    int speed() const { return _speed; }

    // Wires up the ADS1015 channel that measures this motor's IPROPI sense
    // voltage (done by Motorshield during construction).
    void attachCurrentSense(Ads1015 &adc, uint8_t adcChannel);

    // Motor current from the DRV8251A IPROPI sense voltage. Blocks ~1 ms for
    // the ADC conversion; NAN if no sense is attached or on ADC error.
    float readCurrentAmps();

protected:
    virtual void writePins(uint16_t dutyA, uint16_t dutyB) = 0;

private:
    DecayMode _decayMode = DecayMode::Brake;
    int _speed = 0;
    Ads1015 *_senseAdc = nullptr;
    uint8_t _senseChannel = 0;
};

// Motors 1 & 2: DRV8251A inputs on ESP32 GPIOs, PWM via LEDC.
class GpioMotor : public Motor
{
public:
    GpioMotor(uint8_t pinA, uint8_t pinB);
    void begin();

protected:
    void writePins(uint16_t dutyA, uint16_t dutyB) override;

private:
    uint8_t _pinA;
    uint8_t _pinB;
};

// Motors 3-6: DRV8251A inputs on PCA9685 PWM expander channels.
class Pca9685Motor : public Motor
{
public:
    Pca9685Motor(Pca9685 &pwm, uint8_t channelA, uint8_t channelB);

protected:
    void writePins(uint16_t dutyA, uint16_t dutyB) override;

private:
    Pca9685 &_pwm;
    uint8_t _chA;
    uint8_t _chB;
};
