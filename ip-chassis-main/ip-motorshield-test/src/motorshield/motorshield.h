#pragma once
#include <Arduino.h>
#include <Wire.h>
#include "../../pinout.h"
#include "../../config.h"
#include "../drivers/pca9685.h"
#include "../drivers/ads1015.h"
#include "../drivers/mcp23017.h"
#include "motor.h"
#include "shield-servo.h"

// Facade over the IP Motorshield PCB V3 hardware: hands out the 6 DC motors
// and 8 servos and hides whether they sit on ESP32 GPIOs or behind the I2C
// expanders (2x PCA9685, 2x ADS1015, 1x MCP23017).
class Motorshield
{
public:
    Motorshield();

    // Brings up the I2C bus and all five expander chips plus the LEDC PWM
    // for motors 1 & 2. Logs each missing chip on Serial; returns false if
    // any chip did not respond.
    bool begin();

    Motor &motor(uint8_t n);      // n = 1..6
    ShieldServo &servo(uint8_t n); // n = 1..8
    Mcp23017 &gpio();              // user pins GPB0-6

    // Averaged battery voltage via ADS1015 #2. Returns NAN on ADC error.
    float batteryVoltage(uint8_t samples = 1);
    // 5V servo supply rail voltage. Returns NAN on ADC error.
    float servoRailVoltage(uint8_t samples = 1);

private:
    Pca9685 _motorPwm;
    Pca9685 _servoPwm;
    Ads1015 _adcMotors14; // Isense motors 1-4
    Ads1015 _adcPower56;  // battery, 5V rail, Isense motors 5-6
    Mcp23017 _mcp;

    GpioMotor _motor1;
    GpioMotor _motor2;
    Pca9685Motor _motor3;
    Pca9685Motor _motor4;
    Pca9685Motor _motor5;
    Pca9685Motor _motor6;
    Motor *_motors[6];

    ShieldServo _servos[8];
};
