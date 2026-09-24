#include "motorshield.h"

#ifdef MCP23017_INTB_PIN
namespace
{
volatile bool g_mcp23017InterruptFlag = false;

void IRAM_ATTR onMcp23017IntbFallingEdge()
{
    g_mcp23017InterruptFlag = true;
}
}
#endif

Motorshield::Motorshield()
    : _motorPwm(PCA9685_MOTOR_I2C_ADDR),
      _servoPwm(PCA9685_SERVO_I2C_ADDR),
      _adcMotors14(ADS1015_1_I2C_ADDR),
      _adcPower56(ADS1015_2_I2C_ADDR),
      _mcp(MCP23017_I2C_ADDR),
      _motor1(MOTOR_1_A_PIN, MOTOR_1_B_PIN),
      _motor2(MOTOR_2_A_PIN, MOTOR_2_B_PIN),
      _motor3(_motorPwm, MOTOR_3_A_CHANNEL, MOTOR_3_B_CHANNEL),
      _motor4(_motorPwm, MOTOR_4_A_CHANNEL, MOTOR_4_B_CHANNEL),
      _motor5(_motorPwm, MOTOR_5_A_CHANNEL, MOTOR_5_B_CHANNEL),
      _motor6(_motorPwm, MOTOR_6_A_CHANNEL, MOTOR_6_B_CHANNEL),
      _motors{&_motor1, &_motor2, &_motor3, &_motor4, &_motor5, &_motor6},
      _servos{ShieldServo(_servoPwm, SERVO_1_CHANNEL), ShieldServo(_servoPwm, SERVO_2_CHANNEL),
              ShieldServo(_servoPwm, SERVO_3_CHANNEL), ShieldServo(_servoPwm, SERVO_4_CHANNEL),
              ShieldServo(_servoPwm, SERVO_5_CHANNEL), ShieldServo(_servoPwm, SERVO_6_CHANNEL),
              ShieldServo(_servoPwm, SERVO_7_CHANNEL), ShieldServo(_servoPwm, SERVO_8_CHANNEL)}
{
    _motor1.attachCurrentSense(_adcMotors14, ADS1015_1_ISENSE_MOTOR1_CH, motorCurrentAdcSamples);
    _motor2.attachCurrentSense(_adcMotors14, ADS1015_1_ISENSE_MOTOR2_CH, motorCurrentAdcSamples);
    _motor3.attachCurrentSense(_adcMotors14, ADS1015_1_ISENSE_MOTOR3_CH, motorCurrentAdcSamples);
    _motor4.attachCurrentSense(_adcMotors14, ADS1015_1_ISENSE_MOTOR4_CH, motorCurrentAdcSamples);
    _motor5.attachCurrentSense(_adcPower56, ADS1015_2_ISENSE_MOTOR5_CH, motorCurrentAdcSamples);
    _motor6.attachCurrentSense(_adcPower56, ADS1015_2_ISENSE_MOTOR6_CH, motorCurrentAdcSamples);
}

bool Motorshield::begin()
{
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(400000);

    bool ok = true;
    if (!_motorPwm.begin(pcaMotorPwmFreqHz))
    {
        Serial.printf("Motorshield: motor PCA9685 (0x%02X) not responding\n", PCA9685_MOTOR_I2C_ADDR);
        ok = false;
    }
    if (!_servoPwm.begin(pcaServoPwmFreqHz))
    {
        Serial.printf("Motorshield: servo PCA9685 (0x%02X) not responding\n", PCA9685_SERVO_I2C_ADDR);
        ok = false;
    }
    if (!_adcMotors14.begin())
    {
        Serial.printf("Motorshield: ADS1015 #1 (0x%02X) communication/configuration check failed\n",
                      ADS1015_1_I2C_ADDR);
        ok = false;
    }
    if (!_adcPower56.begin())
    {
        Serial.printf("Motorshield: ADS1015 #2 (0x%02X) communication/configuration check failed\n",
                      ADS1015_2_I2C_ADDR);
        ok = false;
    }
    if (!_mcp.begin())
    {
        Serial.printf("Motorshield: MCP23017 (0x%02X) communication/configuration check failed\n",
                      MCP23017_I2C_ADDR);
        ok = false;
    }
#ifdef MCP23017_INTB_PIN
    else
    {
        // Keep default active-low MCP interrupt output and catch assertion edge.
        _mcp.getCapturedInterrupt();
        pinMode(MCP23017_INTB_PIN, INPUT);
        attachInterrupt(digitalPinToInterrupt(MCP23017_INTB_PIN), onMcp23017IntbFallingEdge, FALLING);
    }
#endif

    _motor1.begin();
    _motor2.begin();
    return ok;
}

Motor &Motorshield::motor(uint8_t n)
{
    n = constrain(n, (uint8_t)1, (uint8_t)6);
    return *_motors[n - 1];
}

ShieldServo &Motorshield::servo(uint8_t n)
{
    n = constrain(n, (uint8_t)1, (uint8_t)8);
    return _servos[n - 1];
}

Mcp23017 &Motorshield::gpio()
{
    return _mcp;
}

bool Motorshield::consumeGpioInterruptFlag()
{
#ifdef MCP23017_INTB_PIN
    noInterrupts();
    bool wasSet = g_mcp23017InterruptFlag;
    g_mcp23017InterruptFlag = false;
    interrupts();
    return wasSet;
#else
    return false;
#endif
}

float Motorshield::batteryVoltage()
{
    return _adcPower56.readVoltage(ADS1015_2_BAT_MON_CH) *
           BATTERY_VOLTAGE_FROM_MONITOR_VOLTAGE_CONVERSION_FACTOR;
}

float Motorshield::servoRailVoltage()
{
    return _adcPower56.readVoltage(ADS1015_2_5V_SERVO_MON_CH) *
           SERVO_5V_VOLTAGE_FROM_MONITOR_VOLTAGE_CONVERSION_FACTOR;
}
