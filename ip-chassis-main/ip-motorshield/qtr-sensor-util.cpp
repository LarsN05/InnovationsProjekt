#include "config.h"
#include "pinout.h"
#include "qtr-sensor-util.h"
#include "onboard-led.h"

#include <Arduino.h>

void printSensorValues(QTRSensors &qtr)
{
    uint16_t sensorValues[QTR_SENSOR_COUNT];
    qtr.read(sensorValues);

    Serial.print("Sensor values (time from high to low): ");
    for (int i = 0; i < QTR_SENSOR_COUNT; i++)
    {
        Serial.print(sensorValues[i]);
        if (i < QTR_SENSOR_COUNT - 1)
        {
            Serial.print("μs, ");
        }
    }
    Serial.println("μs");
}

static const int calibrationTurnSpeed = 1024;

void turnRight(Motor &motorL, Motor &motorR)
{
    motorR.brake();
    motorL.brake();
    delay(100);
    motorR.setSpeed(-calibrationTurnSpeed);
    motorL.setSpeed(calibrationTurnSpeed);
}

void turnLeft(Motor &motorL, Motor &motorR)
{
    motorR.brake();
    motorL.brake();
    delay(100);
    motorR.setSpeed(calibrationTurnSpeed);
    motorL.setSpeed(-calibrationTurnSpeed);
}

/**
 * @brief Automatically calibrates the QTR-Sensors for line following by moving the robot around.
 */
void calibrateSensors(QTRSensors &qtr, Motor &motorL, Motor &motorR)
{
    bool isNextTurnLeft = true;
    int16_t nTurns = 4;
    int16_t iterPerTurn = 40;
    int16_t halfTurn = iterPerTurn / 2;
    int16_t totalIterations = (nTurns * iterPerTurn) + halfTurn;
    for (int16_t i = 0; i < totalIterations; i++)
    {
        if (i == 0 || (i - halfTurn) % iterPerTurn == 0)
        {
            if (isNextTurnLeft)
            {
                turnLeft(motorL, motorR);
                isNextTurnLeft = false;
            }
            else
            {
                turnRight(motorL, motorR);
                isNextTurnLeft = true;
            }
        }
        qtr.calibrate();
    }
    uint32_t startOfReturnMove = millis();
    if (isNextTurnLeft)
    {
        turnLeft(motorL, motorR);
    }
    else
    {
        turnRight(motorL, motorR);
    }
    uint16_t sensorValues[QTR_SENSOR_COUNT];
    qtr.readLineBlack(sensorValues);
    while ((sensorValues[2] < BLACKLINE_THRESHOLD) && (millis() - startOfReturnMove < 1000))
    {
        delay(10);
        qtr.readLineBlack(sensorValues);
    }
    motorL.brake();
    motorR.brake();

    if (millis() - startOfReturnMove >= 1000)
    {
        Serial.println("Calibration error: Robot did not find the line again after calibration. Please check the wiring and adjust the BLACKLINE_THRESHOLD if necessary.");
        while (HOLD_ON_CALIBRATION_FAILURE)
        {
            blinkWarningPattern(255, 0, 0, 1);
        }
    }

    delay(1000);

    uint16_t finalPosition = qtr.readLineBlack(sensorValues);
    Serial.print("Calibration final position: ");
    Serial.println(finalPosition);

    if (sensorValues[0] > 100 || sensorValues[4] > 100)
    {
        Serial.println("Calibration warning: outer sensors detect line after centering.");
    }

    auto printCalibrationArray = [&](const char *label, const uint16_t *values)
    {
        Serial.print(label);
        if (values == nullptr)
        {
            Serial.println("n/a");
            return;
        }
        for (uint8_t i = 0; i < QTR_SENSOR_COUNT; ++i)
        {
            Serial.print(values[i]);
            if (i < QTR_SENSOR_COUNT - 1)
            {
                Serial.print(", ");
            }
        }
        Serial.println();
    };

    auto printCalibration = [&](const char *title, const QTRSensors::CalibrationData &data)
    {
        Serial.println(title);
        if (!data.initialized)
        {
            Serial.println("  not initialized");
            return;
        }
        printCalibrationArray("  min: ", data.minimum);
        printCalibrationArray("  max: ", data.maximum);
    };

    printCalibration("Calibration data (emitters on):", qtr.calibrationOn);
    printCalibration("Calibration data (emitters off):", qtr.calibrationOff);
}
