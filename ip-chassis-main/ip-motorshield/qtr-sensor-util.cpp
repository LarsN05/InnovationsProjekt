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
    // Each sweep lasts a fixed time: qtr.calibrate() is much faster on a bright surface,
    // so counting iterations made the sweeps too short to cross the line.
    const uint32_t sweepMs = 600;
    for (int16_t turn = 0; turn <= nTurns; turn++)
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

        // first sweep is half as long, so the robot oscillates around its start position
        const uint32_t durationMs = (turn == 0) ? sweepMs / 2 : sweepMs;
        const uint32_t sweepStartMs = millis();
        while (millis() - sweepStartMs < durationMs)
        {
            qtr.calibrate();
        }
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

    // Fine centering: the coarse return move stops as soon as the middle sensor touches the line,
    // and the robot overshoots. Turn in short pulses towards the line until it is under the middle sensor.
    // Same direction rule as the line following PID: position < 2000 -> turn right, > 2000 -> turn left.
    // Skipped if the middle sensor saw no difference between line and floor during calibration
    // (sensor bar not working): the robot would otherwise keep turning for the whole timeout.
    const bool middleSensorCalibrated = qtr.calibrationOn.maximum != nullptr && qtr.calibrationOn.minimum != nullptr &&
                                        (qtr.calibrationOn.maximum[2] - qtr.calibrationOn.minimum[2]) > 100;
    if (!middleSensorCalibrated)
    {
        Serial.println("Calibration error: the middle sensor did not see a line (min = max). Check the sensor bar power (VCC/GND), CTRL and the signal wires. Centering skipped.");
        blinkWarningPattern(255, 0, 0, 5);
    }
    else
    {
        delay(200);
        const int center = QTR_LINE_MAX_VALUE / 2;
        const uint32_t centeringStartMs = millis();
        uint8_t inToleranceReads = 0;
        while (millis() - centeringStartMs < calibrationCenterTimeoutMs)
        {
            const int position = qtr.readLineBlack(sensorValues);
            const int error = position - center;
            if (abs(error) < calibrationCenterTolerance)
            {
                if (++inToleranceReads >= 3)
                    break; // stable in the middle
                delay(calibrationCenterSettleMs);
                continue;
            }
            inToleranceReads = 0;
            const int dir = (error < 0) ? 1 : -1; // +1 = turn right
            motorL.setSpeed(dir * calibrationCenterSpeed);
            motorR.setSpeed(-dir * calibrationCenterSpeed);
            delay(calibrationCenterPulseMs);
            motorL.brake();
            motorR.brake();
            delay(calibrationCenterSettleMs);
        }
        motorL.brake();
        motorR.brake();
        if (inToleranceReads < 3)
        {
            Serial.println("Calibration warning: could not center the robot on the line (timeout).");
        }
    }

    delay(300);

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
