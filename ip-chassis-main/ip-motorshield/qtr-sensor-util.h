#pragma once
#include <QTRSensors.h>
#include "src/motorshield/motor.h"

void printSensorValues(QTRSensors &qtr);
void calibrateSensors(QTRSensors &qtr, Motor &motorL, Motor &motorR);
void turnLeft(Motor &motorL, Motor &motorR);
void turnRight(Motor &motorL, Motor &motorR);