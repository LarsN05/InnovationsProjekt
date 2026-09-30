#pragma once

#include "shared-globals.h"
#include "qtr-sensor-util.h"
#include "onboard-led.h"
#include "config.h"

bool perform180Turn(QTRSensors &qtr, Motor &motorL, Motor &motorR);
bool allignOnCheckpoint(PIDController &pid, QTRSensors &qtr, Motor &motorL, Motor &motorR);