#include "movement-functions.h"
#include "shared-globals.h"
#include "qtr-sensor-util.h"
#include "onboard-led.h"
#include "config.h"

constexpr uint32_t minTurnTimeMs = 400; //should ensure a minimal turn of atleast 120°
constexpr uint32_t turnTimeOutMs = 5000; //constant for the maximum time a turn should be attempted for
constexpr uint32_t allignementMs = 500; //constant for the allignement reversal time
constexpr uint16_t reversalSpeed = 600; //speed for reversal
constexpr uint32_t allignementTimeOutMs = 3000; // timout for reversal in allignOnCheckpoint

bool perform180Turn(QTRSensors &qtr, Motor &motorL, Motor &motorR){
    turnLeft(motorL, motorR);


    uint32_t startOfTurn = millis(); //variable for detecting if the turn failed (took too long)
    uint16_t sensorValues[QTR_SENSOR_COUNT];
    qtr.readLineBlack(sensorValues);

    //first turn until the sensor is not on the line anymore
    while((sensorValues[2] > BLACKLINE_THRESHOLD) && millis() - startOfTurn < turnTimeOutMs){
      delay(10);
      qtr.readLineBlack(sensorValues);
    }

    //turn until the it detects the line again with minturntime to make sure it doesn't stop at a perpendicular line
    while(((sensorValues[2] < BLACKLINE_THRESHOLD) || (millis() - startOfTurn < minTurnTimeMs)) && millis() - startOfTurn < turnTimeOutMs){
      delay(10);
      qtr.readLineBlack(sensorValues);
    }

    //ensure that the sensor is centered on the line
    /*while((sensorValues[3] > BLACKLINE_THRESHOLD || sensorValues[1] > BLACKLINE_THRESHOLD) && millis() - startOfTurn < turnTimeOutMs){
      if(sensorValues[1] > BLACKLINE_THRESHOLD){
        motorL.brake();
        motorR.brake();
        turnRight(motorL, motorR);
        while(sensorValues[1] > BLACKLINE_THRESHOLD){
          delay(10);
          qtr.readLineBlack(sensorValues);
        }
      }
      if(sensorValues[3] > BLACKLINE_THRESHOLD){
        motorL.brake();
        motorR.brake();
        turnLeft(motorL, motorR);
        while(sensorValues[3] > BLACKLINE_THRESHOLD){
          delay(10);
          qtr.readLineBlack(sensorValues);
        }
      }
      delay(10);
      qtr.readLineBlack(sensorValues);
    }*/

    motorL.brake();
    motorR.brake();

    bool turnSuccess = millis() - startOfTurn < turnTimeOutMs;

    if(!turnSuccess){
      Serial.println("180-turn error");
    }
    
    return turnSuccess;
}

bool allignOnCheckpoint(PIDController &pid, QTRSensors &qtr, Motor &motorL, Motor &motorR){
  
  uint16_t sensorValues[QTR_SENSOR_COUNT];
  uint32_t startMs = millis();
  uint32_t periodMs = 1000/driveLoopHzTarget;

  //follow the line for a short time to ensure proper allignement
  while(millis() - startMs < allignementMs){
    int position = qtr.readLineBlack(qtrSensorValues);
    float signal = calculatePIDstep(pid, position, driveLoopHzTarget);
    setMotorSpeeds(signal, motorL, motorR);
    delay(periodMs);
  }
  motorL.brake();
  motorR.brake();
  motorL.setSpeed(-reversalSpeed);
  motorR.setSpeed(-reversalSpeed);
  qtr.readLineBlack(sensorValues);
  
  //reverse until line is detected again
  while(sensorValues[0] < BLACKLINE_THRESHOLD && millis() - startMs < allignementTimeOutMs){
      delay(10);
      qtr.readLineBlack(sensorValues);
  }
  
  motorL.brake();
  motorR.brake();


  bool allignementSuccess = millis() - startMs < allignementTimeOutMs;
  
  if(!allignementSuccess){
    Serial.println("allignement error");
  }

  return allignementSuccess;
}
