#include "movement-functions.h"
#include "shared-globals.h"
#include "qtr-sensor-util.h"
#include "onboard-led.h"
#include "config.h"
#include "src/motorshield/motorshield.h"

extern Motorshield shield;

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

static volatile long encoderTicks = 0;
static bool encodedMotorRegulated = false; // true while adjustSpeed() controls the motor
static float encodedMotorTargetRpm = 0;
static float encodedMotorIntegral = 0;     // I part of the speed controller in duty
static float encodedMotorMeasuredRpm = 0;
static float encodedMotorOutput = 0;       // last duty sent to the motor, for debugging

static void IRAM_ATTR encoderIsr(){
  encoderTicks += digitalRead(ENCODER_B_PIN) ? 1 : -1; // channel B tells the direction on a rising edge of A
}

void initEncodedMotor(){
  pinMode(ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(ENCODER_B_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), encoderIsr, RISING);
}

// speed: -100..100 (%), Vorzeichen = Richtung
void encodedMotorSetSpeed(int speed){
  speed = constrain(speed, -100, 100);
  encodedMotorRegulated = false;
  shield.motor(encodedMotorNumber).setSpeed(speed * encodedMotorMaxDuty / 100);
}

// rpm: Sollgeschwindigkeit an der Abtriebswelle, Vorzeichen = Richtung
void adjustSpeed(int rpm){
  rpm = constrain(rpm, -encodedMotorMaxRpm, encodedMotorMaxRpm);
  if(rpm != 0 && abs(rpm) < encodedMotorMinRpm){
    rpm = (rpm > 0) ? encodedMotorMinRpm : -encodedMotorMinRpm;
  }
  //restart the controller from zero when stopping or changing direction
  if(rpm == 0 || (rpm > 0) != (encodedMotorTargetRpm > 0)){
    encodedMotorIntegral = 0;
  }
  encodedMotorTargetRpm = rpm;
  encodedMotorRegulated = true; // updateEncodedMotor() brakes on rpm == 0, so this is safe to call from the web task
}

void updateEncodedMotor(){
  static uint32_t lastUpdateMs = 0;
  static long tickHistory[encoderRpmWindow] = {0};
  static uint32_t timeHistoryMs[encoderRpmWindow] = {0};
  static uint8_t historyIndex = 0;
  uint32_t nowMs = millis();
  if(nowMs - lastUpdateMs < encoderUpdatePeriodMs){
    return;
  }
  float dt = (nowMs - lastUpdateMs) / 1000.0f;
  lastUpdateMs = nowMs;

  //rpm over the last encoderRpmWindow updates: a single 20 ms window holds too few ticks at low rpm
  long ticks = encoderTicks; // 32 bit read is atomic on the ESP32
  float windowS = (nowMs - timeHistoryMs[historyIndex]) / 1000.0f;
  encodedMotorMeasuredRpm = encoderDirection * (ticks - tickHistory[historyIndex]) / encoderTicksPerRev / windowS * 60.0f;
  tickHistory[historyIndex] = ticks;
  timeHistoryMs[historyIndex] = nowMs;
  historyIndex = (historyIndex + 1) % encoderRpmWindow;

  if(!encodedMotorRegulated){
    return;
  }
  if(encodedMotorTargetRpm == 0){
    shield.motor(encodedMotorNumber).brake();
    encodedMotorRegulated = false;
    encodedMotorOutput = 0;
    return;
  }

  //feedforward: the duty the motor roughly needs for this rpm without load
  float direction = (encodedMotorTargetRpm > 0) ? 1.0f : -1.0f;
  float feedforward = direction * encoderStartDuty + encoderFeedforward * encodedMotorTargetRpm;

  //PI: too slow -> more power, too fast -> less power
  float error = encodedMotorTargetRpm - encodedMotorMeasuredRpm;
  float output = feedforward + encoderSpeedKp * error + encodedMotorIntegral;

  //never drive against the target direction: too fast -> less power down to 0, the motor slows down by itself
  float minOutput = (direction > 0) ? 0.0f : (float)-encodedMotorMaxDuty;
  float maxOutput = (direction > 0) ? (float)encodedMotorMaxDuty : 0.0f;

  //integral windup prevention: stop integrating while the output is clipped in the direction of the error
  bool clippedHigh = output >= maxOutput && error > 0;
  bool clippedLow = output <= minOutput && error < 0;
  if(!clippedHigh && !clippedLow){
    encodedMotorIntegral += encoderSpeedKi * error * dt;
  }

  //breakaway: a standing motor gets at least encoderBreakawayDuty, otherwise the I part needs seconds to overcome static friction
  bool stalled = fabs(encodedMotorMeasuredRpm) < 1.0f;
  if(stalled && direction * output < encoderBreakawayDuty){
    output = direction * encoderBreakawayDuty;
  }

  output = constrain(output, minOutput, maxOutput);
  encodedMotorOutput = output;
  shield.motor(encodedMotorNumber).setSpeed((int)output);
}

// wie delay(), aber der Encoder-Motor wird waehrenddessen weiter geregelt
void encodedMotorDelay(uint32_t ms){
  uint32_t startMs = millis();
  while(millis() - startMs < ms){
    updateEncodedMotor();
    delay(1);
  }
}

void runEncodedMotor(int rpm, uint32_t durationMs){
  adjustSpeed(rpm);
  encodedMotorDelay(durationMs);
  adjustSpeed(0);
  shield.motor(encodedMotorNumber).brake();
}

float encodedMotorRpm(){
  return encodedMotorMeasuredRpm;
}

float encodedMotorTargetRpmValue(){
  return encodedMotorTargetRpm;
}

long encodedMotorTicks(){
  return encoderTicks;
}

void printEncodedMotorDebug(){
  static uint32_t lastPrintMs = 0;
  static uint32_t lastLoopCount = 0;
  static uint32_t loopCount = 0;
  ++loopCount;
  uint32_t nowMs = millis();
  if(nowMs - lastPrintMs < 200){
    return;
  }
  float loopHz = (loopCount - lastLoopCount) * 1000.0f / (nowMs - lastPrintMs);
  lastPrintMs = nowMs;
  lastLoopCount = loopCount;
  Serial.printf("t=%lu soll=%.0f ist=%.1f ticks=%ld duty=%.0f I=%.0f A=%d B=%d loopHz=%.0f\n",
                nowMs, encodedMotorTargetRpm, encodedMotorMeasuredRpm, (long)encoderTicks, encodedMotorOutput,
                encodedMotorIntegral, digitalRead(ENCODER_A_PIN), digitalRead(ENCODER_B_PIN), loopHz);
}
