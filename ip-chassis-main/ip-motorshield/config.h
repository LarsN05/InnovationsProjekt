#pragma once
#include <stdint.h>

// Set to 1 to enable the WiFi web interface for live telemetry and remote control.
// Set to 0 for standalone mode: the robot auto-starts after calibration with no WiFi.
#define ENABLE_WEB_INTERFACE 1

// Set to 1 to test the encoder motor: skips the sensor calibration and the line following,
// the robot does not drive. Set the motor rpm live in the web interface ("Encoder motor test").
#define ENCODER_MOTOR_TEST_MODE 0

// Set to 1 to test allignOnCheckpoint() alone: the robot follows the line normally, but at EVERY
// cross line it only runs the alignment (no mission actions) and then stops. Results go to Serial.
// Press Start in the web interface for the next try. Set back to 0 for the real run.
#define ALIGN_TEST_MODE 0

#if ALIGN_TEST_MODE && ENCODER_MOTOR_TEST_MODE
#error "ALIGN_TEST_MODE and ENCODER_MOTOR_TEST_MODE cannot both be 1"
#endif

constexpr char accessPointNamePrefix[] = "IP-Motorshield";  // The WiFi SSID will be this prefix followed by the last 3 bytes of the MAC address, e.g. "IP-Motorshield-1A2B3C".
constexpr char accessPointTeamName[] = "H09";  // Insert team name here, e.g. "TeamA"
constexpr char accessPointPassword[] = "dropzone9886";  // You should change the default password such that no other person can easily connect to your system and control it.

// Keep the M1/M2 LEDC PWM aligned with the M3-M6 PCA9685 target frequency.
constexpr int pwmFreq = 1526;
constexpr int pwmResolution = 12; // 0..4095, 12-bit everywhere (matches PCA9685)

#define PWM_MAX_VALUE ((1 << pwmResolution) - 1)

// PCA9685 PWM frequencies. The motor expander runs at the chip maximum
// (~1526 Hz, prescale 3), matching the M1/M2 LEDC target above.
// Servos need the standard 50 Hz frame.
constexpr float pcaMotorPwmFreqHz = 1526.0f;
constexpr float pcaServoPwmFreqHz = 50.0f;

// Default servo pulse range, mapped onto the servo's mechanical travel.
constexpr uint16_t servoDefaultMinUs = 500;
constexpr uint16_t servoDefaultMaxUs = 2500;
// Default mechanical travel; override per servo with setAngleRange(), e.g. 180.
constexpr float servoDefaultMaxAngleDeg = 270.0f;

// DRV8251A IPROPI current mirror: 1575 uA/A (datasheet typ) into the 1.3k
// shunt resistor (R9 etc. in the V3 schematic) -> 2.0475 V per amp.
constexpr float motorIsenseVoltsPerAmp = 2.0475f;
// Samples averaged per motor-current reading. Values greater than one use the
// ADS1015 continuous-conversion mode for the duration of the blocking read.
// At 3300 SPS, 15 current samples span ~4.55 ms and ~6.9 PWM cycles. The
// non-harmonic rates distribute samples across the waveform with a maximum
// uncovered phase gap of about 27 degrees, providing good coverage.
constexpr uint16_t motorCurrentAdcSamples = 15;

// TT motor with encoder (see movement-functions.cpp)
constexpr uint8_t encodedMotorNumber = 6;        // motor port on the shield (M6)
constexpr int encodedMotorMaxDuty = 2600;        // ca. 7.6 V at 12 V battery, TT motor is rated 3-6 V
constexpr float encoderTicksPerRev = 288.0f;     // ticks per output shaft revolution (rising edges of A), measured by turning the wheel by hand
constexpr int encodedMotorMaxRpm = 170;          // datasheet: 155 rpm +-5% at the wheel axle, higher targets are clipped
constexpr int encoderDirection = 1;             // set to -1 if the measured rpm is negative while the target is positive (motor runs away)
constexpr uint32_t encoderUpdatePeriodMs = 20;   // speed controller runs at 50 Hz
// Speed controller: duty = feedforward (start duty + duty per rpm) + P + I
// Feedforward measured from the log: running needs ca. 400 duty at 15 rpm and 550 duty at 35 rpm
constexpr int encoderStartDuty = 300;            // duty offset of the feedforward (friction)
constexpr float encoderFeedforward = 7.0f;       // duty per rpm on top of encoderStartDuty
constexpr int encoderBreakawayDuty = 750;        // minimum duty while the motor stands still: a stalled motor needs more to start than to keep running
constexpr int encodedMotorMinRpm = 20;           // below this the motor sticks and slips, smaller targets (except 0) are raised to this
constexpr float encoderSpeedKp = 2.0f;           // duty per rpm error: reacts immediately to load; lower if it jitters
constexpr float encoderSpeedKi = 10.0f;          // duty per rpm error and second: removes the remaining error; lower if it oscillates
constexpr uint8_t encoderRpmWindow = 5;          // rpm is measured over this many updates (5 x 20 ms = 100 ms); larger = smoother but slower

// Barrier sensor: small value = strong reflection (object close or bright)
constexpr uint16_t barrierTimeoutUs = 1000;      // read() returns this value when nothing reflects
constexpr uint16_t barrierThreshold = 600;       // measured: cube in front ~295-305 us, no cube 1000 us (timeout)

// Motor ports (M1..M6). Drive motors should stay on M1/M2 (direct ESP32 PWM, lowest latency).
constexpr uint8_t driveMotorLeft = 2;      // swap these two numbers if the robot steers away from the line
constexpr uint8_t driveMotorRight = 1;
constexpr bool invertDriveMotors = false;  // set to true if the robot drives backwards (sensor bar trailing)
constexpr uint8_t revolverMotorNumber = 5;

// Checkpoint detection (cross line = all 5 sensors black)
constexpr uint8_t checkpointConfirmReads = 2;  // consecutive all-black drive-loop reads needed to count a checkpoint (2 x 25 ms at 40 Hz). Set to 1 if checkpoints are missed at high speed.
constexpr uint32_t checkpointLockoutMs = 600;  // after a checkpoint, further cross lines are ignored for this long (filters double counts)

// Mission: checkpoint numbers. What happens at each one is listed in the table in mission.cpp.
// Track: Wettbewerbsstrecke 2026. Each cross line counts once. The cannon shoots to the LEFT of the robot.
//  1      distribution center (DS)
//  2-4    Haus 1: bottom line, robot drives east, house on its left
//  5-7    Haus 2: top line, robot drives north, house on its left | 180 degree turn at 7
//  7-9    Haus 3: same line after the turn, robot drives south, house on its left (7 = shot right after the turn)
//  10-12  Haus 1 again, now on the right: the cannon cannot reach it, no shots
//  13     distribution center = finish
constexpr int distributionCenter = 1;
constexpr int turnCheckpoint = 7;         // top cross line: shoot Haus 2, 180 degree turn, shoot Haus 3
constexpr int dcCheckpoint = 12;          // not used by the mission yet
constexpr int finalCheckpoint = 13;       // distribution center

// Shot settings for the 12 possible trajectories: 3 houses x 4 balcony heights.
// Placeholder values - determine them experimentally on the track.
struct ShotSetting
{
    bool leftSlide; // switch position: true = left slide, false = right slide (only used if enableSlideSwitch)
    int rpm;        // shooter speed in rpm
};

enum HouseIndex
{
    HouseClose = 0,  // Haus 1: next to the bottom line, checkpoints 2-4
    HouseMedium = 1, // Haus 2: left of the top line, checkpoints 5-7 before the turn
    HouseFar = 2,    // Haus 3: right of the top line, checkpoints 7-9 after the turn, farthest from the line
    HouseCount = 3
};
constexpr int balconyHeightCount = 4; // index 0 = lowest balcony ... 3 = highest balcony

constexpr ShotSetting shotSettings[HouseCount][balconyHeightCount] = {
    //                      height 0       height 1       height 2       height 3
    /* HouseClose  */ {{false, 100}, {false, 100}, {false, 100}, {false, 100}},
    /* HouseMedium */ {{false, 100}, {false, 100}, {false, 100}, {false, 100}},
    /* HouseFar    */ {{false, 100}, {false, 100}, {false, 100}, {false, 100}},
};

// Shooting plan: one line per shot. At every cross line listed here the robot stops and fires these shots
// in order (several lines with the same checkpoint = several shots there). Slide and rpm come from shotSettings.
// Each cross line faces one balcony of its house; balconyHeight says which balcony that is.
// afterTurn is only relevant at turnCheckpoint: false = before the 180 turn (Haus 2), true = after it (Haus 3).
// The balcony heights below are placeholders - set them to the balcony each cross line actually faces.
struct PlannedShot
{
    int checkpoint;
    bool afterTurn;
    HouseIndex house;
    int balconyHeight; // 0..balconyHeightCount-1
};

constexpr PlannedShot shotPlan[] = {
    // Haus 1 (bottom line, driving east)
    {2, false, HouseClose, 0},
    {3, false, HouseClose, 1},
    {4, false, HouseClose, 2},
    // Haus 2 (top line, driving north)
    {5, false, HouseMedium, 0},
    {6, false, HouseMedium, 1},
    {7, false, HouseMedium, 2},
    // Haus 3 (top line after the 180 turn, driving south)
    {7, true, HouseFar, 2},
    {8, true, HouseFar, 1},
    {9, true, HouseFar, 0},
};

// Mission: distribution center (checkpoint 1).
// Sequence: stop on the cross line, wait, back off, extend the arm, drive forward (cubes fall), retract the arm, continue.
// The moves are measured from the checkpoint cross line instead of being purely timed:
// each move drives until the cross line is reached or left, then keeps going for the "extra" time.
// The extra times are first estimates - tune them on the track.
constexpr int missionDriveSpeed = 800;
constexpr uint8_t distributionServo = 1;
constexpr float distributionServoRangeDeg = 270.0f;
constexpr float distributionServoDownDeg = 0.0f;
constexpr float distributionServoUpDeg = 150.0f;
constexpr uint32_t distributionStandStillMs = 3500;   // rule: stand still at the marker for at least 3 s before the flap is triggered
constexpr uint32_t distributionBackOffExtraMs = 400;  // back off: until the cross line is left, then this long
constexpr uint32_t distributionForwardExtraMs = 400;  // forward with the arm extended: until the cross line is reached again, then this long
constexpr uint32_t distributionServoSettleMs = 500;   // time for the arm to move
constexpr uint32_t distributionDropWaitMs = 1000;     // after driving forward: time for the cubes to fall before the arm retracts
constexpr uint32_t crossLineTimeoutMs = 2000;         // a move that does not find the cross line in time stops the run

// Mission: drive moves
constexpr uint32_t moveRampMs = 100;               // speed ramps up over this time to avoid wheel slip
constexpr float nominalBatteryVoltageV = 12.6f;    // mission move speeds are scaled by nominal / measured battery voltage
constexpr float minValidBatteryVoltageV = 9.0f;    // below this (USB only, power switch off) no compensation is applied
constexpr uint32_t checkpointStopPauseMs = 1000;   // pause for "Stop at checkpoints" in the web interface

// Passive piezo buzzer (buzzer.cpp, pin in pinout.h). Sounds can be played from the web interface.
constexpr bool enableBuzzer = true;
constexpr bool enableBuzzerPushPull = true;  // second buzzer leg on GPIO 13 (SPI MISO) instead of GND: twice the voltage, much louder.
                                             // ONLY for a piezo buzzer. Set to false if leg 2 is on GND.
constexpr bool enableMissionSounds = true; // play matching sounds automatically during the run (checkpoints, shooting, finish, ...)

// Slide switch servo (setSwitch). Leave disabled until the servo is mounted.
constexpr bool enableSlideSwitch = false;
constexpr uint8_t slideSwitchServo = 8;
constexpr float slideSwitchRangeDeg = 270.0f;      // set to 180 for a 180 degree servo
constexpr float switchCenterDeg = 165;             // center position of the switch servo
constexpr float switchOffsetDeg = 20;              // switch travel to each side of the center


#define BLACKLINE_THRESHOLD 300 // Minimum threshold for the center sensor to consider the robot to be centered on. Note that this value depends on supply voltage (ca. 300 for 3V3 and ca. 550 for 5V) and the reflectivity of the surface. If the calibration does not finish, try to lower this threshold.


// 12V main battery supply is connected to the ADC through voltage divider with R_upper = 10k and R_lower = 2k7, so the conversion factor is 2.7 / (10 + 2.7) = 0.2126.
#define BATTERY_VOLTAGE_TO_MONITOR_VOLTAGE_CONVERSION_FACTOR (0.2126) // Bat_monitor = Vbat * 0.2126
#define BATTERY_VOLTAGE_FROM_MONITOR_VOLTAGE_CONVERSION_FACTOR (1.0 / BATTERY_VOLTAGE_TO_MONITOR_VOLTAGE_CONVERSION_FACTOR) // Vbat = Bat_mon * 4.704

// 5V Servo supply is connected to the ADC through voltage divider with R_upper = 4k7 and R_lower = 6k8, so the conversion factor is 6.8 / (4.7 + 6.8) = 0.5913.
#define SERVO_5V_VOLTAGE_TO_MONITOR_VOLTAGE_CONVERSION_FACTOR (0.5913) // Servo_5V_monitor = 5V_Servo * 0.5913
#define SERVO_5V_VOLTAGE_FROM_MONITOR_VOLTAGE_CONVERSION_FACTOR (1.0 / SERVO_5V_VOLTAGE_TO_MONITOR_VOLTAGE_CONVERSION_FACTOR) // 5V_Servo = Servo_5V_monitor * 1.691

#define HOLD_ON_CALIBRATION_FAILURE (false)