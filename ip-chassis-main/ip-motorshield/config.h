#pragma once
#include <stdint.h>

// Set to 1 to enable the WiFi web interface for live telemetry and remote control.
// Set to 0 for standalone mode: the robot auto-starts after calibration with no WiFi.
#define ENABLE_WEB_INTERFACE 0

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
// Name of the web interface: open http://aleea.local instead of http://192.168.4.1 (the IP keeps working).
// Lowercase letters, digits and '-' only (no special characters such as the reversed E: hostnames are ASCII).
constexpr char webHostName[] = "aleea";
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
// Line following speed between the checkpoints (PWM 0..4095, was 960). Only the normal line following uses it:
// the distribution center, the turns, the alignments and the row 3 drive keep their own slower speeds.
// Can also be changed live in the web interface ("Normal motor speed"). If the robot wobbles on the line at this
// speed, lower P a little or raise D; if it misses cross lines, set checkpointConfirmReads to 1.
constexpr int lineFollowSpeed = 1300;
// At a balcony cross line the robot brakes at the faster speed and may slide past the line: it then backs
// slowly onto the cross line again, so the shooting position stays the same as at the slow speed.
constexpr uint32_t balconyBackOntoLineTimeoutMs = 800;
constexpr uint8_t checkpointConfirmReads = 2;  // consecutive all-black drive-loop reads needed to count a checkpoint (2 x 25 ms at 40 Hz). Set to 1 if checkpoints are missed at high speed.
constexpr uint32_t checkpointLockoutMs = 600;  // after a checkpoint, further cross lines are ignored for this long (filters double counts)

// Mission: checkpoint numbers. What happens at each one is listed in the table in mission.cpp.
// Track: Wettbewerbsstrecke 2026. Each cross line counts once. The cannon shoots to the LEFT of the robot.
// Every house has 3 balcony rows and 3 cross lines. Row 1 is shot at the house's 2nd cross line,
// row 2 at its 3rd cross line, row 3 after following the line for thirdBalconyDriveMs (there is no 4th line).
//  1      distribution center (DS)
//  2-4    Haus 1: bottom line, robot drives east  -> row 1 at 3, row 2 at 4, row 3 at 4 + drive
//  5-7    Haus 2: top line, robot drives north    -> row 1 at 6, row 2 at 7, row 3 at 7 after a ~7 degree
//         right turn on the spot (and back), then 180 degree turn. After the turn the sensors are on the middle
//         cross line: the robot aligns on it and shoots Haus 3 row 1 there (inside the action, not counted).
//  8      Haus 3: bottom cross line of the top line  -> row 2 at 8, row 3 at 8 + drive
//  9-11   Haus 1 again, now on the right: the cannon cannot reach it, no shots
//  12     distribution center = finish
constexpr int distributionCenter = 1;
constexpr int turnCheckpoint = 7;         // top cross line: Haus 2 shots, turn, Haus 3 row 1
constexpr int dcCheckpoint = 11;          // not used by the mission yet
constexpr int finalCheckpoint = 12;       // distribution center

// Every house has 3 balcony rows (one per cross line position). Every balcony gets shot:
// row 1 has 3 balconies, row 2 has 4, row 3 has 3 -> 10 shots per house.
// Each shot has its own setting {rpm, switchRight}: switchRight true = right slide, false = left slide.
enum HouseIndex
{
    HouseClose = 0,  // Haus 1: next to the bottom line, checkpoints 2-4
    HouseMedium = 1, // Haus 2: left of the top line, checkpoints 5-7 before the turn
    HouseFar = 2,    // Haus 3: right of the top line, checkpoints 8-9 after the turn, farthest from the line
    HouseCount = 3
};
constexpr int balconyRowCount = 3;                          // row 1, 2, 3 in driving direction
constexpr int maxShotsPerRow = 4;
constexpr int shotsPerRow[balconyRowCount] = {3, 4, 3};    // row 1: 3 shots, row 2: 4 shots, row 3: 3 shots

struct Shot
{
    int rpm;          // shooter speed in rpm
    bool switchRight; // switch servo: true = right slide, false = left slide
};

// shots[house][row][shot]: shot 1 = lowest balcony. Placeholder rpm - tune every shot on the track.
// Firing order alternates per row (mission.cpp): 1st row of the run bottom -> top, 2nd row top -> bottom, and so on.
// Row 2 shot 4 = top balcony of the middle row: uses the right slide.
// Rows 1 and 3 fire only the first 3 shots; their 4th entry is unused.
constexpr Shot shots[HouseCount][balconyRowCount][maxShotsPerRow] = {
    // Haus 1           shot 1         shot 2         shot 3         shot 4
    {
        /* row 1 */ {{70, false}, {95, false}, {108, true}, {0, false}},
        /* row 2 */ {{70, false}, {95, false}, {108, true}, {115, true}},
        /* row 3 */ {{70, false}, {95, false}, {108, true}, {0, false}},
    },
    // Haus 2
    {
        /* row 1 */ {{88, false}, {110, false}, {125, true}, {0, false}},
        /* row 2 */ {{88, false}, {110, false}, {125, true}, {130, true}},
        /* row 3 */ {{88, false}, {110, false}, {125, true}, {0, false}},
    },
    // Haus 3
    {
        /* row 1 */ {{120, false}, {130, false}, {145, true}, {0, false}},
        /* row 2 */ {{120, false}, {130, false}, {145, true}, {150, true}},
        /* row 3 */ {{120, false}, {130, false}, {145, true}, {0, false}},
    },
};

// Row 3 has no cross line of its own: after the row 2 shots the robot follows the line for this long.
constexpr uint32_t thirdBalconyDriveMs = 1450;   // tested on the track; follows the line
constexpr int shotDriveSpeed = 800;

// Shooting plan: at which cross line each balcony row is shot. All shots of that row are fired there.
// afterTurn: only at turnCheckpoint - false = before the 180 turn (Haus 2), true = right after it (Haus 3 row 1,
//            on the middle cross line after aligning on it).
// move: what the robot does before the row:
//   Stay          shoot where the robot stands
//   FollowLine    follow the line for thirdBalconyDriveMs
//   TurnRight     turn right on the spot by about 7 degrees (rowTurnMs), shoot, turn back
enum class RowMove
{
    Stay,
    FollowLine,
    TurnRight
};

struct RowStop
{
    int checkpoint;
    bool afterTurn;
    HouseIndex house;
    int row; // 0..2 = row 1..3
    RowMove move;
};

constexpr RowStop shotPlan[] = {
    // Haus 1 (bottom line, driving east): cross lines 2, 3, 4
    {3, false, HouseClose, 0, RowMove::Stay},
    {4, false, HouseClose, 1, RowMove::Stay},
    {4, false, HouseClose, 2, RowMove::FollowLine},
    // Haus 2 (top line, driving north): cross lines 5, 6, 7
    // Put // in front of a house's lines to skip it (the 180 turn at cross line 7 still happens).
    {6, false, HouseMedium, 0, RowMove::Stay},
    {7, false, HouseMedium, 1, RowMove::Stay},
    {7, false, HouseMedium, 2, RowMove::TurnRight}, // stays on cross line 7, turns ~7 degrees towards row 3
    // Haus 3 (top line after the 180 turn, driving south): middle cross line right after the turn, then 8
    {7, true, HouseFar, 0, RowMove::Stay},
    {8, false, HouseFar, 1, RowMove::Stay},
    {8, false, HouseFar, 2, RowMove::FollowLine},
};

// RowMove::TurnRight (Haus 2 row 3 at cross line 7), measured with the line sensors instead of time:
// 1. back off the cross line until only the main line is under the sensors (on the cross line all sensors are black)
// 2. turn right in short pulses (centerTurnSpeed / centerPulseMs) until the line has moved rowTurnLineOffset
//    from the middle of the sensor bar -> always the same angle, independent of battery and floor
// 3. shoot, turn back in pulses until the line is centered again, drive forward onto cross line 7
// Tuning: Serial prints "turnToLineOffset: reached offset ...". Larger rowTurnLineOffset = larger angle.
// Position scale: 0..4000, one sensor = 1000. Estimate: about 1000 for 7 degrees.
constexpr int rowTurnLineOffset = 1000;
constexpr uint32_t rowTurnBackOffExtraMs = 100;   // back off: until the cross line is left, then this long
constexpr uint32_t rowTurnTimeoutMs = 1500;       // pulse turn gives up after this time
// Fallback (only if the robot cannot get off the cross line): time based turn on the spot.
// Right = the cannon (on the left side) swings towards the front.
constexpr uint32_t rowTurnMs = 80;
constexpr int rowTurnSpeed = 900;

// After the 180 turn: the robot drives forward (with line steering) until the sensors see the middle cross line.
// Then it aligns: forward past the cross line for turnAlignWiggleMs (straightens it on the line), and back
// slowly until it is on the cross line again. Then it shoots Haus 3 row 1.
constexpr uint32_t turnAlignSearchMs = 1500;     // finding the cross line: gives up after this long
constexpr uint32_t turnAlignWiggleMs = 500;      // forward part of the alignment
constexpr uint32_t turnAlignBackTimeoutMs = 1500;
constexpr int turnAlignSpeed = 700;

// QTR alignment after the 180 turn
// 1. Centering: after the turn the robot turns on the spot in short pulses until the line is under the
//    middle sensor (corrects the overshoot of the fast turn).
constexpr int centerTurnSpeed = 850;           // pivot speed of a pulse; raise it if the robot does not move
constexpr uint32_t centerPulseMs = 30;         // length of one pulse
constexpr int centerTolerance = 300;           // allowed line position error (0..4000 scale, 2000 = middle)
constexpr uint32_t centerTimeOutMs = 1500;
// 2. Squaring: when backing onto the cross line, each wheel drives until the outer sensor on its side sees
//    black, so the sensor bar ends parallel to the cross line (robot straight on the track).
//    Sensor 4 is on the left side (a line under it makes the PID steer left). If the robot twists away
//    from the line instead of squaring up, swap these two numbers.
constexpr uint8_t leftOuterSensor = 4;
constexpr uint8_t rightOuterSensor = 0;

// Revolver (M5) speed while turning to the next cube
constexpr int revolverTurnSpeed = 1500;   // original value; max 4095
// Jammed revolver: if no cube reaches the barrier sensor within revolverJamMs (or a cube sits at the sensor and
// does not clear), the revolver is taken as jammed: it turns backwards briefly to free the cube, then forwards again.
// Normal time per cube: see "turnUntilPackage: package detected after ... ms" on Serial. Set revolverJamMs to
// about 2x that time.
constexpr uint32_t revolverJamMs = 2500;           // no cube for this long -> jammed
constexpr uint32_t revolverFirstCubeJamMs = 8000;  // same for the very first cube after power-on: the revolver has to fill up first
constexpr uint32_t revolverUnjamReverseMs = 300;   // turn backwards this long to free the cube
constexpr int revolverUnjamSpeed = 2500;           // speed of the backwards turn (stronger than normal to break it loose)
constexpr uint8_t revolverMaxUnjamTries = 3;       // after this many tries the shot is skipped
// After each shot the shooter keeps this rpm until the cube has left, then goes straight to the next shot's rpm.
// The shooter only stops after the last shot of the run (keeps its momentum between rows and houses).
constexpr uint32_t shotFollowThroughMs = 2000;
// If a shot's exact rpm is not reached within the spin-up timeout, the cube is still fired as long as the
// shooter runs steadily above this rpm (otherwise that shot is skipped).
constexpr float minShotRpm = 65.0f;

// Mission: distribution center (checkpoint 1).
// Sequence: stop on the cross line, wait, back off, extend the arm, drive forward (triggers the DS),
// drive back, retract the arm, drive forward over the cross line, continue.
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
constexpr uint32_t distributionDropWaitMs = 1000;     // after driving forward: time for the cubes to fall
// Sideways shift towards the distribution center (it is on the LEFT): while backing off the cross line the robot
// turns its nose a little to the right, reverses (which moves it to the left), then turns back parallel to the line.
// After the drop the same move is done mirrored (back to the center line), then the robot centers on the line.
// Shift = back-off distance x sin(angle). Start: about 20 degrees -> roughly 2 cm. 0 = no shift.
// More shift: larger distributionShiftTurnMs (or longer distributionBackOffExtraMs).
constexpr uint32_t distributionShiftTurnMs = 240;     // turn on the spot for this long (about 12 ms per degree at rowTurnSpeed)
constexpr uint32_t crossLineTimeoutMs = 2000;

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
constexpr bool enableSlideSwitch = true;   // the switch servo is set before every shot (switchRight in shots[])
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

// Centering after the calibration sweeps: the robot turns in short pulses until the line is under the
// middle sensor, so it starts aligned with the line. Line position 0..4000, 2000 = centered.
constexpr int calibrationCenterSpeed = 700;            // turn speed of the pulses (must be above the motor deadzone ~560)
constexpr uint32_t calibrationCenterPulseMs = 25;      // motor on per pulse
constexpr uint32_t calibrationCenterSettleMs = 60;     // motor braked after each pulse, then the sensors are read
constexpr int calibrationCenterTolerance = 150;        // done when |position - 2000| is below this
constexpr uint32_t calibrationCenterTimeoutMs = 4000;  // give up after this long