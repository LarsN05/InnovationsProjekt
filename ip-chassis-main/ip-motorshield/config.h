#pragma once
#include <stdint.h>

// Set to 1 to enable the WiFi web interface for live telemetry and remote control.
// Set to 0 for standalone mode: the robot auto-starts after calibration with no WiFi.
#define ENABLE_WEB_INTERFACE 1

constexpr char accessPointNamePrefix[] = "IP-Motorshield";  // The WiFi SSID will be this prefix followed by the last 3 bytes of the MAC address, e.g. "IP-Motorshield-1A2B3C".
constexpr char accessPointTeamName[] = "H09";  // Insert team name here, e.g. "TeamA"
constexpr char accessPointPassword[] = "motorshield";  // You should change the default password such that no other person can easily connect to your system and control it.

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


#define BLACKLINE_THRESHOLD 300 // Minimum threshold for the center sensor to consider the robot to be centered on. Note that this value depends on supply voltage (ca. 300 for 3V3 and ca. 550 for 5V) and the reflectivity of the surface. If the calibration does not finish, try to lower this threshold.


// 12V main battery supply is connected to the ADC through voltage divider with R_upper = 10k and R_lower = 2k7, so the conversion factor is 2.7 / (10 + 2.7) = 0.2126.
#define BATTERY_VOLTAGE_TO_MONITOR_VOLTAGE_CONVERSION_FACTOR (0.2126) // Bat_monitor = Vbat * 0.2126
#define BATTERY_VOLTAGE_FROM_MONITOR_VOLTAGE_CONVERSION_FACTOR (1.0 / BATTERY_VOLTAGE_TO_MONITOR_VOLTAGE_CONVERSION_FACTOR) // Vbat = Bat_mon * 4.704

// 5V Servo supply is connected to the ADC through voltage divider with R_upper = 4k7 and R_lower = 6k8, so the conversion factor is 6.8 / (4.7 + 6.8) = 0.5913.
#define SERVO_5V_VOLTAGE_TO_MONITOR_VOLTAGE_CONVERSION_FACTOR (0.5913) // Servo_5V_monitor = 5V_Servo * 0.5913
#define SERVO_5V_VOLTAGE_FROM_MONITOR_VOLTAGE_CONVERSION_FACTOR (1.0 / SERVO_5V_VOLTAGE_TO_MONITOR_VOLTAGE_CONVERSION_FACTOR) // 5V_Servo = Servo_5V_monitor * 1.691

#define HOLD_ON_CALIBRATION_FAILURE (false)