#pragma once

// Pinout for IP Motorshield PCB V3, based on Motorshield_ESP32_V3_Pinout_28052026.xlsx

// --- I2C ---
#define I2C_SDA_PIN (8)
#define I2C_SCL_PIN (9)

// --- SPI ---
#define SPI_CS_PIN   (10)
#define SPI_MOSI_PIN (11)
#define SPI_SCK_PIN  (12)
#define SPI_MISO_PIN (13)

// --- DC motor control (DRV8251), direct GPIO ---
// Motors 1 & 2 are connected directly to ESP32 GPIOs
#define MOTOR_1_A_PIN (5)
#define MOTOR_1_B_PIN (4)

#define MOTOR_2_A_PIN (2)
#define MOTOR_2_B_PIN (1)

// Motors 3-6 are controlled via PCA9685 PWM expander (I2C address 0x44)
#define PCA9685_MOTOR_I2C_ADDR (0x44)

#define MOTOR_3_A_CHANNEL (6)
#define MOTOR_3_B_CHANNEL (7)

#define MOTOR_4_A_CHANNEL (4)
#define MOTOR_4_B_CHANNEL (5)

#define MOTOR_5_A_CHANNEL (2)
#define MOTOR_5_B_CHANNEL (3)

#define MOTOR_6_A_CHANNEL (0)
#define MOTOR_6_B_CHANNEL (1)

// --- Servo control via PCA9685 (I2C address 0x40) ---
#define PCA9685_SERVO_I2C_ADDR (0x40)

#define SERVO_1_CHANNEL (0)
#define SERVO_2_CHANNEL (1)
#define SERVO_3_CHANNEL (2)
#define SERVO_4_CHANNEL (3)
#define SERVO_5_CHANNEL (4)
#define SERVO_6_CHANNEL (5)
#define SERVO_7_CHANNEL (6)
#define SERVO_8_CHANNEL (7)

// --- QTR-MD-05RC sensor LED control pins ---
// QTR emitter enable (CTRL) should be hardwired to 3V3 on PCB — no GPIO is connected.
// Referencing this macro is a compile error to catch stale call sites.
#define QTR_EMITTER_ENABLE_PIN _Pragma("GCC error \"QTR_EMITTER_ENABLE_PIN (CTRL) should be hardwired to 3V3 on the PCB — it is not required to toggle this pin via GPIO.\"")

#define QTR_LED_1_PIN (16)
#define QTR_LED_2_PIN (15)
#define QTR_LED_3_PIN (41)
#define QTR_LED_4_PIN (7)
#define QTR_LED_5_PIN (42)
#define QTR_SENSOR_PINS ((const uint8_t[]){QTR_LED_1_PIN, QTR_LED_2_PIN, QTR_LED_3_PIN, QTR_LED_4_PIN, QTR_LED_5_PIN})
#define QTR_SENSOR_COUNT (sizeof(QTR_SENSOR_PINS) / sizeof(QTR_SENSOR_PINS[0]))
#define QTR_LINE_MAX_VALUE ((QTR_SENSOR_COUNT - 1) * 1000)

// --- Battery & current monitoring via ADS1015 ADC expanders ---
// ADS1015 #1 (I2C address 0x48): motor current sensing for motors 1-4
#define ADS1015_1_I2C_ADDR            (0x48)
#define ADS1015_1_ISENSE_MOTOR1_CH    (2)
#define ADS1015_1_ISENSE_MOTOR2_CH    (1)
#define ADS1015_1_ISENSE_MOTOR3_CH    (0)
#define ADS1015_1_ISENSE_MOTOR4_CH    (3)

// ADS1015 #2 (I2C address 0x49): 12V battery, 5V servo rail, motor current sensing for motors 5-6
#define ADS1015_2_I2C_ADDR            (0x49)
#define ADS1015_2_BAT_MON_CH          (0)
#define ADS1015_2_ISENSE_MOTOR6_CH    (1)
#define ADS1015_2_ISENSE_MOTOR5_CH    (2)
#define ADS1015_2_5V_SERVO_MON_CH     (3)

// --- Port expander MCP23017 (I2C address 0x20) ---
#define MCP23017_I2C_ADDR (0x20)

// --- Hardware RGB LED on the ESP32S3 DevKit (WS2812B) ---
#define ESP32_RGB_LED_DATA_PIN (38)
