#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <stdint.h>

namespace ESP32Pins
{
    // Left Side Pins (Top to Bottom)
    struct LeftSide
    {
        // Input only pins
        static const uint8_t GPIO36 = 36;  // Input only, ADC1_0
        static const uint8_t GPIO39 = 39;  // Input only, ADC1_3
        static const uint8_t GPIO34 = 34;  // Input only, ADC1_6
        static const uint8_t GPIO35 = 35;  // Input only, ADC1_7
        // Regular GPIO pins
        static const uint8_t GPIO32 = 32;  // ADC1_4, Touch9, XTAL_32K_P
        static const uint8_t GPIO33 = 33;  // ADC1_5, Touch8, XTAL_32K_N
        static const uint8_t GPIO25 = 25;  // ADC2_8, DAC1
        static const uint8_t GPIO26 = 26;  // ADC2_9, DAC2
        static const uint8_t GPIO27 = 27;  // ADC2_7, Touch7
        static const uint8_t GPIO14 = 14;  // ADC2_6, Touch6
        static const uint8_t GPIO12 = 12;  // ADC2_5, Touch5
        static const uint8_t GPIO13 = 13;  // ADC2_4, Touch4
    };

    // Right Side Pins (Top to Bottom)
    struct RightSide
    {
        static const uint8_t GPIO23 = 23;  // MOSI, V_SPI_D
        static const uint8_t GPIO22 = 22;  // V_SPI_WP
        static const uint8_t GPIO1  = 1;   // TX0
        static const uint8_t GPIO3  = 3;   // RX0
        static const uint8_t GPIO21 = 21;  // V_SPI_HD
        static const uint8_t GPIO19 = 19;  // MISO, V_SPI_Q
        static const uint8_t GPIO18 = 18;  // SCK, V_SPI_CLK
        static const uint8_t GPIO5  = 5;   // V_SPI_CS0
        static const uint8_t GPIO17 = 17;  // TX2
        static const uint8_t GPIO16 = 16;  // RX2
        static const uint8_t GPIO4  = 4;   // ADC2_0, Touch0
        static const uint8_t GPIO2  = 2;   // ADC2_2, Touch2
        static const uint8_t GPIO15 = 15;  // ADC2_3, Touch3
    };
};  // namespace ESP32Pins

namespace Config
{
    // System Configuration
    struct System
    {
        static const uint32_t SERIAL_BAUD_RATE = 115200;  // Serial communication speed
        static const uint32_t STARTUP_DELAY_MS = 1000;    // Initial delay for system stability
    };

    // SPI Configuration
    struct SPI
    {
        static const uint8_t MOSI = ESP32Pins::RightSide::GPIO23;
        static const uint8_t MISO = ESP32Pins::RightSide::GPIO19;
        static const uint8_t SCK  = ESP32Pins::RightSide::GPIO18;

        static const uint8_t MOTOR1_CS = ESP32Pins::RightSide::GPIO4;
        static const uint8_t MOTOR2_CS = ESP32Pins::RightSide::GPIO5;
        static const uint8_t MOTOR3_CS = ESP32Pins::RightSide::GPIO15;
        static const uint8_t MOTOR4_CS = ESP32Pins::RightSide::GPIO16;

        static const uint8_t TEST_VALUE = 0x55;  // Test pattern for SPI communication
    };

    // TMC5160 Motor Control Configuration
    struct TMC5160T_Driver
    {
        // Number of motors in the system
        static const uint8_t NUM_MOTORS = 4;

        static const uint8_t MOTOR1_EN_PIN = ESP32Pins::RightSide::GPIO17;
        static const uint8_t MOTOR2_EN_PIN = ESP32Pins::LeftSide::GPIO25;
        static const uint8_t MOTOR3_EN_PIN = ESP32Pins::RightSide::GPIO21;
        static const uint8_t MOTOR4_EN_PIN = ESP32Pins::RightSide::GPIO22;

        static const uint8_t MOTOR1_STEP_PIN = ESP32Pins::LeftSide::GPIO26;
        static const uint8_t MOTOR2_STEP_PIN = ESP32Pins::LeftSide::GPIO27;
        static const uint8_t MOTOR3_STEP_PIN = ESP32Pins::LeftSide::GPIO32;
        static const uint8_t MOTOR4_STEP_PIN = ESP32Pins::LeftSide::GPIO33;

        static const uint8_t MOTOR1_DIR_PIN = ESP32Pins::LeftSide::GPIO12;
        static const uint8_t MOTOR2_DIR_PIN = ESP32Pins::LeftSide::GPIO13;
        static const uint8_t MOTOR3_DIR_PIN = ESP32Pins::LeftSide::GPIO14;
        static const uint8_t MOTOR4_DIR_PIN = ESP32Pins::RightSide::GPIO2;

        // Motor driver settings
        static const uint16_t TMC_CURRENT_MA = 1000;  // Motor RMS current in mA
        static const uint8_t  MICROSTEPS     = 16;    // Microstep resolution

        // Current control settings
        static const uint16_t CURRENT_STEP_MA = 100;   // Current adjustment step size in mA
        static const uint16_t MIN_CURRENT_MA  = 100;   // Minimum allowed current in mA
        static const uint16_t MAX_CURRENT_MA  = 2000;  // Maximum allowed current in mA

        // Speed control settings
        static const uint32_t SPEED_STEP = 100;    // Speed adjustment step size in steps/sec
        static const uint32_t ACCEL_STEP = 50;     // Acceleration adjustment step size in steps/sec²
        static const uint32_t MIN_SPEED  = 100;    // Minimum speed in steps/sec
        static const uint32_t MAX_SPEED  = 10000;  // Maximum speed in steps/sec

        // Temperature monitoring settings
        static const uint8_t  TEMP_WARNING_THRESHOLD = 80;    // Temperature warning threshold in °C
        static const uint32_t TEMP_PRINT_INTERVAL    = 1000;  // Temperature print interval in ms

        // Status monitoring settings
        static const uint16_t STATUS_PRINT_INTERVAL = 1000;  // Status print interval in steps

        // Driver configuration settings
        static const uint8_t  TOFF       = 5;     // Driver off time
        static const uint8_t  BLANK_TIME = 24;    // Driver blank time
        static const uint8_t  IHOLDDELAY = 6;     // Hold current delay
        static const uint16_t TCOOLTHRS  = 1000;  // CoolStep threshold
        static const uint8_t  TPOWERDOWN = 10;    // Power down time after standstill

        // StallGuard and CoolStep settings
        static const int8_t   SGTHRS                 = 10;    // StallGuard threshold
        static const uint8_t  CURRENT_SCALING        = 32;    // Current scaling factor
        static const uint8_t  IRUNDELAY              = 5;     // Run current delay
        static const uint8_t  PWM_OFS                = 36;    // PWM offset
        static const uint8_t  PWM_GRAD               = 14;    // PWM gradient
        static const uint8_t  PWM_FREQ               = 1;     // PWM frequency
        static const uint8_t  HSTRT                  = 5;     // Hysteresis start
        static const uint8_t  HEND                   = 3;     // Hysteresis end
        static const uint32_t LOAD_THRESHOLD         = 1000;  // Load threshold
        static const uint32_t LOAD_WARNING_THRESHOLD = 1500;  // Load warning threshold
    };

    // Command Handler Configuration
    namespace CommandHandler
    {
        constexpr char CMD_FORWARD     = 'w';  // *
        constexpr char CMD_REVERSE     = 's';  // *
        constexpr char CMD_STOP        = 'x';  // *
        constexpr char CMD_RESET       = 'z';
        constexpr char CMD_TEST_SPI    = 't';  // *
        constexpr char CMD_SHOW_STATUS = 'i';  // *
        constexpr char CMD_SHOW_CONFIG = 'p';  // *
        constexpr char CMD_SHOW_TEMP   = 'm';
        constexpr char CMD_TOGGLE_MODE = 'n';
        constexpr char CMD_HELP        = 'h';  // *
        constexpr char CMD_HELP_ALT    = '?';  // *
    }  // namespace CommandHandler

    // Motor Controller Configuration
    struct MotorController
    {
        // Status monitoring settings
        static constexpr int STATUS_PRINT_INTERVAL = 1000;  // Status print interval in steps

        // Current settings with motor specifications constraints
        static constexpr uint16_t CURRENT_STEP     = 100;   // Current adjustment step size in mA
        static constexpr uint16_t MIN_CURRENT      = 100;   // Minimum allowed current in mA
        static constexpr uint16_t MAX_RUN_CURRENT  = 1000;  // Maximum run current in mA (1A max)
        static constexpr uint16_t MAX_HOLD_CURRENT = 500;   // Maximum hold current in mA (0.5A max)

        // Speed and acceleration settings
        static constexpr uint16_t SPEED_STEP = 100;    // Speed adjustment step size in steps/sec
        static constexpr uint16_t ACCEL_STEP = 100;    // Acceleration adjustment step size in steps/sec²
        static constexpr uint16_t MIN_SPEED  = 100;    // Minimum speed in steps/sec
        static constexpr uint16_t MAX_SPEED  = 10000;  // Maximum speed in steps/sec
        static constexpr uint16_t MIN_ACCEL  = 100;    // Minimum acceleration in steps/sec²
        static constexpr uint16_t MAX_ACCEL  = 10000;  // Maximum acceleration in steps/sec²

        // Timing settings
        static constexpr int STEP_DELAY          = 500;   // Step pulse delay in microseconds
        static constexpr int TEMP_PRINT_INTERVAL = 1000;  // Temperature print interval in ms
    };

    // Motor Specifications Configuration
    struct MotorSpecs
    {
        // General specifications
        struct General
        {
            static constexpr float   STEP_ANGLE            = 1.8f;  // degrees
            static constexpr uint8_t NUMBER_OF_PHASES      = 2;
            static constexpr float   INSULATION_RESISTANCE = 100.0f;  // MΩ min at 500V DC
            static constexpr char    INSULATION_CLASS      = 'B';
            static constexpr float   ROTOR_INERTIA         = 2.0f;   // g·cm²
            static constexpr float   MASS                  = 30.0f;  // grams
        };

        // Motor constraints for pancake type
        struct Constraints
        {
            static constexpr float MAX_TEMPERATURE  = 50.0f;   // default: 80 °C (Class B insulation limit)
            static constexpr float MAX_CURRENT      = 0.55f;   // A (10% safety margin)
            static constexpr float MIN_VOLTAGE      = 2.0f;    // V (minimum operating voltage)
            static constexpr float MAX_ACCELERATION = 500.0f;  // default: 1000 steps/s² (for smooth operation)
            static constexpr float MAX_SPEED        = 250.0f;  // default: 500 steps/s (for stable operation)
        };

        // Operational parameters
        struct Operation
        {
            static constexpr uint16_t STEPS_PER_REV        = 200;    // steps per revolution
            static constexpr float    STARTUP_CURRENT      = 0.3f;   // A (reduced current for startup)
            static constexpr float    IDLE_CURRENT         = 0.25f;  // A (reduced current when idle)
            static constexpr float    RUN_CURRENT          = 100;    // Default 1000mA
            static constexpr float    HOLD_CURRENT         = 100;    // Default 500mA
            static constexpr float    SPEED                = 200;    // Default 1000 steps/sec
            static constexpr float    ACCELERATION         = 500;    // Default 1000 steps/sec²
            static constexpr uint32_t MAX_SPEED            = 10000;  // Maximum speed in steps/sec
            static constexpr uint32_t MAX_ACCELERATION     = 10000;  // Maximum acceleration in steps/sec²
            static constexpr uint32_t MAX_DECELERATION     = 10000;  // Maximum deceleration in steps/sec²
            static constexpr uint32_t HIGH_SPEED_THRESHOLD = 5000;   // Threshold for high speed operation
        };

        // Electrical specifications
        struct Electrical
        {
            static constexpr float RATED_VOLTAGE    = 7.25f;  // V
            static constexpr float RATED_CURRENT    = 0.5f;   // A
            static constexpr float PHASE_RESISTANCE = 3.5f;   // Ω ±10%
            static constexpr float PHASE_INDUCTANCE = 0.90f;  // mH ±20%
            static constexpr float HOLDING_TORQUE   = 16.0f;  // mN·m
            static constexpr float DETENT_TORQUE    = 2.0f;   // mN·m
        };
    };
};  // namespace Config

#endif  // CONFIG_H