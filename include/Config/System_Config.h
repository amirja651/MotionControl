#ifndef SYSTEM_CONFIG_H
#define SYSTEM_CONFIG_H

#include <stdint.h>

namespace CONFIG
{
    // System Configuration
    struct SYSTEM
    {
        // Motor driver settings
        static const uint16_t TMC_CURRENT_MA = 1000;  // Motor RMS current in mA
        static const uint8_t  MICROSTEPS     = 16;    // Microstep resolution

        // Current control settings
        static const uint16_t CURRENT_STEP_MA = 100;   // Current adjustment step size in mA
        static const uint16_t MIN_CURRENT_MA  = 100;   // Minimum allowed current in mA
        static const uint16_t MAX_CURRENT_MA  = 2000;  // Maximum allowed current in mA

        // Speed control settings
        static const uint32_t SPEED_STEP = 100;   // Speed adjustment step size in steps/sec
        static const uint32_t ACCEL_STEP = 50;    // Acceleration adjustment step size in steps/sec²
        static const uint32_t MIN_SPEED  = 100;   // Minimum speed in steps/sec
        static const uint32_t MAX_SPEED  = 500;   // Maximum speed in steps/sec
        static const uint32_t MIN_ACCEL  = 100;   // Minimum acceleration in steps/sec²
        static const uint32_t MAX_ACCEL  = 1000;  // Maximum acceleration in steps/sec²
        static const uint32_t MAX_DECEL  = 1000;  // Maximum deceleration in steps/sec²

        // Temperature monitoring settings
        static const uint8_t  TEMP_WARNING_THRESHOLD = 80;    // Temperature warning threshold in °C
        static const uint32_t TEMP_PRINT_INTERVAL    = 1000;  // Temperature print interval in ms

        // Status monitoring settings
        static const uint16_t STATUS_PRINT_INTERVAL = 1000;  // Status print interval in steps

        // Driver configuration settings
        static const uint8_t  TOFF       = 3;     // Driver off time
        static const uint8_t  BLANK_TIME = 24;    // Driver blank time
        static const uint8_t  IHOLDDELAY = 6;     // Hold current delay
        static const uint16_t TCOOLTHRS  = 1000;  // CoolStep threshold
        static const uint8_t  TPOWERDOWN = 10;    // Power down time after standstill

        // StallGuard and CoolStep settings
        static const int8_t   SGTHRS                 = 10;    // StallGuard threshold
        static const uint8_t  CURRENT_SCALING        = 32;    // Current scaling factor
        static const uint8_t  PWM_OFS                = 36;    // PWM offset
        static const uint8_t  PWM_GRAD               = 14;    // PWM gradient
        static const uint8_t  PWM_FREQ               = 1;     // PWM frequency
        static const uint8_t  HSTRT                  = 5;     // Hysteresis start
        static const uint8_t  HEND                   = 3;     // Hysteresis end
        static const uint32_t LOAD_THRESHOLD         = 1000;  // Load threshold
        static const uint32_t LOAD_WARNING_THRESHOLD = 1500;  // Load warning threshold
    };

    namespace MotorSpecs
    {
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
    }  // namespace MotorSpecs
}  // namespace CONFIG

#endif  // SYSTEM_CONFIG_H
