#ifndef MOTOR_SPECS_H
#define MOTOR_SPECS_H

#include <stdint.h>

namespace CONFIG
{
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
}  // namespace CONFIG

#endif  // MOTOR_SPECS_H
