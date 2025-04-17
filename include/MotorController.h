#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <TMCStepper.h>
#include "Config/Motor_Specs.h"

constexpr uint16_t csPin = 10;
// constexpr uint16_t enPin = 2;
constexpr float r_sense = 0.075;

/**
 * @brief Structure to hold communication test results
 */
struct CommunicationResult
{
    bool   success;  // Whether the communication test was successful
    String message;  // Status message
};

struct DriverConfig
{
    uint8_t csPin;
    uint8_t stepPin;
    uint8_t dirPin;
    uint8_t enPin;
};

/**
 * Motor Controller class for managing TMC5160 stepper motor driver
 * This class implements a high-precision controller for the TMC5160 stepper motor driver,
 * optimized for medical-grade applications using pancake motors.
 */

class MotorController
{
public:
    MotorController(String name, DriverConfig config);
    void   begin();                          // Initialize the motor controller
    void   moveForward();                    // Move motor in forward direction
    void   moveReverse();                    // Move motor in reverse direction
    void   step();                           // Execute single step
    void   stop();                           // Stop motor movement
    void   update();                         // Update motor state and check status
    void   increaseRunCurrent();             // Increase motor running current
    void   decreaseRunCurrent();             // Decrease motor running current
    void   increaseHoldCurrent();            // Increase motor holding current
    void   decreaseHoldCurrent();            // Decrease motor holding current
    void   increaseSpeed();                  // Increase motor speed
    void   decreaseSpeed();                  // Decrease motor speed
    void   increaseAcceleration();           // Increase motor acceleration
    void   decreaseAcceleration();           // Decrease motor acceleration
    void   toggleStealthChop();              // Toggle stealth chop mode
    void   setStealthChopMode(bool enable);  // Set stealth chop mode
    bool   testCommunication();              // Performs a basic SPI communication test
    String motorName() const;                // Print the name of this motor controller instance

private:
    // Internal configuration methods
    void configureDriver();     // Configure driver parameters
    void optimizeForPancake();  // Optimize the motor controller for pancake motor

    // Driver instance and state variables
    TMC5160Stepper driver;  // TMC5160 driver instance

    // Pin assignments
    const uint8_t csPin;    // Chip select pin
    const uint8_t stepPin;  // Step pin
    const uint8_t dirPin;   // Direction pin
    const uint8_t enPin;    // Enable pin

    bool   isMoving;      // Current movement state
    String instanceName;  // Name of this motor controller instance

    // Current settings
    uint16_t runCurrent;   // Current running current value
    uint16_t holdCurrent;  // Current holding current value

    // Speed and acceleration settings
    uint16_t speed;         // Current speed in steps per second
    uint16_t acceleration;  // Current acceleration in steps per second squared

    // Instance identification

    // Advanced control parameters
    uint32_t coolStepThreshold;       // CoolStep threshold value
    int8_t   stallGuardThreshold;     // StallGuard threshold value
    bool     stallGuardFilter;        // StallGuard filter state
    bool     spreadCycleEnabled;      // SpreadCycle mode state
    bool     microstepInterpolation;  // Microstep interpolation state

    // Advanced current control parameters
    uint8_t currentHoldDelay;  // Current hold delay

    // Motion control parameters
    uint8_t  rampMode;         // Current ramp mode
    uint32_t maxSpeed;         // Maximum speed
    uint32_t maxAcceleration;  // Maximum acceleration
    uint32_t maxDeceleration;  // Maximum deceleration

    // Current settings with motor specifications constraints
    static constexpr uint16_t CURRENT_STEP     = 100;  // Current adjustment step size in mA
    static constexpr uint16_t MIN_CURRENT      = 100;  // Minimum allowed current in mA
    static constexpr uint16_t MAX_RUN_CURRENT  = 300;  // Maximum run current in mA (1A max)
    static constexpr uint16_t MAX_HOLD_CURRENT = 300;  // Maximum hold current in mA (0.5A max)

    // Speed and acceleration settings
    static constexpr uint16_t SPEED_STEP = 100;   // Speed adjustment step size in steps/sec
    static constexpr uint16_t ACCEL_STEP = 100;   // Acceleration adjustment step size in steps/sec²
    static constexpr uint16_t MIN_SPEED  = 100;   // Minimum speed in steps/sec
    static constexpr uint16_t MAX_SPEED  = 500;   // Maximum speed in steps/sec
    static constexpr uint16_t MIN_ACCEL  = 100;   // Minimum acceleration in steps/sec²
    static constexpr uint16_t MAX_ACCEL  = 1000;  // Maximum acceleration in steps/sec²
};

#endif