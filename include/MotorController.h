#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <TMCStepper.h>
#include "Config/Motor_Specs.h"
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
    void     begin();                                   // Initialize the motor controller
    void     moveForward();                             // Move motor in forward direction
    void     moveReverse();                             // Move motor in reverse direction
    void     stop();                                    // Stop motor movement
    void     update();                                  // Update motor state and check status
    uint32_t getDriverStatus();                         // Get current driver status
    void     increaseRunCurrent();                      // Increase motor running current
    void     decreaseRunCurrent();                      // Decrease motor running current
    void     increaseHoldCurrent();                     // Increase motor holding current
    void     decreaseHoldCurrent();                     // Decrease motor holding current
    uint16_t getRunCurrent() const;                     // Get current running current value
    uint16_t getHoldCurrent() const;                    // Get current holding current value
    void     increaseSpeed();                           // Increase motor speed
    void     decreaseSpeed();                           // Decrease motor speed
    void     increaseAcceleration();                    // Increase motor acceleration
    void     decreaseAcceleration();                    // Decrease motor acceleration
    uint16_t getSpeed() const;                          // Get current speed value
    uint16_t getAcceleration() const;                   // Get current acceleration value
    void     printDriverStatus();                       // Print current driver status
    void     printDriverConfig();                       // Print current driver configuration
    int      getTemperature();                          // Get current driver temperature
    void     printTemperature();                        // Print current temperature
    void     toggleStealthChop();                       // Toggle stealth chop mode
    void     setStealthChopMode(bool enable);           // Set stealth chop mode
    bool     diagnoseTMC5160();                         //  Diagnose the TMC5160 driver
    bool     testCommunication();                       // Performs a basic SPI communication test
    uint8_t  transfer(uint8_t data);                    // Performs a single byte SPI transfer
    void     enableDriver();                            // Enable the motor driver by setting EN pin low
    void     disableDriver();                           // Disable the motor driver by setting EN pin high
    void     enableSPI();                               // Enable SPI communication by setting CS pin low
    void     disableSPI();                              // Disable SPI communication by setting CS pin high
    void     resetDriverState();                        // Reset the driver by disabling and re-enabling it
    void     stepHigh();                                //  Set step pin high
    void     stepLow();                                 // Set step pin low
    void     dirHigh();                                 // Set direction pin high (forward)
    void     dirLow();                                  // Set direction pin low (reverse)
    String   motorName() const;                         // Print the name of this motor controller instance
    void     optimizeForPancake();                      // Optimize the motor controller for pancake motor
    void     setCoolStepThreshold(uint32_t threshold);  // Set CoolStep threshold
    void     setStallGuardThreshold(int8_t threshold);  // Set StallGuard threshold
    void     setStallGuardFilter(bool enable);          // Enable/disable StallGuard filter
    void     setSpreadCycle(bool enable);               // Enable/disable SpreadCycle mode
    void     setMicrostepInterpolation(bool enable);    // Enable/disable microstep interpolation
    void     setCurrentScaling(uint8_t scaling);        // Set current scaling factor
    void     setCurrentHoldDelay(uint8_t delay);        // Set current hold delay
    void     setCurrentRunDelay(uint8_t delay);         // Set current run delay
    void     setRampMode(uint8_t mode);                 // Set ramp mode (0: Positioning, 1: Velocity)
    void     setMaxSpeed(uint32_t speed);               // Set maximum speed
    void     setMaxAcceleration(uint32_t accel);        // Set maximum acceleration
    void     setMaxDeceleration(uint32_t decel);        // Set maximum deceleration
    void     enableDiagnostics();                       // Enable advanced diagnostics
    void     disableDiagnostics();                      // Disable advanced diagnostics
    uint32_t getStallGuardValue();                      // Get current StallGuard value
    uint32_t getLoadValue();                            // Get current load value
    bool     isStalled();                               // Check if motor is stalled

private:
    // Internal configuration methods
    void configureDriver();                       // Configure driver parameters
    void setupPins();                             // Setup GPIO pins
    void step();                                  // Execute single step
    bool checkAndReinitializeDriver();            // Check and reinitialize driver if needed
    void handlePowerLoss();                       // Handle power loss situation
    void checkStall();                            // Check for motor stall condition
    void setMovementDirection(bool forward);      // Set movement direction and update state
    void printStatusRegister(uint32_t status);    // Print driver status register
    void printErrorFlags(uint32_t status);        // Print error flags
    void printStallGuardStatus(uint32_t status);  // Print stall guard status
    void printDriverState(uint32_t status);       // Print driver state
    void updateDiagnostics();                     // Update diagnostic information
    void handleStall();                           // Handle stall condition
    void optimizeCurrent();                       // Optimize current based on load
    void checkLoad();                             // Check motor load
    void adjustMicrostepping();                   // Adjust microstepping based on speed

    // Driver instance and state variables
    TMC5160Stepper driver;        // TMC5160 driver instance
    bool           isMoving;      // Current movement state
    bool           direction;     // Current movement direction
    const int      stepDelay;     // Delay between steps
    unsigned long  lastStepTime;  // Timestamp of last step
    int            stepCounter;   // Step counter for status updates

    // Pin assignments
    const uint8_t csPin;    // Chip select pin
    const uint8_t stepPin;  // Step pin
    const uint8_t dirPin;   // Direction pin
    const uint8_t enPin;    // Enable pin

    // Current settings
    uint16_t runCurrent;   // Current running current value
    uint16_t holdCurrent;  // Current holding current value

    // Speed and acceleration settings
    uint16_t speed;         // Current speed in steps per second
    uint16_t acceleration;  // Current acceleration in steps per second squared

    // Temperature monitoring variables
    unsigned long lastTempPrintTime;  // Last temperature print timestamp
    int           lastTemperature;    // Last recorded temperature

    // Instance identification
    String instanceName;  // Name of this motor controller instance

    // Advanced control parameters
    bool     diagnosticsEnabled;      // Whether advanced diagnostics are enabled
    uint32_t coolStepThreshold;       // CoolStep threshold value
    int8_t   stallGuardThreshold;     // StallGuard threshold value
    bool     stallGuardFilter;        // StallGuard filter state
    bool     spreadCycleEnabled;      // SpreadCycle mode state
    bool     microstepInterpolation;  // Microstep interpolation state

    // Advanced current control parameters
    uint8_t currentScaling;    // Current scaling factor
    uint8_t currentHoldDelay;  // Current hold delay
    uint8_t currentRunDelay;   // Current run delay

    // Motion control parameters
    uint8_t  rampMode;         // Current ramp mode
    uint32_t maxSpeed;         // Maximum speed
    uint32_t maxAcceleration;  // Maximum acceleration
    uint32_t maxDeceleration;  // Maximum deceleration

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

#endif