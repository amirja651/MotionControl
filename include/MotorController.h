#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <TMCStepper.h>
#include "Config.h"

/**
 * @brief Structure to hold communication test results
 */
struct CommunicationResult
{
    bool   success;  // Whether the communication test was successful
    String message;  // Status message
};

/**
 * @brief Motor Controller class for managing TMC5160 stepper motor driver
 *
 * This class implements a high-precision controller for the TMC5160 stepper motor driver,
 * optimized for medical-grade applications using pancake motors.
 */
class MotorController
{
public:
    /**
     * @brief Constructor for MotorController
     * @param name Name of this motor controller instance
     * @param csPin Chip select pin for this motor
     * @param stepPin Step pin for this motor
     * @param dirPin Direction pin for this motor
     * @param enPin Enable pin for this motor
     * @param mosiPin MOSI pin for SPI (optional, defaults to Config::SPI::MOSI)
     * @param misoPin MISO pin for SPI (optional, defaults to Config::SPI::MISO)
     * @param sckPin SCK pin for SPI (optional, defaults to Config::SPI::SCK)
     */
    MotorController(String name, uint8_t csPin, uint8_t stepPin, uint8_t dirPin, uint8_t enPin,
                    uint8_t mosiPin = Config::SPI::MOSI, uint8_t misoPin = Config::SPI::MISO,
                    uint8_t sckPin = Config::SPI::SCK);

    // Core motor control methods
    void     begin();            // Initialize the motor controller
    void     moveForward();      // Move motor in forward direction
    void     moveReverse();      // Move motor in reverse direction
    void     stop();             // Stop motor movement
    void     update();           // Update motor state and check status
    uint32_t getDriverStatus();  // Get current driver status

    // Current control methods
    void     increaseRunCurrent();    // Increase motor running current
    void     decreaseRunCurrent();    // Decrease motor running current
    void     increaseHoldCurrent();   // Increase motor holding current
    void     decreaseHoldCurrent();   // Decrease motor holding current
    uint16_t getRunCurrent() const;   // Get current running current value
    uint16_t getHoldCurrent() const;  // Get current holding current value

    // Speed and acceleration control
    void     increaseSpeed();          // Increase motor speed
    void     decreaseSpeed();          // Decrease motor speed
    void     increaseAcceleration();   // Increase motor acceleration
    void     decreaseAcceleration();   // Decrease motor acceleration
    uint16_t getSpeed() const;         // Get current speed value
    uint16_t getAcceleration() const;  // Get current acceleration value

    // Status and configuration methods
    void printDriverStatus();  // Print current driver status
    void printDriverConfig();  // Print current driver configuration

    // Temperature monitoring
    int  getTemperature();    // Get current driver temperature
    void printTemperature();  // Print current temperature

    // StealthChop mode
    void toggleStealthChop();  // Toggle stealth chop mode

    void setStealthChopMode(bool enable);  // Set stealth chop mode

    /**
     * @brief Diagnose the TMC5160 driver
     * @return true if the driver is healthy, false otherwise
     */
    bool diagnoseTMC5160();

    /**
     * @brief Performs a basic SPI communication test
     * @param enableMessage Whether to generate detailed messages
     * @return CommunicationResult containing test status and message
     */
    bool testCommunication();

    /**
     * @brief Performs a single byte SPI transfer
     * @param data The byte to send over SPI
     * @return The byte received from the SPI device
     */
    uint8_t transfer(uint8_t data);

    /**
     * @brief Enable the motor driver by setting EN pin low
     */
    void enableDriver();

    /**
     * @brief Disable the motor driver by setting EN pin high
     */
    void disableDriver();

    /**
     * @brief Enable SPI communication by setting CS pin low
     */
    void enableSPI();

    /**
     * @brief Disable SPI communication by setting CS pin high
     */
    void disableSPI();

    /**
     * @brief Reset the driver by disabling and re-enabling it
     * This method performs a complete reset of the driver state
     */
    void resetDriverState();

    /**
     * @brief Set step pin high
     */
    void stepHigh();

    /**
     * @brief Set step pin low
     */
    void stepLow();

    /**
     * @brief Set direction pin high (forward)
     */
    void dirHigh();

    /**
     * @brief Set direction pin low (reverse)
     */
    void dirLow();

    /**
     * @brief Print the name of this motor controller instance
     * @return The name of this motor controller instance
     */
    String motorName() const;

    /**
     * @brief Optimize the motor controller for pancake motor
     */
    void optimizeForPancake();

    // Advanced motor control methods
    void setCoolStepThreshold(uint32_t threshold);  // Set CoolStep threshold
    void setStallGuardThreshold(int8_t threshold);  // Set StallGuard threshold
    void setStallGuardFilter(bool enable);          // Enable/disable StallGuard filter
    void setSpreadCycle(bool enable);               // Enable/disable SpreadCycle mode
    void setMicrostepInterpolation(bool enable);    // Enable/disable microstep interpolation

    // Advanced current control
    void setCurrentScaling(uint8_t scaling);  // Set current scaling factor
    void setCurrentHoldDelay(uint8_t delay);  // Set current hold delay
    void setCurrentRunDelay(uint8_t delay);   // Set current run delay

    // Motion control
    void setRampMode(uint8_t mode);           // Set ramp mode (0: Positioning, 1: Velocity)
    void setMaxSpeed(uint32_t speed);         // Set maximum speed
    void setMaxAcceleration(uint32_t accel);  // Set maximum acceleration
    void setMaxDeceleration(uint32_t decel);  // Set maximum deceleration

    // Advanced diagnostics
    void     enableDiagnostics();   // Enable advanced diagnostics
    void     disableDiagnostics();  // Disable advanced diagnostics
    uint32_t getStallGuardValue();  // Get current StallGuard value
    uint32_t getLoadValue();        // Get current load value
    bool     isStalled();           // Check if motor is stalled

private:
    // Internal configuration methods
    void configureDriver();                   // Configure driver parameters
    void setupPins();                         // Setup GPIO pins
    void step();                              // Execute single step
    bool checkAndReinitializeDriver();        // Check and reinitialize driver if needed
    void handlePowerLoss();                   // Handle power loss situation
    void checkStall();                        // Check for motor stall condition
    void setMovementDirection(bool forward);  // Set movement direction and update state

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
    const uint8_t mosiPin;  // MOSI pin
    const uint8_t misoPin;  // MISO pin
    const uint8_t sckPin;   // SCK pin

    // Current settings
    uint16_t runCurrent;   // Current running current value
    uint16_t holdCurrent;  // Current holding current value

    // Speed and acceleration settings
    uint16_t speed;         // Current speed in steps per second
    uint16_t acceleration;  // Current acceleration in steps per second squared

    // Status printing helper methods
    void printStatusRegister(uint32_t status);    // Print driver status register
    void printErrorFlags(uint32_t status);        // Print error flags
    void printStallGuardStatus(uint32_t status);  // Print stall guard status
    void printDriverState(uint32_t status);       // Print driver state

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

    // Advanced diagnostics
    void updateDiagnostics();    // Update diagnostic information
    void handleStall();          // Handle stall condition
    void optimizeCurrent();      // Optimize current based on load
    void checkLoad();            // Check motor load
    void adjustMicrostepping();  // Adjust microstepping based on speed
};

#endif