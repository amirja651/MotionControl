#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <TMCStepper.h>
#include "Config/TMC5160T_Driver.h"

constexpr uint16_t csPin   = 10;
constexpr float    r_sense = 0.075;

struct DriverConfig
{
    uint8_t   csPin;
    uint8_t   stepPin;
    uint8_t   dirPin;
    uint8_t   enPin;
    MotorType motorType;
};

class MotorController
{
public:
    MotorController(String name, DriverConfig config);
    TMC5160Stepper driver;  // TMC5160 driver instance

    void begin();                          // Initialize the motor controller
    void moveForward();                    // Move motor in forward direction
    void moveReverse();                    // Move motor in reverse direction
    void step();                           // Execute single step
    void stop();                           // Stop motor movement
    void update();                         // Update motor state and check status
    void toggleStealthChop();              // Toggle stealth chop mode
    void setStealthChopMode(bool enable);  // Set stealth chop mode
    bool testCommunication();              // Performs a basic SPI communication test
    void configureDriver();                // Configure driver parameters
    void optimizeForPancake();             // Optimize the motor controller for pancake motor
    void optimizeFor11HS13_1004H();        // Optimize the motor controller for 11HS13-1004H motor
    void optimize2();                      // Optimize the motor controller for pancake motor
    void enable();                         // Enable the motor controller
    void disable();                        // Disable the motor controller
    bool isRotational();                   // Get the motor type
    // Pin assignments
    const uint8_t csPin;    // Chip select pin
    const uint8_t stepPin;  // Step pin
    const uint8_t dirPin;   // Direction pin
    const uint8_t enPin;    // Enable pin

    uint16_t runCurrent;       // Current running current value
    uint16_t holdCurrent;      // Current holding current value
    uint16_t speed;            // Current speed in steps per second
    uint16_t acceleration;     // Current acceleration in steps per second squared
    uint32_t maxSpeed;         // Maximum speed
    uint32_t maxAcceleration;  // Maximum acceleration
    uint32_t maxDeceleration;  // Maximum deceleration

    String instanceName;  // Name of this motor controller instance
    bool   isMoving;      // Current movement state
private:
    MotorType motorType;
};

#endif