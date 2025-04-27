#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <TMCStepper.h>
#include "Config/TMC5160T_Driver.h"

static constexpr float default_RS = 0.075;

class MotorController
{
public:
    MotorController(uint16_t pinDIR, uint16_t pinSTEP, uint16_t pinEN, uint16_t pinCS, int8_t link_index = -1);
    MotorController(uint16_t pinDIR, uint16_t pinSTEP, uint16_t pinEN, uint16_t pinCS, uint16_t pinMOSI, uint16_t pinMISO,
                    uint16_t pinSCK, int8_t link_index = -1);
    TMC5160Stepper driver;  // TMC5160 driver instance

    void begin();                          // Initialize the motor controller
    void moveForward();                    // Move motor in forward direction
    void moveReverse();                    // Move motor in reverse direction
    void step();                           // Execute single step
    void stop();                           // Stop motor movement
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

    bool isMoving;  // Current movement state
private:
    MotorType motorType;
};

#endif