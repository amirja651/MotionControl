#ifndef MOTOR_INSTANCES_H
#define MOTOR_INSTANCES_H

#include "Config/System_Config.h"
#include "MotorController.h"

// Array of motor controller instances
extern MotorController motors[CONFIG::SYSTEM::NUM_MOTORS];

// Function to initialize motor instances
void initializeMotors();

#endif  // MOTOR_INSTANCES_H