#ifndef MOTOR_INSTANCES_H
#define MOTOR_INSTANCES_H

#include "MotorController.h"

// Array of motor controller instances
extern MotorController motors[Config::TMC5160T_Driver::NUM_MOTORS];

// Function to initialize motor instances
void initializeMotors();

#endif  // MOTOR_INSTANCES_H