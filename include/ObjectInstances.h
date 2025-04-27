#ifndef MOTOR_INSTANCES_H
#define MOTOR_INSTANCES_H

#include "MAE3Encoder2.h"
#include "MotorController.h"
#include "PIDController.h"

// Number of motors in the system
#define NUM_MOTORS 1

extern MotorController motors[NUM_MOTORS];
extern MAE3Encoder2    encoders2[NUM_MOTORS];
extern PIDController   pids[NUM_MOTORS];

// Function to initialize motor instances
void initializeSystem();

#endif  // MOTOR_INSTANCES_H