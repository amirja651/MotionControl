#ifndef MOTOR_INSTANCES_H
#define MOTOR_INSTANCES_H

#include "MAE3Encoder2.h"
#include "MotorController.h"
#include "PIDController.h"

// Number of motors in the system
#define NUM_MOTORS  4
#define MOTOR_NAME1 "Motor 1"
#define MOTOR_NAME2 "Motor 2"
#define MOTOR_NAME3 "Motor 3"
#define MOTOR_NAME4 "Motor 4"

extern MotorController motors[NUM_MOTORS];
extern MAE3Encoder2    encoders2[NUM_MOTORS];
extern PIDController   pids[NUM_MOTORS];

// Function to initialize motor instances
void initializeSystem();

#endif  // MOTOR_INSTANCES_H