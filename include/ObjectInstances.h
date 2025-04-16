#ifndef MOTOR_INSTANCES_H
#define MOTOR_INSTANCES_H

#include "Config/System_Config.h"
#include "MAE3Encoder.h"
#include "MotorController.h"
#include "PIDController.h"

extern MotorController motors[NUM_MOTORS];
extern MAE3Encoder     encoders[NUM_MOTORS];
extern PIDController   pids[NUM_MOTORS];

// Function to initialize motor instances
void initializeSystem();

#endif  // MOTOR_INSTANCES_H