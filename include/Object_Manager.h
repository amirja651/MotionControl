#ifndef OBJECT_MANAGER_H
#define OBJECT_MANAGER_H

#include "MAE3Encoder2.h"
#include "PIDController.h"

#define NUM_MOTORS 4

extern MAE3Encoder2  encoders2[NUM_MOTORS];
extern PIDController pids[NUM_MOTORS];

void initializeOtherObjects();

#endif  // OBJECT_INSTANCES_H
