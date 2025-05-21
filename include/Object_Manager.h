#ifndef OBJECT_MANAGER_H
#define OBJECT_MANAGER_H

#include "PIDController.h"

#define NUM_MOTORS 4

extern PIDController pids[NUM_MOTORS];

void initializeOtherObjects();

#endif  // OBJECT_INSTANCES_H
