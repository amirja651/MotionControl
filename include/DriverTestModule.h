#ifndef DRIVER_TEST_MODULE_H
#define DRIVER_TEST_MODULE_H

#include "MotorController.h"

class DriverTestModule
{
public:
    DriverTestModule(MotorController* motors, uint8_t numMotors);

    // Main test function that runs all tests
    bool testAllDrivers();

private:
    // Individual test functions
    bool testBasicCommunication();
    bool testDriverConfiguration();
    bool testDaisyChainCommunication();
    bool testConcurrentOperation();

    MotorController* motors;
    uint8_t          numMotors;
};

#endif  // DRIVER_TEST_MODULE_H