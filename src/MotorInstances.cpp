#include "MotorInstances.h"
#include "Config/TMC5160T_Driver.h"
#include "MotorController.h"

DriverConfig dc1 = {CONFIG::MOTOR::CS1, CONFIG::MOTOR::STEP1, CONFIG::MOTOR::DIR1, CONFIG::MOTOR::EN1};
DriverConfig dc2 = {CONFIG::MOTOR::CS2, CONFIG::MOTOR::STEP2, CONFIG::MOTOR::DIR2, CONFIG::MOTOR::EN2};
DriverConfig dc3 = {CONFIG::MOTOR::CS3, CONFIG::MOTOR::STEP3, CONFIG::MOTOR::DIR3, CONFIG::MOTOR::EN3};
DriverConfig dc4 = {CONFIG::MOTOR::CS4, CONFIG::MOTOR::STEP4, CONFIG::MOTOR::DIR4, CONFIG::MOTOR::EN4};

// Define the motors array
MotorController motors[CONFIG::SYSTEM::NUM_MOTORS] = {MotorController("Motor 1", dc1), MotorController("Motor 2", dc2),
                                                      MotorController("Motor 3", dc3), MotorController("Motor 4", dc4)};

void initializeMotors()
{
    for (uint8_t i = 0; i < CONFIG::SYSTEM::NUM_MOTORS; i++)
    {
        motors[i].begin();
        motors[i].testCommunication();
    }
}