#include "ObjectInstances.h"
#include "ArduinoLog.h"
#include "Config/TMC5160T_Driver.h"
#include "DriverTestModule.h"
#include "MotorController.h"
#include "PIDController.h"

PIDConfig pidConfig = {_KP, _KI, _KD};

MotorController motors[NUM_MOTORS] = {
    MotorController(DIR_A, STEP_A, EN_A, CS_A, 0), MotorController(DIR_B, STEP_B, EN_B, CS_A, 1),
    MotorController(DIR_C, STEP_C, EN_C, CS_A, 2), MotorController(DIR_D, STEP_D, EN_D, CS_A, 3)};

MAE3Encoder2 encoders2[NUM_MOTORS] = {
    MAE3Encoder2(ENC_A, ENC_A, 0, EncoderResolution::BITS_12), MAE3Encoder2(ENC_B, ENC_B, 1, EncoderResolution::BITS_12),
    MAE3Encoder2(ENC_C, ENC_C, 2, EncoderResolution::BITS_12), MAE3Encoder2(ENC_D, ENC_D, 3, EncoderResolution::BITS_12)};

PIDController pids[NUM_MOTORS] = {PIDController(pidConfig), PIDController(pidConfig), PIDController(pidConfig),
                                  PIDController(pidConfig)};

void initializeSystem()
{
    Log.noticeln(F("Initializing system with %d motors..."), NUM_MOTORS);

    // Create and run driver tests
    DriverTestModule driverTest(motors, NUM_MOTORS);
    if (!driverTest.testAllDrivers())
    {
        Log.errorln(F("Driver test failed. System initialization aborted."));
        return;
    }

    // Initialize motors and encoders
    for (uint8_t i = 0; i < NUM_MOTORS; i++)
    {
        Log.noticeln(F("Initializing motor %d..."), i + 1);

        // Initialize motor with increased delay
        motors[i].begin();
        delay(500);  // Increased delay between motor initializations

        // Initialize encoder with delay
        // delay(200);
        // encoders2[i].begin();

        // Initialize PID with delay
        // delay(200);
        // pids[i].begin();

        // Set PID limits with delay
        // delay(100);
        /*{
             pids[i].setOutputLimits(-180.0, 180.0);  // 180 degrees
         }
         else
         {
             pids[i].setOutputLimits(-15000.0, 15000.0);  // 15mm
         }*/

        Log.noticeln(F("Motor %d initialization complete"), i + 1);
    }

    Log.noticeln(F("System initialization complete"));

    while (true)
    {
        delay(1000);
    }
}