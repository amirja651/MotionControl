#include "ObjectInstances.h"
#include "ArduinoLog.h"
#include "Config/TMC5160T_Driver.h"
#include "MotorController.h"
#include "PIDController.h"

PIDConfig pidConfig = {CONFIG::PID::KP, CONFIG::PID::KI, CONFIG::PID::KD};

#if NUM_MOTORS == 1

MotorController motors[1] = {
    MotorController(CONFIG::MOTOR::DIR1, CONFIG::MOTOR::STEP1, CONFIG::MOTOR::EN1, CONFIG::MOTOR::CS1, 0)};

MAE3Encoder2 encoders2[1] = {MAE3Encoder2(CONFIG::ENCODER::ENC1, CONFIG::ENCODER::ENC1, 0, EncoderResolution::BITS_12)};

PIDController pids[1] = {PIDController(pidConfig)};

#endif

void initializeSystem()
{
    for (uint8_t i = 0; i < NUM_MOTORS; i++)
    {
        motors[i].begin();

        if (!motors[i].testCommunication())
        {
            Log.errorln(F("Motor %d communication test: FAILED"), i + 1);
        }
        else
        {
            Log.noticeln(F("Driver %d firmware version: %d"), i + 1, motors[i].driver.version());

            if (motors[i].driver.sd_mode())
            {
                Log.noticeln(F("Driver %d is hardware configured for Step & Dir mode"), i + 1);
            }

            if (motors[i].driver.drv_enn())
            {
                Log.noticeln(F("Driver %d is not hardware enabled"), i + 1);
            }
        }

        encoders2[i].begin();
        pids[i].begin();
    }

    if (motors[0].isRotational())
    {
        pids[0].setOutputLimits(-180.0, 180.0);  // 180 degrees
    }
    else
    {
        pids[0].setOutputLimits(-15000.0, 15000.0);  // 15mm
    }
    // pids[1].setOutputLimits(-180.0, 180.0);      // 180 degrees
    // pids[2].setOutputLimits(-180.0, 180.0);      // 180 degrees
    // pids[3].setOutputLimits(-15000.0, 15000.0);  // 15mm
}