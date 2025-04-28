#include "ObjectInstances.h"
#include "ArduinoLog.h"
#include "Config/TMC5160T_Driver.h"
#include "MAE3Encoder.h"
#include "MotorController.h"
#include "PIDController.h"

PIDConfig pidConfig = {_KP, _KI, _KD};

DriverConfig dc1 = {CS_A, STEP_A, DIR_A, EN_A, MotorType::LINEAR, 0};
DriverConfig dc2 = {CS_B, STEP_B, DIR_B, EN_A, MotorType::LINEAR, 1};

MotorController motors[2] = {MotorController(MOTOR_NAME1, dc1), MotorController(MOTOR_NAME2, dc2)};

MAE3Encoder2 encoders2[2] = {MAE3Encoder2(ENC_A, ENC_A, 0, EncoderResolution::BITS_12),
                             MAE3Encoder2(ENC_B, ENC_B, 1, EncoderResolution::BITS_12)};

PIDController pids[2] = {PIDController(pidConfig), PIDController(pidConfig)};

void initializeSystem()
{
    for (uint8_t i = 0; i < NUM_MOTORS; i++)
    {
        motors[i].begin();
        delay(1000);
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
        delay(1000);
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