#include "ObjectInstances.h"
#include "ArduinoLog.h"
#include "Config/TMC5160T_Driver.h"
#include "MotorController.h"
#include "PIDController.h"

PIDConfig    pidConfig = {CONFIG::PID::KP, CONFIG::PID::KI, CONFIG::PID::KD};
DriverConfig dc1 = {CONFIG::MOTOR::DIR_A, CONFIG::MOTOR::STEP_A, CONFIG::MOTOR::CS_A, CONFIG::MOTOR::EN_A, MotorType::LINEAR};
DriverConfig dc2 = {CONFIG::MOTOR::DIR_B, CONFIG::MOTOR::STEP_B, CONFIG::MOTOR::CS_B, CONFIG::MOTOR::EN_B, MotorType::ROTATIONAL};
DriverConfig dc3 = {CONFIG::MOTOR::DIR_C, CONFIG::MOTOR::STEP_C, CONFIG::MOTOR::CS_C, CONFIG::MOTOR::EN_C, MotorType::ROTATIONAL};
DriverConfig dc4 = {CONFIG::MOTOR::DIR_D, CONFIG::MOTOR::STEP_D, CONFIG::MOTOR::CS_D, CONFIG::MOTOR::EN_D, MotorType::ROTATIONAL};

#if NUM_MOTORS == 1

MotorController motors[NUM_MOTORS]  = {MotorController(MOTOR_NAME1, dc1)};
MAE3Encoder2  encoders2[NUM_MOTORS] = {MAE3Encoder2(CONFIG::ENCODER::ENC1, CONFIG::ENCODER::ENC1, 0, EncoderResolution::BITS_12)};
PIDController pids[NUM_MOTORS]      = {PIDController(pidConfig)};

#elif NUM_MOTORS == 2

MotorController motors[NUM_MOTORS] = {MotorController(MOTOR_NAME1, dc1), MotorController(MOTOR_NAME2, dc2)};

MAE3Encoder2 encoders2[NUM_MOTORS] = {MAE3Encoder2(CONFIG::ENCODER::ENC1, CONFIG::ENCODER::ENC1, 0, EncoderResolution::BITS_12),
                                      MAE3Encoder2(CONFIG::ENCODER::ENC2, CONFIG::ENCODER::ENC2, 1, EncoderResolution::BITS_12)};

PIDController pids[NUM_MOTORS] = {PIDController(pidConfig), PIDController(pidConfig)};

#elif NUM_MOTORS == 4

MotorController motors[NUM_MOTORS] = {MotorController(MOTOR_NAME1, dc1), MotorController(MOTOR_NAME2, dc2),
                                      MotorController(MOTOR_NAME3, dc3), MotorController(MOTOR_NAME4, dc4)};

MAE3Encoder2 encoders2[NUM_MOTORS] = {MAE3Encoder2(CONFIG::ENCODER::ENC1, CONFIG::ENCODER::ENC1, 0, EncoderResolution::BITS_12),
                                      MAE3Encoder2(CONFIG::ENCODER::ENC2, CONFIG::ENCODER::ENC2, 1, EncoderResolution::BITS_12),
                                      MAE3Encoder2(CONFIG::ENCODER::ENC3, CONFIG::ENCODER::ENC3, 2, EncoderResolution::BITS_12),
                                      MAE3Encoder2(CONFIG::ENCODER::ENC4, CONFIG::ENCODER::ENC4, 3, EncoderResolution::BITS_12)};

PIDController pids[NUM_MOTORS] = {PIDController(pidConfig), PIDController(pidConfig), PIDController(pidConfig),
                                  PIDController(pidConfig)};

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

            /*if (motors[i].driver.sd_mode())
            {
                Log.noticeln(F("Driver %d is hardware configured for Step & Dir mode"), i + 1);
            }

            if (motors[i].driver.drv_enn())
            {
                Log.noticeln(F("Driver %d is not hardware enabled"), i + 1);
            }*/
        }

        encoders2[i].begin();
        pids[i].begin();

        if (i == 0)
        {
            pids[i].setOutputLimits(-15000.0, 15000.0);  // 15mm
        }
        else if (i > 0)
        {
            pids[i].setOutputLimits(-180.0, 180.0);  // 180 degrees
        }
    }
}