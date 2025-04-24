#include "ObjectInstances.h"
#include "ArduinoLog.h"
#include "Config/TMC5160T_Driver.h"
#include "MAE3Encoder.h"
#include "MotorController.h"
#include "PIDController.h"

PIDConfig pidConfig = {CONFIG::PID::KP, CONFIG::PID::KI, CONFIG::PID::KD};

#if NUM_MOTORS == 1

DriverConfig dc1 = {CONFIG::MOTOR::CS1, CONFIG::MOTOR::STEP1, CONFIG::MOTOR::DIR1, CONFIG::MOTOR::EN1, MotorType::LINEAR};

MotorController motors[1] = {MotorController(MOTOR_NAME1, dc1)};

MAE3Encoder2 encoders2[1] = {MAE3Encoder2(CONFIG::ENCODER::ENC1, CONFIG::ENCODER::ENC1, 0, EncoderResolution::BITS_12)};

PIDController pids[1] = {PIDController(pidConfig)};

#elif NUM_MOTORS == 4

DriverConfig dc1 = {CONFIG::MOTOR::CS1, CONFIG::MOTOR::STEP1, CONFIG::MOTOR::DIR1, CONFIG::MOTOR::EN1, MotorType::ROTATIONAL};
DriverConfig dc2 = {CONFIG::MOTOR::CS2, CONFIG::MOTOR::STEP2, CONFIG::MOTOR::DIR2, CONFIG::MOTOR::EN2, MotorType::ROTATIONAL};
DriverConfig dc3 = {CONFIG::MOTOR::CS3, CONFIG::MOTOR::STEP3, CONFIG::MOTOR::DIR3, CONFIG::MOTOR::EN3, MotorType::ROTATIONAL};
DriverConfig dc4 = {CONFIG::MOTOR::CS4, CONFIG::MOTOR::STEP4, CONFIG::MOTOR::DIR4, CONFIG::MOTOR::EN4, MotorType::LINEAR};

MotorController motors[4] = {MotorController(MOTOR_NAME1, dc1), MotorController(MOTOR_NAME2, dc2),
                             MotorController(MOTOR_NAME3, dc3), MotorController(MOTOR_NAME4, dc4)};

MAE3Encoder2 encoders2[4] = {MAE3Encoder2(CONFIG::ENCODER::ENC1, CONFIG::ENCODER::ENC1, 0, EncoderResolution::BITS_12),
                             MAE3Encoder2(CONFIG::ENCODER::ENC2, CONFIG::ENCODER::ENC2, 1, EncoderResolution::BITS_12),
                             MAE3Encoder2(CONFIG::ENCODER::ENC3, CONFIG::ENCODER::ENC3, 2, EncoderResolution::BITS_12),
                             MAE3Encoder2(CONFIG::ENCODER::ENC4, CONFIG::ENCODER::ENC4, 3, EncoderResolution::BITS_10)};

PIDController pids[4] = {PIDController(pidConfig), PIDController(pidConfig), PIDController(pidConfig), PIDController(pidConfig)};

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

    pids[0].setOutputLimits(-180.0, 180.0);  // 180 degrees
    // pids[1].setOutputLimits(-180.0, 180.0);      // 180 degrees
    // pids[2].setOutputLimits(-180.0, 180.0);      // 180 degrees
    // pids[3].setOutputLimits(-15000.0, 15000.0);  // 15mm
}