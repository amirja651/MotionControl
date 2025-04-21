#include "ObjectInstances.h"
#include "ArduinoLog.h"
#include "Config/TMC5160T_Driver.h"
#include "MAE3Encoder.h"
#include "MotorController.h"
#include "PIDController.h"

PIDConfig pidConfig = {CONFIG::PID::KP, CONFIG::PID::KI, CONFIG::PID::KD};

#if NUM_MOTORS == 1

DriverConfig dc1 = {CONFIG::MOTOR::CS1, CONFIG::MOTOR::STEP1, CONFIG::MOTOR::DIR1, CONFIG::MOTOR::EN1};

MotorController motors[1] = {MotorController(MOTOR_NAME1, dc1)};

MAE3Encoder encoders[1] = {MAE3Encoder(CONFIG::ENCODER::ENC1, CONFIG::ENCODER::ENC1)};

PIDController pids[1] = {PIDController(pidConfig)};

#elif NUM_MOTORS == 2

DriverConfig dc1 = {CONFIG::MOTOR::CS1, CONFIG::MOTOR::STEP1, CONFIG::MOTOR::DIR1, CONFIG::MOTOR::EN1};
DriverConfig dc2 = {CONFIG::MOTOR::CS2, CONFIG::MOTOR::STEP2, CONFIG::MOTOR::DIR2, CONFIG::MOTOR::EN2};

MotorController motors[2] = {MotorController(MOTOR_NAME1, dc1), MotorController(MOTOR_NAME2, dc2)};

MAE3Encoder encoders[2] = {MAE3Encoder(CONFIG::ENCODER::ENC1, CONFIG::ENCODER::ENC1),
                           MAE3Encoder(CONFIG::ENCODER::ENC2, CONFIG::ENCODER::ENC2)};

PIDController pids[2] = {PIDController(pidConfig), PIDController(pidConfig)};

#elif NUM_MOTORS == 3

DriverConfig dc1 = {CONFIG::MOTOR::CS1, CONFIG::MOTOR::STEP1, CONFIG::MOTOR::DIR1, CONFIG::MOTOR::EN1};
DriverConfig dc2 = {CONFIG::MOTOR::CS2, CONFIG::MOTOR::STEP2, CONFIG::MOTOR::DIR2, CONFIG::MOTOR::EN2};
DriverConfig dc3 = {CONFIG::MOTOR::CS3, CONFIG::MOTOR::STEP3, CONFIG::MOTOR::DIR3, CONFIG::MOTOR::EN3};

MotorController motors[3] = {MotorController(MOTOR_NAME1, dc1), MotorController(MOTOR_NAME2, dc2),
                             MotorController(MOTOR_NAME3, dc3)};

MAE3Encoder encoders[3] = {MAE3Encoder(CONFIG::ENCODER::ENC1, CONFIG::ENCODER::ENC1),
                           MAE3Encoder(CONFIG::ENCODER::ENC2, CONFIG::ENCODER::ENC2),
                           MAE3Encoder(CONFIG::ENCODER::ENC3, CONFIG::ENCODER::ENC3)};

PIDController pids[3] = {PIDController(pidConfig), PIDController(pidConfig), PIDController(pidConfig)};

#elif NUM_MOTORS == 4

DriverConfig dc1 = {CONFIG::MOTOR::CS1, CONFIG::MOTOR::STEP1, CONFIG::MOTOR::DIR1, CONFIG::MOTOR::EN1};
DriverConfig dc2 = {CONFIG::MOTOR::CS2, CONFIG::MOTOR::STEP2, CONFIG::MOTOR::DIR2, CONFIG::MOTOR::EN2};
DriverConfig dc3 = {CONFIG::MOTOR::CS3, CONFIG::MOTOR::STEP3, CONFIG::MOTOR::DIR3, CONFIG::MOTOR::EN3};
DriverConfig dc4 = {CONFIG::MOTOR::CS4, CONFIG::MOTOR::STEP4, CONFIG::MOTOR::DIR4, CONFIG::MOTOR::EN4};

MotorController motors[4] = {MotorController(MOTOR_NAME1, dc1), MotorController(MOTOR_NAME2, dc2),
                             MotorController(MOTOR_NAME3, dc3), MotorController(MOTOR_NAME4, dc4)};

MAE3Encoder encoders[4] = {MAE3Encoder(CONFIG::ENCODER::ENC1, CONFIG::ENCODER::ENC1),
                           MAE3Encoder(CONFIG::ENCODER::ENC2, CONFIG::ENCODER::ENC2),
                           MAE3Encoder(CONFIG::ENCODER::ENC3, CONFIG::ENCODER::ENC3),
                           MAE3Encoder(CONFIG::ENCODER::ENC4, CONFIG::ENCODER::ENC4)};

PIDController pids[4] = {PIDController(pidConfig), PIDController(pidConfig), PIDController(pidConfig),
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
            Log.errorln(F("Please check the motor connections and try again."));

            while (!motors[i].testCommunication())
                ;
        }

        Log.noticeln(F("Driver firmware version: %d"), motors[i].driver.version());

        if (motors[i].driver.sd_mode())
        {
            Log.noticeln(F("Driver is hardware configured for Step & Dir mode"));
        }

        if (motors[i].driver.drv_enn())
        {
            Log.noticeln(F("Driver is not hardware enabled"));
        }

        encoders[i].begin();

        pids[i].begin();
        pids[i].pid->SetOutputLimits(-180.0, 180.0);
    }
}