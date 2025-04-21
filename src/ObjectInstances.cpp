#include "ObjectInstances.h"
#include "Config/TMC5160T_Driver.h"
#include "MAE3Encoder.h"
#include "MotorController.h"
#include "PIDController.h"

PIDConfig pidConfig = {CONFIG::PID::KP, CONFIG::PID::KI, CONFIG::PID::KD};

#if NUM_MOTORS == 1

DriverConfig dc1 = {CONFIG::MOTOR::CS1, CONFIG::MOTOR::STEP1, CONFIG::MOTOR::DIR1, CONFIG::MOTOR::EN1};

MotorController motors[NUM_MOTORS] = {MotorController(MOTOR_NAME1, dc1)};

MAE3Encoder encoders[NUM_MOTORS] = {MAE3Encoder(CONFIG::ENCODER::ENC1, CONFIG::ENCODER::ENC1)};

PIDController pids[NUM_MOTORS] = {PIDController(&motors[0], &encoders[0], pidConfig)};

#elif NUM_MOTORS == 2

DriverConfig dc1 = {CONFIG::MOTOR::CS1, CONFIG::MOTOR::STEP1, CONFIG::MOTOR::DIR1, CONFIG::MOTOR::EN1};
DriverConfig dc2 = {CONFIG::MOTOR::CS2, CONFIG::MOTOR::STEP2, CONFIG::MOTOR::DIR2, CONFIG::MOTOR::EN2};

MotorController motors[NUM_MOTORS] = {MotorController(MOTOR_NAME1, dc1), MotorController(MOTOR_NAME2, dc2)};

MAE3Encoder encoders[NUM_MOTORS] = {MAE3Encoder(CONFIG::ENCODER::ENC1, CONFIG::ENCODER::ENC1),
                                    MAE3Encoder(CONFIG::ENCODER::ENC2, CONFIG::ENCODER::ENC2)};

PIDController pids[NUM_MOTORS] = {PIDController(&motors[0], &encoders[0], pidConfig),
                                  PIDController(&motors[1], &encoders[1], pidConfig)};

#elif NUM_MOTORS == 3

DriverConfig dc1 = {CONFIG::MOTOR::CS1, CONFIG::MOTOR::STEP1, CONFIG::MOTOR::DIR1, CONFIG::MOTOR::EN1};
DriverConfig dc2 = {CONFIG::MOTOR::CS2, CONFIG::MOTOR::STEP2, CONFIG::MOTOR::DIR2, CONFIG::MOTOR::EN2};
DriverConfig dc3 = {CONFIG::MOTOR::CS3, CONFIG::MOTOR::STEP3, CONFIG::MOTOR::DIR3, CONFIG::MOTOR::EN3};

MotorController motors[NUM_MOTORS] = {MotorController(MOTOR_NAME1, dc1), MotorController(MOTOR_NAME2, dc2),
                                      MotorController(MOTOR_NAME3, dc3)};

MAE3Encoder encoders[NUM_MOTORS] = {MAE3Encoder(CONFIG::ENCODER::ENC1, CONFIG::ENCODER::ENC1),
                                    MAE3Encoder(CONFIG::ENCODER::ENC2, CONFIG::ENCODER::ENC2),
                                    MAE3Encoder(CONFIG::ENCODER::ENC3, CONFIG::ENCODER::ENC3)};

PIDController pids[NUM_MOTORS] = {PIDController(&motors[0], &encoders[0], pidConfig),
                                  PIDController(&motors[1], &encoders[1], pidConfig),
                                  PIDController(&motors[2], &encoders[2], pidConfig)};

#elif NUM_MOTORS == 4

DriverConfig dc1 = {CONFIG::MOTOR::CS1, CONFIG::MOTOR::STEP1, CONFIG::MOTOR::DIR1, CONFIG::MOTOR::EN1};
DriverConfig dc2 = {CONFIG::MOTOR::CS2, CONFIG::MOTOR::STEP2, CONFIG::MOTOR::DIR2, CONFIG::MOTOR::EN2};
DriverConfig dc3 = {CONFIG::MOTOR::CS3, CONFIG::MOTOR::STEP3, CONFIG::MOTOR::DIR3, CONFIG::MOTOR::EN3};
DriverConfig dc4 = {CONFIG::MOTOR::CS4, CONFIG::MOTOR::STEP4, CONFIG::MOTOR::DIR4, CONFIG::MOTOR::EN4};

MotorController motors[NUM_MOTORS] = {MotorController(MOTOR_NAME1, dc1), MotorController(MOTOR_NAME2, dc2),
                                      MotorController(MOTOR_NAME3, dc3), MotorController(MOTOR_NAME4, dc4)};

MAE3Encoder encoders[NUM_MOTORS] = {MAE3Encoder(CONFIG::ENCODER::ENC1, CONFIG::ENCODER::ENC1),
                                    MAE3Encoder(CONFIG::ENCODER::ENC2, CONFIG::ENCODER::ENC2),
                                    MAE3Encoder(CONFIG::ENCODER::ENC3, CONFIG::ENCODER::ENC3),
                                    MAE3Encoder(CONFIG::ENCODER::ENC4, CONFIG::ENCODER::ENC4)};

PIDController pids[NUM_MOTORS] = {
    PIDController(&motors[0], &encoders[0], pidConfig), PIDController(&motors[1], &encoders[1], pidConfig),
    PIDController(&motors[2], &encoders[2], pidConfig), PIDController(&motors[3], &encoders[3], pidConfig)};

#endif

void initializeSystem()
{
    for (uint8_t i = 0; i < NUM_MOTORS; i++)
    {
        motors[i].begin();
        bool testOK = motors[i].testCommunication();

        Serial.print((testOK ? "" : "Motor " + String(i + 1) + " communication test: FAILED\n"));

        encoders[i].begin();

        pids[i].begin();
        pids[i].setPositionThreshold(0.5f);  // 0.5 degrees threshold
        pids[i].setTarget(5.0f);
        pids[i].setOutputLimits(-180.0f, 180.0f);
    }
}