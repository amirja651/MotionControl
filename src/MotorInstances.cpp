#include "MotorInstances.h"

// Define the motors array
MotorController motors[Config::TMC5160T_Driver::NUM_MOTORS] = {
    MotorController("Motor 1", Config::SPI::MOTOR1_CS, Config::TMC5160T_Driver::MOTOR1_STEP_PIN,
                    Config::TMC5160T_Driver::MOTOR1_DIR_PIN, Config::TMC5160T_Driver::MOTOR1_EN_PIN),
    MotorController("Motor 2", Config::SPI::MOTOR2_CS, Config::TMC5160T_Driver::MOTOR2_STEP_PIN,
                    Config::TMC5160T_Driver::MOTOR2_DIR_PIN, Config::TMC5160T_Driver::MOTOR2_EN_PIN),
    MotorController("Motor 3", Config::SPI::MOTOR3_CS, Config::TMC5160T_Driver::MOTOR3_STEP_PIN,
                    Config::TMC5160T_Driver::MOTOR3_DIR_PIN, Config::TMC5160T_Driver::MOTOR3_EN_PIN),
    MotorController("Motor 4", Config::SPI::MOTOR4_CS, Config::TMC5160T_Driver::MOTOR4_STEP_PIN,
                    Config::TMC5160T_Driver::MOTOR4_DIR_PIN, Config::TMC5160T_Driver::MOTOR4_EN_PIN)};

void initializeMotors()
{
    for (uint8_t i = 0; i < Config::TMC5160T_Driver::NUM_MOTORS; i++)
    {
        motors[i].begin();
        Serial.print(F("\nMotor "));
        Serial.print(i + 1);
        Serial.print(F(" - "));
        motors[i].testCommunication();
    }
}