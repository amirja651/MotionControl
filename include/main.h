#include "CLI_Manager.h"
#include "ESP32_Manager.h"
#include "MAE3Encoder.h"
static const uint16_t ENC_A       = 36;
static const uint16_t ENC_B       = 39;
static const uint16_t ENC_C       = 34;
static const uint16_t ENC_D       = 35;
MAE3Encoder           encoders[4] = {MAE3Encoder(ENC_A, 0), MAE3Encoder(ENC_B, 1), MAE3Encoder(ENC_C, 2), MAE3Encoder(ENC_D, 3)};
#include "Motor_Manager.h"
#include "Position_Storage.h"
#include "esp_task_wdt.h"
#include <SPI.h>
#include <inttypes.h>

constexpr float UM_PER_PIXEL = 5.2f;

const String errorMotorNumberIsRequired                = "ERROR: Motor number (-n) requires a value";
const String errorMotorNumberIsInvalid                 = "ERROR: Invalid motor number. Must be between 1 and 4";
const String errorTheCommandIsOnlyValidForLinearMotors = "ERROR: The command is only valid for linear motors";

int32_t last_pulse[NUM_MOTORS]       = {0, 0, 0, 0};
bool    is_set_motor_number          = false;
bool    command_received[NUM_MOTORS] = {false, false, false, false};
bool    isVeryShortDistance          = false;

// Command history support
#define HISTORY_SIZE 10
String commandHistory[HISTORY_SIZE];
int    historyCount = 0;
int    historyIndex = -1;  // -1 means not navigating

// Task handles
TaskHandle_t encoderUpdateTaskHandle = NULL;
TaskHandle_t motorUpdateTaskHandle   = NULL;
TaskHandle_t serialReadTaskHandle    = NULL;
TaskHandle_t serialPrintTaskHandle   = NULL;

uint8_t motor_index = 0;

inline uint8_t getMotorIndex()
{
    return motor_index;
}
void setMotorIndex(uint8_t motorIndex)
{
    if (motorIndex >= NUM_MOTORS)
    {
        Serial.println(F("ERROR: Invalid motor index!"));
        return;
    }

    motor_index = motorIndex;
}

float target[NUM_MOTORS] = {0};

inline float getTarget()
{
    return target[motor_index];
}
inline void setTarget(float position)
{
    target[motor_index] = position;
}

inline void clearScreen()
{
    printf("\e[1;1H\e[2J");  // clear screen
}

void  encoderUpdateTask(void* pvParameters);
float wrapAngle180(float value);
float calculateSignedPositionError(float current_pos);
void  motorUpdateTask(void* pvParameters);
void  serialReadTask(void* pvParameters);
void  serialPrintTask(void* pvParameters);
void  motorStopAndSavePosition();
bool  validationInputAndSetMotorIndex(String motorNumber);
void  setMotorIndex(uint8_t motorIndex);
void  resetAllCommandReceivedFlags();
bool  validateAndSetTargetPosition(String targetStr);
void  printSerial();