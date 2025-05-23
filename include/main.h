#include "CLI_Manager.h"
#include "ESP32_Manager.h"
#include "MAE3Encoder.h"
#include "Motor_Manager.h"
#include "Position_Storage.h"
#include "UnitConversion.h"
#include "esp_task_wdt.h"
#include <SPI.h>
#include <inttypes.h>

enum motorState
{
    STOPPED,
    MOVING,
    ERROR
};

constexpr double ROTATIONAL_POSITION_MIN = 0.1f;
constexpr double ROTATIONAL_POSITION_MAX = 359.9f;

constexpr double ROTATIONAL_OUTPUT_LIMIT_MIN = -180.0;
constexpr double ROTATIONAL_OUTPUT_LIMIT_MAX = 180.0;

constexpr double LINEAR_LOWER_LIMIT_PX = 550.0;
constexpr double LINEAR_UPPER_LIMIT_PX = 880.0;
constexpr double LINEAR_OFFSET_PX      = 680.0;

constexpr float ROTATIONAL_THRESHOLD = 0.05f;
constexpr float LINEAR_THRESHOLD     = 0.05f;

static const uint16_t ENC_A = 36;
static const uint16_t ENC_B = 39;
static const uint16_t ENC_C = 34;
static const uint16_t ENC_D = 35;

const String errorMotorNumberIsRequired                = "ERROR: Motor number (-n) requires a value";
const String errorMotorNumberIsInvalid                 = "ERROR: Invalid motor number. Must be between 1 and 4";
const String errorTheCommandIsOnlyValidForLinearMotors = "ERROR: The command is only valid for linear motors";

uint8_t       motor_index                  = 0;
bool          is_set_motor_number          = false;
double        linear_Lower_limit_um        = LINEAR_LOWER_LIMIT_PX * UM_PER_PIXEL;  // Lower limit in pixels
double        linear_upper_limit_um        = LINEAR_UPPER_LIMIT_PX * UM_PER_PIXEL;  // Upper limit in pixels
double        linear_offset_um             = LINEAR_OFFSET_PX * UM_PER_PIXEL;       // Offset in pixels
double        target[NUM_MOTORS]           = {0, 0, 0, 0};
double        current_position[NUM_MOTORS] = {0, 0, 0, 0};
uint64_t      last_pulse[NUM_MOTORS]       = {0, 0, 0, 0};
bool          command_received[NUM_MOTORS] = {false, false, false, false};

// Buffer for storing output
char outputBuffer[200];

// Command history support
#define HISTORY_SIZE 10
String commandHistory[HISTORY_SIZE];
int    historyCount     = 0;
int    historyIndex     = -1;  // -1 means not navigating
double distanceToTarget = 0;

MAE3Encoder encoders[NUM_MOTORS] = {MAE3Encoder(ENC_A, 0), MAE3Encoder(ENC_B, 1), MAE3Encoder(ENC_C, 2), MAE3Encoder(ENC_D, 3)};
motorState  motorLastState[NUM_MOTORS] = {motorState::STOPPED, motorState::STOPPED, motorState::STOPPED, motorState::STOPPED};

// Task handles
TaskHandle_t encoderUpdateTaskHandle = NULL;
TaskHandle_t motorUpdateTaskHandle   = NULL;
TaskHandle_t serialReadTaskHandle    = NULL;
TaskHandle_t serialPrintTaskHandle   = NULL;

void    encoderUpdateTask(void* pvParameters);
void    motorUpdateTask(void* pvParameters);
void    serialReadTask(void* pvParameters);
void    serialPrintTask(void* pvParameters);
void    motorStopAndSavePosition();
double  getShortestAngularDistance(double current, double target);
double  getSignedPositionError(double current, double target);
void    rotationalMotorUpdate();
void    linearMotorUpdate();
void    printSerial();
void    setTarget(double position);
double  getTarget();
bool    isValidMotorIndex(uint8_t motorIndex);
bool    validationInputAndSetMotorIndex(String motorNumber);
uint8_t getMotorIndex();
void    setMotorIndex(uint8_t motorIndex);
void    resetCommandReceived();
bool    validationInputAndSetLinearOffset(String linearOffsetStr);
void    setLinearOffset(double offsetUm);
bool    validationInputAndSetLinearLowerLimit(String lowerLimitPx);
bool    validationInputAndSetLinearUpperLimit(String upperLimitPx);
void    setLowerLimits(double lowerLimit);
void    setUpperLimits(double upperLimit);
bool    validationInputAndSetTarget(String targetStr);
void    clearScreen();
