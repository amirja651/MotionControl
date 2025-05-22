#include "CLI_Manager.h"
#include "Constants.h"
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

static const uint16_t ENC_A = 36;
static const uint16_t ENC_B = 39;
static const uint16_t ENC_C = 34;
static const uint16_t ENC_D = 35;

const String errorMotorNumberIsRequired = "ERROR: Motor number (-n) requires a value";
const String errorMotorNumberIsInvalid  = "ERROR: Invalid motor number. Must be between 1 and 4";

uint8_t       _motorIndex                  = 0;
unsigned long _lastUpdateTime              = 0;
bool          _isGetMotorNumber            = false;
double        _linearLowerLimitUm          = LINEAR_LOWER_LIMIT_PX * UM_PER_PIXEL;  // Lower limit in pixels
double        _linearUpperLimitUm          = LINEAR_UPPER_LIMIT_PX * UM_PER_PIXEL;  // Upper limit in pixels
double        _linearOffsetUm              = LINEAR_OFFSET_PX * UM_PER_PIXEL;       // Offset in pixels
double        _target[NUM_MOTORS]          = {0, 0, 0, 0};
double        _currentPosition[NUM_MOTORS] = {0, 0, 0, 0};
double        _positionError[NUM_MOTORS]   = {0, 0, 0, 0};
uint64_t      lastPulseWidthUs[NUM_MOTORS] = {0, 0, 0, 0};
bool          _commandReceived[NUM_MOTORS] = {false, false, false, false};

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
void    motorStopAndSavePosition(double currentPosition);
double  getShortestAngularDistance(double current, double target);
double  getSignedPositionError(double current, double target);
void    rotationalMotorUpdate();
void    linearMotorUpdate();
void    printSerial();
void    setTarget(double position);
double  getTarget();
double  getPositionError();
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

void setup()
{
    SPI.begin();
    Serial.begin(115200);
    esp_task_wdt_init(10, true);  // 10 seconds timeout, panic reset enabled
    esp_task_wdt_add(NULL);       // Add the current task (setup)
    esp_log_level_set("*", ESP_LOG_VERBOSE);
    delay(1000);
    while (!Serial)
    {
        delay(10);
    }

    printSystemInfo();
    initializeCLI();
    initPositionStorage();  // Initialize EEPROM
    initializeDriversAndTest();

    encoders[0].begin();
    encoders[1].begin();
    encoders[2].begin();
    encoders[3].begin();

    xTaskCreatePinnedToCore(encoderUpdateTask, "EncoderUpdateTask", 2048, NULL, 5, &encoderUpdateTaskHandle, 1);  // Core 1
    xTaskCreatePinnedToCore(motorUpdateTask, "MotorUpdateTask", 2048, NULL, 3, &motorUpdateTaskHandle, 0);        // Core 0
    xTaskCreatePinnedToCore(serialReadTask, "SerialReadTask", 2048, NULL, 2, &serialReadTaskHandle, 0);           // Core 0
    xTaskCreatePinnedToCore(serialPrintTask, "SerialPrintTask", 2048, NULL, 1, &serialPrintTaskHandle, 0);        // Core 0

    esp_task_wdt_add(encoderUpdateTaskHandle);  // Register with WDT
    esp_task_wdt_add(motorUpdateTaskHandle);    // Register with WDT
    esp_task_wdt_add(serialReadTaskHandle);     // Register with WDT
    esp_task_wdt_add(serialPrintTaskHandle);    // Register with WDT

    // printf("\e[1;1H\e[2J");  // clear screen
}

void loop()
{
    esp_task_wdt_reset();
    vTaskDelay(100);  // Add a short delay to prevent WDT reset
}

void encoderUpdateTask(void* pvParameters)
{
    const TickType_t xFrequency    = pdMS_TO_TICKS(1);
    TickType_t       xLastWakeTime = xTaskGetTickCount();

    while (1)
    {
        uint8_t motorIndex = getMotorIndex();

        for (int i = 0; i < NUM_MOTORS; i++)
        {
            if (i == motorIndex)
            {
                if (!encoders[i].isEnabled())
                {
                    encoders[i].enable();
                }
            }
            else
            {
                if (encoders[i].isEnabled())
                {
                    encoders[i].disable();
                }
            }
        }

        encoders[motorIndex].processPWM();

        unsigned long now = millis();
        if (now - _lastUpdateTime >= 2)  // 2ms
        {
            encoders[motorIndex].update();
            _lastUpdateTime = now;
        }

        esp_task_wdt_reset();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void motorUpdateTask(void* pvParameters)
{
    const TickType_t xFrequency    = pdMS_TO_TICKS(5);
    TickType_t       xLastWakeTime = xTaskGetTickCount();

    while (1)
    {
        uint8_t motorIndex = getMotorIndex();

        if (!_commandReceived[motorIndex])
            continue;

        if (communicationTest[motorIndex] == "FAILED")
        {
            Serial.println(F("ERROR: Motor communication failed!"));
            continue;
        }

        // Only for rotational motors
        if (motorType[motorIndex] == MotorType::ROTATIONAL)
        {
            rotationalMotorUpdate();
        }
        else
        {
            linearMotorUpdate();
        }

        esp_task_wdt_reset();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void serialReadTask(void* pvParameters)
{
    const TickType_t xFrequency    = pdMS_TO_TICKS(100);
    TickType_t       xLastWakeTime = xTaskGetTickCount();
    String           inputBuffer   = "";
    String           lastInput     = "";

    while (1)
    {
        while (Serial.available())
        {
            char c = Serial.read();
            // Handle escape sequences for arrow keys
            static int escState = 0;  // 0: normal, 1: got '\x1b', 2: got '['
            if (escState == 0 && c == '\x1b')
            {
                escState = 1;
                continue;
            }
            if (escState == 1 && c == '[')
            {
                escState = 2;
                continue;
            }
            if (escState == 2)
            {
                if (c == 'A')
                {  // Up arrow
                    if (historyCount > 0)
                    {
                        if (historyIndex < historyCount - 1)
                            historyIndex++;
                        inputBuffer = commandHistory[(historyCount - 1 - historyIndex) % HISTORY_SIZE];
                        // Clear current line and print inputBuffer
                        Serial.print(F("\r> "));
                        Serial.print(inputBuffer);
                        Serial.print(F("           \r> "));  // Overwrite any old chars
                        Serial.print(inputBuffer);
                    }
                    escState = 0;
                    continue;
                }
                else if (c == 'B')
                {  // Down arrow
                    if (historyCount > 0 && historyIndex > 0)
                    {
                        historyIndex--;
                        inputBuffer = commandHistory[(historyCount - 1 - historyIndex) % HISTORY_SIZE];
                        Serial.print(F("\r> "));
                        Serial.print(inputBuffer);
                        Serial.print(F("           \r> "));
                        Serial.print(inputBuffer);
                    }
                    else if (historyIndex == 0)
                    {
                        historyIndex = -1;
                        inputBuffer  = lastInput;
                        Serial.print(F("\r> "));
                        Serial.print(inputBuffer);
                        Serial.print(F("           \r> "));
                        Serial.print(inputBuffer);
                    }
                    escState = 0;
                    continue;
                }
                escState = 0;
                continue;
            }
            // Handle Enter
            if (c == '\n')
            {
                if (inputBuffer.length() > 0)
                {
                    Serial.println();
                    Serial.print(F("# "));
                    Serial.println(inputBuffer.c_str());
                    cli.parse(inputBuffer);
                    // Add to history
                    if (historyCount == 0 || commandHistory[(historyCount - 1) % HISTORY_SIZE] != inputBuffer)
                    {
                        commandHistory[historyCount % HISTORY_SIZE] = inputBuffer;
                        if (historyCount < HISTORY_SIZE)
                            historyCount++;
                        else
                            historyCount = HISTORY_SIZE;
                    }
                    historyIndex = -1;
                    lastInput    = "";
                    inputBuffer  = "";  // Clear the buffer
                }
            }
            else if (c == '\b' || c == 127)  // Handle backspace
            {
                if (inputBuffer.length() > 0)
                {
                    inputBuffer.remove(inputBuffer.length() - 1);
                    Serial.print(F("\b \b"));
                }
            }
            else if (c == 'u')
            {
                // Manual step left (reverse)
                motorMoveReverse(getMotorIndex());
            }
            else if (c == 'i')
            {
                // Manual step right (forward)
                motorMoveForward(getMotorIndex());
            }
            else
            {
                inputBuffer += c;  // Add character to buffer
                Serial.print(c);
                if (historyIndex == -1)
                    lastInput = inputBuffer;
            }
            esp_task_wdt_reset();
        }

        if (cli.available())
        {
            Command c = cli.getCmd();

            if (c == cmdMotor)
            {
                // Get motor number
                if (c.getArgument("n").isSet())
                {
                    String motorNumStr = c.getArgument("n").getValue();

                    resetCommandReceived();

                    if (!validationInputAndSetMotorIndex(motorNumStr))
                    {
                        continue;
                    }
                }
                else
                {
                    Serial.println(errorMotorNumberIsRequired);
                    continue;
                }

                // Handle Get motor position command
                if (c.getArgument("c").isSet())
                {
                    if (!_isGetMotorNumber)
                    {
                        Serial.println(errorMotorNumberIsRequired);
                        continue;
                    }

                    _isGetMotorNumber  = false;
                    uint8_t motorIndex = getMotorIndex();

                    Serial.print(F("*"));
                    Serial.print(motorIndex + 1);
                    Serial.print(F("#"));

                    if (motorType[motorIndex] == MotorType::ROTATIONAL)
                    {
                        Serial.print(encoders[motorIndex].getPositionDegrees());
                    }
                    else
                    {
                        Serial.print(encoders[motorIndex].getPositionUM());
                    }

                    Serial.println(F("#"));
                    continue;
                }

                // Handle stop command
                if (c.getArgument("s").isSet())
                {
                    if (!_isGetMotorNumber)
                    {
                        Serial.println(errorMotorNumberIsRequired);
                        continue;
                    }

                    _isGetMotorNumber  = false;
                    uint8_t motorIndex = getMotorIndex();

                    resetCommandReceived();
                    motorStop(motorIndex);

                    Serial.print(F("Motor "));
                    Serial.print(motorIndex + 1);
                    Serial.println(F(" stopped"));
                    continue;
                }

                // Handle offset command
                if (c.getArgument("o").isSet())
                {
                    if (!_isGetMotorNumber)
                    {
                        Serial.println(errorMotorNumberIsRequired);
                        continue;
                    }

                    _isGetMotorNumber = false;

                    // Only allow for Motor 1
                    uint8_t motorIndex = getMotorIndex();

                    if (motorType[motorIndex] != MotorType::LINEAR)
                    {
                        Serial.println(F("ERROR: Offset command is only valid for linear motors"));
                        continue;
                    }

                    String linearOffsetStr = c.getArgument("0").getValue();

                    if (!validationInputAndSetLinearOffset(linearOffsetStr))
                    {
                        continue;
                    }
                }

                // Handle lower limit command
                if (c.getArgument("lo").isSet())
                {
                    if (!_isGetMotorNumber)
                    {
                        Serial.println(errorMotorNumberIsRequired);
                        continue;
                    }

                    _isGetMotorNumber = false;

                    // Only allow for Motor 1
                    uint8_t motorIndex = getMotorIndex();

                    if (motorType[motorIndex] != MotorType::LINEAR)
                    {
                        Serial.println(F("ERROR: Lower limit command is only valid for linear motors"));
                        continue;
                    }

                    String lowerLimitPx = c.getArgument("lo").getValue();

                    if (!validationInputAndSetLinearLowerLimit(lowerLimitPx))
                    {
                        continue;
                    }
                }

                // Handle upper limit command
                if (c.getArgument("up").isSet())
                {
                    if (!_isGetMotorNumber)
                    {
                        Serial.println(errorMotorNumberIsRequired);
                        continue;
                    }

                    _isGetMotorNumber = false;

                    // Only allow for Motor 1
                    uint8_t motorIndex = getMotorIndex();

                    if (motorType[motorIndex] != MotorType::LINEAR)
                    {
                        Serial.println(F("ERROR: Upper limit command is only valid for linear motors"));
                        continue;
                    }

                    String upperLimitPx = c.getArgument("up").getValue();

                    if (!validationInputAndSetLinearUpperLimit(upperLimitPx))
                    {
                        continue;
                    }
                }

                // Handle position commands
                if (c.getArgument("p").isSet())
                {
                    if (!_isGetMotorNumber)
                    {
                        Serial.println(errorMotorNumberIsRequired);
                        continue;
                    }

                    _isGetMotorNumber = false;

                    String targetStr = c.getArgument("p").getValue();

                    resetCommandReceived();

                    if (!validationInputAndSetTarget(targetStr))
                        continue;
                }

                // Get motor load position
                if (c.getArgument("l").isSet())
                {
                    continue;
                }
            }
            else if (c == cmdHelp)
            {
                Serial.print(F("Help:"));
                Serial.println(cli.toString());
            }
            else if (c == cmdRestart)
            {
                resetCommandReceived();
                Serial.println(F("System restarting..."));

                // Ensure motors are stopped before restart
                for (int i = 0; i < NUM_MOTORS; i++)
                {
                    motorStop(i);
                }

                delay(100);  // Give time for motors to stop
                ESP.restart();
            }
        }

        if (cli.errored())
        {
            CommandError cmdError = cli.getError();

            Serial.print(F("ERROR: "));
            Serial.println(cmdError.toString());
        }
        esp_task_wdt_reset();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void serialPrintTask(void* pvParameters)
{
    const TickType_t xFrequency    = pdMS_TO_TICKS(100);
    TickType_t       xLastWakeTime = xTaskGetTickCount();

    while (1)
    {
        printSerial();
        esp_task_wdt_reset();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void motorStopAndSavePosition(double currentPosition)
{
    uint8_t motorIndex = getMotorIndex();

    motorLastState[motorIndex] = motorState::STOPPED;
    motorStop(motorIndex);
    _commandReceived[motorIndex] = false;            // Reset command flag when target is reached or no command
    saveMotorPosition(motorIndex, currentPosition);  // Save the final position to EEPROM when motor stops

    Serial.print(F("\nMotor "));
    Serial.print(motorIndex + 1);
    Serial.print(F(" stopped at "));
    Serial.print(currentPosition);
    Serial.println(motorType[motorIndex] == MotorType::LINEAR ? F(" um") : F(" °"));
}

double getShortestAngularDistance(double current, double target)
{
    double diff = fabs(current - target);
    if (diff > 180.0)
        diff = 360.0 - diff;
    return diff;  // always positive
}

double getSignedPositionError(double current, double target)
{
    return target - current;  // retains sign
}

void rotationalMotorUpdate()
{
    uint8_t motorIndex      = getMotorIndex();
    float   currentPosition = encoders[motorIndex].getPositionDegrees();
    float   threshold       = ROTATIONAL_THRESHOLD;

    // Calculate signed angular position error [-180, 180]
    float positionError = getPositionError();

    // If previously stopped, save the distance
    if (motorLastState[motorIndex] == motorState::STOPPED)
    {
        distanceToTarget = fabs(positionError);
    }

    // Check if we haven't reached the target yet
    if (fabs(positionError) > threshold)
    {
        motorLastState[motorIndex] = motorState::MOVING;

        // Select the appropriate direction and move the motor
        if (positionError > 0)
        {
            motorMoveForward(motorIndex);
        }
        else
        {
            motorMoveReverse(motorIndex);
        }

        motorStep(motorIndex);  // Execute the step

        // Read the new position and check the error again
        currentPosition = encoders[motorIndex].getPositionDegrees();
        positionError   = getPositionError();

        if (fabs(positionError) <= threshold)
        {
            motorStopAndSavePosition(currentPosition);
        }
    }
    else
    {
        // If previously moving, stop and save the position
        if (motorLastState[motorIndex] == motorState::MOVING)
        {
            motorStopAndSavePosition(currentPosition);
        }
    }
}

void linearMotorUpdate() {}

void printSerial()
{
    uint8_t motorIndex = getMotorIndex();

    if (!isValidMotorIndex(motorIndex))
        return;

    float currentPosition = 0.0f;
    bool  isRotational    = motorType[motorIndex] == MotorType::ROTATIONAL;

    if (isRotational)
    {
        currentPosition = encoders[motorIndex].getPositionDegrees();
    }
    else
    {
        currentPosition = encoders[motorIndex].getPositionUM();
    }

    EncoderState state     = encoders[motorIndex].getState();
    String       direction = state.direction == Direction::CLOCKWISE ? "CW" : "CCW";
    float        degrees   = encoders[motorIndex].getPositionDegrees();
    double       target    = getTarget();

    if (fabs(state.current_Pulse - lastPulseWidthUs[motorIndex]) > 1)
    {
        // printf("\e[2J\e[1;1H");  // clear screen
        //  table header
        Serial.print(F("Motor\tLaps\tCurrent_Pulse\tDegrees\t\tDirection\tPulse_High\tPulse_Low\tTotal_"
                       "Pulse\tPosition\tTarget\n"));

        // Format all values into the buffer
        Serial.print(motorIndex + 1);
        Serial.print(F("\t"));
        Serial.print(state.laps);
        Serial.print(F("\t"));
        Serial.print(state.current_Pulse);
        Serial.print(F("\t\t"));
        Serial.print(degrees);
        Serial.print(F("\t\t"));
        Serial.print(direction.c_str());
        Serial.print(F("\t\t"));
        Serial.print(state.width_high);
        Serial.print(F("\t\t"));
        Serial.print(state.width_low);
        Serial.print(F("\t\t"));
        Serial.print(state.period);
        Serial.print(F("\t\t"));
        Serial.print(currentPosition);
        Serial.print(F("\t\t"));
        Serial.print(target);
        Serial.println();

        lastPulseWidthUs[motorIndex] = state.current_Pulse;
    }
}

void setTarget(double position)
{
    uint8_t motorIndex  = getMotorIndex();
    _target[motorIndex] = position;
}

double getTarget()
{
    uint8_t motorIndex = getMotorIndex();
    return _target[motorIndex];
}

double getPositionError()
{
    uint8_t motorIndex = getMotorIndex();
    return _target[motorIndex] - _currentPosition[motorIndex];
}

bool isValidMotorIndex(uint8_t motorIndex)
{
    if (motorIndex >= NUM_MOTORS)
    {
        Serial.println(F("ERROR: Invalid motor index!"));
        return false;
    }
    return true;
}

bool validationInputAndSetMotorIndex(String motorNumber)
{
    if (motorNumber.length() == 0)
    {
        Serial.println(errorMotorNumberIsRequired);
        return false;
    }

    int motorIndex = motorNumber.toInt() - 1;  // Convert to 0-based index

    if (motorIndex < 0 || motorIndex >= NUM_MOTORS)
    {
        Serial.println(errorMotorNumberIsInvalid);
        return false;
    }

    _isGetMotorNumber = true;
    setMotorIndex(motorIndex);

    return true;
}

uint8_t getMotorIndex()
{
    return _motorIndex;
}

void setMotorIndex(uint8_t motorIndex)
{
    if (!isValidMotorIndex(motorIndex))
        return;
    _motorIndex = motorIndex;
}

void resetCommandReceived()
{
    for (int i = 0; i < NUM_MOTORS; i++)
    {
        _commandReceived[i] = false;
    }
}

bool validationInputAndSetLinearOffset(String linearOffsetStr)
{
    double offsetUm = linearOffsetStr.toDouble() * UM_PER_PIXEL;
    if (offsetUm < _linearLowerLimitUm || offsetUm > _linearUpperLimitUm)
    {
        Serial.print(F("ERROR: Motor 1 offset must be between "));
        Serial.print(_linearLowerLimitUm);
        Serial.print(F(" and "));
        Serial.println(_linearUpperLimitUm);
        return false;
    }
    setLinearOffset(offsetUm);
    return true;
}

void setLinearOffset(double offsetUm)
{
    _linearOffsetUm = offsetUm;
    Serial.print(F("Motor 1 offset set to (px): "));
    Serial.println(_linearOffsetUm, 2);
}

bool validationInputAndSetLinearLowerLimit(String lowerLimitPx)
{
    double lowerLimit = lowerLimitPx.toDouble();
    setLowerLimits(lowerLimit);
    Serial.print(F("Motor 1 lower limit set to (px): "));
    Serial.println(lowerLimit, 2);
    return true;
}

bool validationInputAndSetLinearUpperLimit(String upperLimitPx)
{
    double upperLimit = upperLimitPx.toDouble();
    setUpperLimits(upperLimit);
    Serial.print(F("Motor 1 upper limit set to (px): "));
    Serial.println(upperLimit, 2);
    return true;
}

void setLowerLimits(double lowerLimit)
{
    _linearLowerLimitUm = lowerLimit;
}

void setUpperLimits(double upperLimit)
{
    _linearUpperLimitUm = upperLimit;
}

bool validationInputAndSetTarget(String targetStr)
{
    if (targetStr.length() == 0)
    {
        Serial.println(F("ERROR: Position (-p) requires a value"));
        return false;
    }

    uint8_t motorIndex = getMotorIndex();
    double  position   = targetStr.toDouble();

    Serial.print(F("Motor "));
    Serial.print(motorIndex + 1);
    Serial.print(F(" > new position is: "));
    Serial.print(position);
    Serial.println(motorType[motorIndex] == MotorType::LINEAR ? F(" um") : F(" °"));

    setTarget(position);
    _commandReceived[motorIndex] = true;  // Set flag only after valid set target command
    return true;
}
