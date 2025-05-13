#include "CLI_Manager.h"
#include "Constants.h"
#include "ESP32_Manager.h"
#include "Motor_Manager.h"
#include "Object_Manager.h"
#include "Position_Storage.h"
#include "UnitConversion.h"
#include "esp_task_wdt.h"
#include <SPI.h>
#include <inttypes.h>

enum motorState
{
    MOTOR_STOPPED,
    MOTOR_MOVING,
    MOTOR_ERROR
};

uint8_t    _motorIndex                 = 0;
double     lastPosition[NUM_MOTORS]    = {0.0f, 0.0f, 0.0f, 0.0f};
bool       commandReceived[NUM_MOTORS] = {false, false, false, false};
motorState motorLastState[NUM_MOTORS]  = {motorState::MOTOR_STOPPED, motorState::MOTOR_STOPPED, motorState::MOTOR_STOPPED,
                                          motorState::MOTOR_STOPPED};

double motor1LowerLimitUm = LINEAR_LOWER_LIMIT_PX * UM_PER_PIXEL;  // Lower limit in pixels
double motor1UpperLimitUm = LINEAR_UPPER_LIMIT_PX * UM_PER_PIXEL;  // Upper limit in pixels
double motor1OffsetUm     = LINEAR_OFFSET_PX * UM_PER_PIXEL;       // Offset in pixels

// Command history support
#define HISTORY_SIZE 10
String commandHistory[HISTORY_SIZE];
int    historyCount     = 0;
int    historyIndex     = -1;  // -1 means not navigating
double distanceToTarget = 0;

// Task handles
TaskHandle_t encoderUpdateTaskHandle = NULL;
TaskHandle_t motorUpdateTaskHandle   = NULL;
TaskHandle_t serialReadTaskHandle    = NULL;
TaskHandle_t serialPrintTaskHandle   = NULL;

void resetCommandReceived();
bool checkPositionLimits(uint8_t motorIndex, double& position, double lowerLimit, double upperLimit);
void encoderUpdateTask(void* pvParameters);
void motorUpdateTask(void* pvParameters);
void serialReadTask(void* pvParameters);
void serialPrintTask(void* pvParameters);
void motorUpdate();
void readSerial(String inputBuffer, String lastInput);
void printSerial();

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
    initializeOtherObjects();

    xTaskCreatePinnedToCore(encoderUpdateTask, "EncoderUpdateTask", 2048, NULL, 5, &encoderUpdateTaskHandle, 1);  // Core 1
    esp_task_wdt_add(encoderUpdateTaskHandle);                                                              // Register with WDT
    xTaskCreatePinnedToCore(motorUpdateTask, "MotorUpdateTask", 2048, NULL, 3, &motorUpdateTaskHandle, 0);  // Core 0
    esp_task_wdt_add(motorUpdateTaskHandle);                                                                // Register with WDT
    xTaskCreatePinnedToCore(serialReadTask, "SerialReadTask", 2048, NULL, 2, &serialReadTaskHandle, 0);     // Core 0
    esp_task_wdt_add(serialReadTaskHandle);                                                                 // Register with WDT
    xTaskCreatePinnedToCore(serialPrintTask, "SerialPrintTask", 2048, NULL, 1, &serialPrintTaskHandle, 0);  // Core 0
    esp_task_wdt_add(serialPrintTaskHandle);                                                                // Register with WDT
}

void loop()
{
    esp_task_wdt_reset();
    vTaskDelay(1);  // Add a short delay to prevent WDT reset
}

void resetCommandReceived()
{
    for (int i = 0; i < NUM_MOTORS; i++)
    {
        commandReceived[i] = false;
    }
}

bool checkPositionLimits(uint8_t motorIndex, double& position, double lowerLimit, double upperLimit)
{
    // Check if position is within limits
    if (position < lowerLimit || position > upperLimit)
    {
        Serial.print(F("Position must be between "));
        Serial.print(lowerLimit);
        Serial.print(F(" and "));
        Serial.print(upperLimit);
        Serial.println(motorType[motorIndex] != MotorType::ROTATIONAL ? F(" um") : F(" ° "));
        return false;
    }

    return true;
}

void motorStopAndSavePosition(uint8_t motorIndex, double currentPosition)
{
    motorLastState[_motorIndex] = motorState::MOTOR_STOPPED;
    motorStop(_motorIndex);
    commandReceived[_motorIndex] = false;             // Reset command flag when target is reached or no command
    saveMotorPosition(_motorIndex, currentPosition);  // Save the final position to EEPROM when motor stops

    Serial.print(F("\nMotor "));
    Serial.print(_motorIndex + 1);
    Serial.print(F(" stopped at "));
    Serial.print(currentPosition);
    Serial.println(motorType[_motorIndex] == MotorType::LINEAR ? F(" um") : F(" °"));
}

void encoderUpdateTask(void* pvParameters)
{
    const TickType_t xFrequency    = pdMS_TO_TICKS(2);
    TickType_t       xLastWakeTime = xTaskGetTickCount();

    while (1)
    {
        encoders2[_motorIndex].update();
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
        motorUpdate();
        esp_task_wdt_reset();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void serialReadTask(void* pvParameters)
{
    const TickType_t xFrequency    = pdMS_TO_TICKS(20);
    TickType_t       xLastWakeTime = xTaskGetTickCount();
    String           inputBuffer   = "";
    String           lastInput     = "";
    while (1)
    {
        readSerial(inputBuffer, lastInput);
        esp_task_wdt_reset();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void serialPrintTask(void* pvParameters)
{
    const TickType_t xFrequency    = pdMS_TO_TICKS(50);
    TickType_t       xLastWakeTime = xTaskGetTickCount();

    while (1)
    {
        printSerial();
        esp_task_wdt_reset();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void motorUpdate()
{
    double currentPosition = 0.0f;
    double threshold       = 0.0f;
    bool   isRotational    = motorType[_motorIndex] == MotorType::ROTATIONAL;

    if (isRotational)
    {
        currentPosition = encoders2[_motorIndex].getCurrentPulse() * PULSE_TO_DEGREE_FACTOR_12B_256;
        threshold       = ROTATIONAL_THRESHOLD;
    }
    else
    {
        currentPosition = encoders2[_motorIndex].getTotalPulses() * PULSE_TO_UM_FACTOR_12B_32;
        threshold       = LINEAR_THRESHOLD;
    }

    double positionError = pids[_motorIndex].getPositionError(currentPosition, isRotational);

    if (motorLastState[_motorIndex] == motorState::MOTOR_STOPPED)
    {
        distanceToTarget = fabs(positionError);
    }

    if (fabs(positionError) > threshold && commandReceived[_motorIndex])
    {
        motorLastState[_motorIndex] = motorState::MOTOR_MOVING;

        if (isRotational)
        {
            currentPosition = encoders2[_motorIndex].getCurrentPulse() * PULSE_TO_DEGREE_FACTOR_12B_256;
        }
        else
        {
            currentPosition = encoders2[_motorIndex].getTotalPulses() * PULSE_TO_UM_FACTOR_12B_32;
        }

        pids[_motorIndex].setInput(currentPosition);
        pids[_motorIndex].pid->Compute();

        if (fabs(positionError) <= threshold)
        {
            motorStopAndSavePosition(_motorIndex, currentPosition);
        }

        if (positionError > 0)
        {
            motorMoveForward(_motorIndex);
            motorStep(_motorIndex);
        }
        else if (positionError < 0)
        {
            motorMoveReverse(_motorIndex);
            motorStep(_motorIndex);
        }

        if (fabs(positionError) <= threshold)
        {
            motorStopAndSavePosition(_motorIndex, currentPosition);
        }
    }
    else
    {
        if (motorLastState[_motorIndex] == motorState::MOTOR_MOVING)
        {
            motorStopAndSavePosition(_motorIndex, currentPosition);
        }
    }
}

void readSerial(String inputBuffer, String lastInput)
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
                    Serial.print("\r> ");
                    Serial.print(inputBuffer);
                    Serial.print("           \r> ");  // Overwrite any old chars
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
                    Serial.print("\r> ");
                    Serial.print(inputBuffer);
                    Serial.print("           \r> ");
                    Serial.print(inputBuffer);
                }
                else if (historyIndex == 0)
                {
                    historyIndex = -1;
                    inputBuffer  = lastInput;
                    Serial.print("\r> ");
                    Serial.print(inputBuffer);
                    Serial.print("           \r> ");
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
                Serial.print("\b \b");
            }
        }
        else if (c == 'u')
        {
            // Manual step left (reverse)
            motorMoveReverse(_motorIndex);
        }
        else if (c == 'i')
        {
            // Manual step right (forward)
            motorMoveForward(_motorIndex);
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
                if (motorNumStr.length() == 0)
                {
                    Serial.println(F("ERROR: Motor number (-n) requires a value"));
                    return;
                }

                int motorIndex = motorNumStr.toInt() - 1;  // Convert to 0-based index

                if (motorIndex < 0 || motorIndex >= NUM_MOTORS)
                {
                    Serial.print(F("ERROR: Invalid motor number. Must be between 1 and "));
                    Serial.println(NUM_MOTORS);
                    return;
                }

                resetCommandReceived();
                _motorIndex = motorIndex;
            }
            else
            {
                Serial.println(F("ERROR: Motor number (-n) is required"));
                return;
            }

            // Get motor load position
            if (c.getArgument("c").isSet())
            {
                Serial.print(F("*"));
                Serial.print(_motorIndex + 1);
                Serial.print(F("#"));

                if (motorType[_motorIndex] == MotorType::ROTATIONAL)
                {
                    Serial.print(encoders2[_motorIndex].getCurrentPulse() * PULSE_TO_DEGREE_FACTOR_12B_256);
                }
                else
                {
                    Serial.print(encoders2[_motorIndex].getTotalPulses() * PULSE_TO_UM_FACTOR_12B_32);
                }

                Serial.println(F("#"));
                return;
            }

            // Handle stop command
            if (c.getArgument("s").isSet())
            {
                Serial.print(F("Motor "));
                Serial.print(_motorIndex + 1);
                Serial.println(F(" stopped"));
                resetCommandReceived();
                motorStop(_motorIndex);
                return;
            }

            // Handle offset command
            if (c.getArgument("o").isSet())
            {
                if (motorType[_motorIndex] == MotorType::LINEAR)  // Only allow for Motor 1
                {
                    double offsetUm = c.getArgument("o").getValue().toDouble() * UM_PER_PIXEL;
                    if (offsetUm < motor1LowerLimitUm || offsetUm > motor1UpperLimitUm)
                    {
                        Serial.print(F("ERROR: Motor 1 offset must be between "));
                        Serial.print(motor1LowerLimitUm);
                        Serial.print(F(" and "));
                        Serial.println(motor1UpperLimitUm);
                        return;
                    }
                    motor1OffsetUm = offsetUm;
                    Serial.print(F("Motor 1 offset set to (px): "));
                    Serial.println(motor1OffsetUm, 2);
                }
            }

            // Handle lower limit command
            if (c.getArgument("lo").isSet())
            {
                if (motorType[_motorIndex] == MotorType::LINEAR)  // Only allow for Motor 1
                {
                    double lowerLimitPx = c.getArgument("lo").getValue().toDouble();
                    // encoders2[_motorIndex].setLowerLimits(pxToMm(lowerLimitPx));
                    Serial.print(F("Motor 1 lower limit set to (px): "));
                    Serial.println(lowerLimitPx, 2);
                }
            }

            // Handle upper limit command
            if (c.getArgument("up").isSet())
            {
                if (motorType[_motorIndex] == MotorType::LINEAR)  // Only allow for Motor 1
                {
                    double upperLimitPx = c.getArgument("up").getValue().toDouble();
                    // encoders2[_motorIndex].setUpperLimits(pxToMm(upperLimitPx));
                    Serial.print(F("Motor 1 upper limit set to (px): "));
                    Serial.println(upperLimitPx, 2);
                }
            }

            // Handle position commands
            if (c.getArgument("p").isSet())
            {
                String posStr = c.getArgument("p").getValue();
                if (posStr.length() == 0)
                {
                    Serial.println(F("ERROR: Position (-p) requires a value"));
                    return;
                }

                resetCommandReceived();
                double position = posStr.toDouble();

                Serial.print(F("Motor "));
                Serial.print(_motorIndex + 1);
                Serial.print(F(" > new position is: "));
                Serial.print(position);
                Serial.println(motorType[_motorIndex] == MotorType::LINEAR ? F(" um") : F(" °"));

                commandReceived[_motorIndex] = true;  // Set flag only after valid command
                pids[_motorIndex].setTarget(position);
                return;
            }

            // Get motor load position
            if (c.getArgument("l").isSet())
            {
                resetCommandReceived();
                double position = loadMotorPosition(_motorIndex);
                double loLimit  = (motorType[_motorIndex] == MotorType::LINEAR) ? motor1LowerLimitUm : ROTATIONAL_POSITION_MIN;
                double upLimit  = (motorType[_motorIndex] == MotorType::LINEAR) ? motor1UpperLimitUm : ROTATIONAL_POSITION_MAX;

                if (!checkPositionLimits(_motorIndex, position, loLimit, upLimit))
                {
                    return;
                }

                Serial.print(F("Motor "));
                Serial.print(_motorIndex + 1);
                Serial.print(F(" > loaded position is: "));
                Serial.print(position);
                Serial.println(motorType[_motorIndex] == MotorType::LINEAR ? F(" um") : F(" °"));

                commandReceived[_motorIndex] = true;  // Set flag only after valid command
                pids[_motorIndex].setTarget(position);
                return;
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
}

void printSerial()
{
    const auto& state = encoders2[_motorIndex].getState();

    String direction       = state.direction == Direction::CLOCKWISE ? "CW" : "CCW";
    String unit            = "";
    double currentPosition = 0.0f;
    double threshold       = 0.0f;
    bool   isRotational    = motorType[_motorIndex] == MotorType::ROTATIONAL;

    if (isRotational)
    {
        currentPosition = encoders2[_motorIndex].getCurrentPulse() * PULSE_TO_DEGREE_FACTOR_12B_256;
        threshold       = ROTATIONAL_DISPLAY_THRESHOLD;
        unit            = "° ";
    }
    else
    {
        currentPosition = encoders2[_motorIndex].getTotalPulses() * PULSE_TO_UM_FACTOR_12B_32;
        threshold       = LINEAR_DISPLAY_THRESHOLD;
        unit            = "um";
    }

    double targetPosition = pids[_motorIndex].getTarget();
    double positionError  = (targetPosition == 0) ? 0 : pids[_motorIndex].getPositionError(currentPosition, isRotational);

    if (targetPosition == 0)
    {
        positionError = 0;
    }

    if (fabs(currentPosition - lastPosition[_motorIndex]) > threshold)
    {
        Serial.println();
        Serial.println(F("--------------------------------------------------------------"));
        Serial.println(F("Motor  |  Direction  |  Pulses/Rev  |  Laps  |  TotalPulses"));

        // Format each value with fixed width
        char buffer[32];  // Increased from 20 to 32
        // Motor (5 chars)
        snprintf(buffer, sizeof(buffer), "%-5d", _motorIndex + 1);
        Serial.print(buffer);
        Serial.print("  |  ");
        // Direction (9 chars)
        snprintf(buffer, sizeof(buffer), "%-9s", direction.c_str());
        Serial.print(buffer);
        Serial.print("  |  ");
        // Pulses/Rev (10 chars)
        snprintf(buffer, sizeof(buffer), "%-10d", encoders2[_motorIndex].getCurrentPulse());
        Serial.print(buffer);
        Serial.print("  |  ");
        // Laps (4 chars)
        snprintf(buffer, sizeof(buffer), "%-4d", state.laps);
        Serial.print(buffer);
        Serial.print("  |  ");
        // Position (13 chars)
        snprintf(buffer, sizeof(buffer), "%-11" PRId64, encoders2[_motorIndex].getTotalPulses());
        Serial.println(buffer);

        lastPosition[_motorIndex] = currentPosition;
    }
}