#include "CLI_Manager.h"
#include "Constants.h"
#include "ESP32_Manager.h"
#include "Motor_Manager.h"
#include "Object_Manager.h"
#include "Position_Storage.h"
#include "UnitConversion.h"
#include <SPI.h>

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

double motor1LowerLimitUm = pxToUm(LINEAR_LOWER_LIMIT_PX);  // Lower limit in pixels
double motor1UpperLimitUm = pxToUm(LINEAR_UPPER_LIMIT_PX);  // Upper limit in pixels
double motor1OffsetUm     = pxToUm(LINEAR_OFFSET_PX);       // Offset in pixels

// Command history support
#define HISTORY_SIZE 10
String commandHistory[HISTORY_SIZE];
int    historyCount = 0;
int    historyIndex = -1;  // -1 means not navigating

// Task handles
TaskHandle_t motorUpdateTaskHandle = NULL;
TaskHandle_t serialReadTaskHandle  = NULL;
TaskHandle_t serialPrintTaskHandle = NULL;

void resetCommandReceived();
void checkEncoderErrors();
bool checkPositionLimits(uint8_t motorIndex, double& position, double lowerLimit, double upperLimit);
void motorUpdateTask(void* pvParameters);
void serialReadTask(void* pvParameters);
void serialPrintTask(void* pvParameters);

void setup()
{
    SPI.begin();

    Serial.begin(115200);
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
    encoders2[0].calculateLaps(pxToMm(motor1OffsetUm));

    xTaskCreatePinnedToCore(motorUpdateTask, "MotorUpdateTask", 4096, NULL, 5, &motorUpdateTaskHandle, 1);  // Core 1
    xTaskCreatePinnedToCore(serialReadTask, "SerialReadTask", 4096, NULL, 4, &serialReadTaskHandle, 0);     // Core 0
    xTaskCreatePinnedToCore(serialPrintTask, "SerialPrintTask", 4096, NULL, 2, &serialPrintTaskHandle, 0);  // Core 0
}

void loop()
{
    delay(100);
}

void resetCommandReceived()
{
    for (int i = 0; i < NUM_MOTORS; i++)
    {
        commandReceived[i] = false;
    }
}

void checkEncoderErrors()
{
    for (int i = 0; i < NUM_MOTORS; i++)
    {
        if (encoders2[i].hasErrors())
        {
            encoders2[i].reportErrors();
        }
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

void motorUpdateTask(void* pvParameters)
{
    const TickType_t xFrequency    = pdMS_TO_TICKS(5);
    TickType_t       xLastWakeTime = xTaskGetTickCount();

    while (1)
    {
        encoders2[_motorIndex].update();

        double currentPosition = mmToUm(encoders2[_motorIndex].getTotalTravelMm());
        double threshold       = LINEAR_THRESHOLD;
        bool   isRotational    = motorType[_motorIndex] == MotorType::ROTATIONAL;

        if (isRotational)
        {
            currentPosition = encoders2[_motorIndex].getPositionDeg();
            threshold       = ROTATIONAL_THRESHOLD;
        }

        double positionError = pids[_motorIndex].getPositionError(currentPosition, isRotational);

        if (fabs(positionError) + threshold > threshold && commandReceived[_motorIndex])
        {
            motorLastState[_motorIndex] = motorState::MOTOR_MOVING;

            if (isRotational)
            {
                currentPosition = encoders2[_motorIndex].getPositionDeg();
            }
            else
            {
                currentPosition = mmToUm(encoders2[_motorIndex].getTotalTravelMm());
            }

            pids[_motorIndex].setInput(currentPosition);
            pids[_motorIndex].pid->Compute();

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
                motorLastState[_motorIndex] = motorState::MOTOR_STOPPED;
                motorStop(_motorIndex);
                commandReceived[_motorIndex] = false;  // Reset command flag when target is reached or no command
            }
        }
        else
        {
            if (motorLastState[_motorIndex] == motorState::MOTOR_MOVING)
            {
                motorLastState[_motorIndex] = motorState::MOTOR_STOPPED;
                motorStop(_motorIndex);
                commandReceived[_motorIndex] = false;  // Reset command flag when tarmotor -n 2 -p 300get is reached or no command

                // Save the final position to EEPROM when motor stops
                saveMotorPosition(_motorIndex, currentPosition);
            }
        }

        taskYIELD();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void serialReadTask(void* pvParameters)
{
    const TickType_t xFrequency    = pdMS_TO_TICKS(500);
    TickType_t       xLastWakeTime = xTaskGetTickCount();
    String           inputBuffer   = "";  // Buffer to store input
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
                motorStep(_motorIndex);
            }
            else if (c == 'i')
            {
                // Manual step right (forward)
                motorMoveForward(_motorIndex);
                motorStep(_motorIndex);
            }
            else
            {
                inputBuffer += c;  // Add character to buffer
                Serial.print(c);
                if (historyIndex == -1)
                    lastInput = inputBuffer;
            }
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
                        continue;
                    }

                    int motorIndex = motorNumStr.toInt() - 1;  // Convert to 0-based index

                    if (motorIndex < 0 || motorIndex >= NUM_MOTORS)
                    {
                        Serial.print(F("ERROR: Invalid motor number. Must be between 1 and "));
                        Serial.println(NUM_MOTORS);
                        continue;
                    }

                    resetCommandReceived();
                    _motorIndex = motorIndex;
                }
                else
                {
                    Serial.println(F("ERROR: Motor number (-n) is required"));
                    continue;
                }

                // Get motor load position
                if (c.getArgument("c").isSet())
                {
                    Serial.print(F("*"));
                    Serial.print(_motorIndex + 1);
                    Serial.print(F("#"));
                    Serial.print(motorType[_motorIndex] == MotorType::ROTATIONAL
                                     ? encoders2[_motorIndex].getPositionDeg()
                                     : mmToUm(encoders2[_motorIndex].getTotalTravelMm()));

                    Serial.println(F("#"));
                    continue;
                }

                // Handle stop command
                if (c.getArgument("s").isSet())
                {
                    Serial.print(F("Motor "));
                    Serial.print(_motorIndex + 1);
                    Serial.println(F(" stopped"));
                    resetCommandReceived();
                    motorStop(_motorIndex);
                    continue;
                }

                // Handle offset command
                if (c.getArgument("o").isSet())
                {
                    if (motorType[_motorIndex] == MotorType::LINEAR)  // Only allow for Motor 1
                    {
                        motor1OffsetUm = c.getArgument("o").getValue().toDouble();
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
                        encoders2[_motorIndex].setLowerLimits(pxToMm(lowerLimitPx));
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
                        encoders2[_motorIndex].setUpperLimits(pxToMm(upperLimitPx));
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
                        continue;
                    }

                    resetCommandReceived();
                    double position = posStr.toDouble();

                    double loLimit = (motorType[_motorIndex] == MotorType::LINEAR) ? motor1LowerLimitUm : ROTATIONAL_POSITION_MIN;
                    double upLimit = (motorType[_motorIndex] == MotorType::LINEAR) ? motor1UpperLimitUm : ROTATIONAL_POSITION_MAX;

                    if (!checkPositionLimits(_motorIndex, position, loLimit, upLimit))
                    {
                        continue;
                    }

                    Serial.print(F("Motor "));
                    Serial.print(_motorIndex + 1);
                    Serial.print(F(" > new position is: "));
                    Serial.print(position);
                    Serial.println(motorType[_motorIndex] == MotorType::LINEAR ? F(" um") : F(" °"));

                    commandReceived[_motorIndex] = true;  // Set flag only after valid command
                    pids[_motorIndex].setTarget(position);
                    continue;
                }

                // Get motor load position
                if (c.getArgument("l").isSet())
                {
                    resetCommandReceived();
                    double position = loadMotorPosition(_motorIndex);
                    double loLimit = (motorType[_motorIndex] == MotorType::LINEAR) ? motor1LowerLimitUm : ROTATIONAL_POSITION_MIN;
                    double upLimit = (motorType[_motorIndex] == MotorType::LINEAR) ? motor1UpperLimitUm : ROTATIONAL_POSITION_MAX;

                    if (!checkPositionLimits(_motorIndex, position, loLimit, upLimit))
                    {
                        continue;
                    }

                    Serial.print(F("Motor "));
                    Serial.print(_motorIndex + 1);
                    Serial.print(F(" > loaded position is: "));
                    Serial.print(position);
                    Serial.println(motorType[_motorIndex] == MotorType::LINEAR ? F(" um") : F(" °"));

                    commandReceived[_motorIndex] = true;  // Set flag only after valid command
                    pids[_motorIndex].setTarget(position);
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

        taskYIELD();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void serialPrintTask(void* pvParameters)
{
    const TickType_t xFrequency    = pdMS_TO_TICKS(300);
    TickType_t       xLastWakeTime = xTaskGetTickCount();

    while (1)
    {
        checkEncoderErrors();

        const auto& state = encoders2[_motorIndex].getState();

        String direction = state.direction == Direction::CLOCKWISE ? "CW" : "CCW";
        String unit      = "";

        unit                     = "° ";
        double currentPositionPx = 0;
        double currentPosition   = encoders2[_motorIndex].getPositionDeg();
        float  threshold         = ROTATIONAL_THRESHOLD;

        bool isLinear = motorType[_motorIndex] == MotorType::LINEAR;

        if (isLinear)
        {
            unit              = "um";
            currentPosition   = mmToUm(encoders2[_motorIndex].getTotalTravelMm());
            currentPositionPx = motor1OffsetUm + umToPx(currentPosition);
            threshold         = LINEAR_THRESHOLD;
        }

        double positionError  = pids[_motorIndex].getPositionError(currentPosition, !isLinear);
        double targetPosition = pids[_motorIndex].getTarget();

        if (fabs(currentPosition - lastPosition[_motorIndex]) > threshold)
        {
            Serial.print(F("Motor  Direction  Pulses/Rev  Laps  Position (px)  Position ("));
            Serial.print(unit);
            Serial.println(F(")  Target  Error"));

            // Format each value with fixed width
            char buffer[20];
            // Motor (5 chars)
            snprintf(buffer, sizeof(buffer), "%-5d", _motorIndex + 1);
            Serial.print(buffer);
            Serial.print("  ");
            // Direction (9 chars)
            snprintf(buffer, sizeof(buffer), "%-9s", direction.c_str());
            Serial.print(buffer);
            Serial.print("  ");
            // Pulses/Rev (10 chars)
            snprintf(buffer, sizeof(buffer), "%-10d", encoders2[_motorIndex].getPulsesPerRevolution());
            Serial.print(buffer);
            Serial.print("  ");
            // Laps (4 chars)
            snprintf(buffer, sizeof(buffer), "%-4d", state.laps);
            Serial.print(buffer);
            Serial.print("  ");
            // Position in px (15 chars)
            snprintf(buffer, sizeof(buffer), "%-13.2f", currentPositionPx);
            Serial.print(buffer);
            Serial.print("  ");
            // Position (13 chars)
            snprintf(buffer, sizeof(buffer), "%-13.2f", currentPosition);
            Serial.print(buffer);
            Serial.print("  ");
            // Target (6 chars)
            snprintf(buffer, sizeof(buffer), "%-6.2f", targetPosition);
            Serial.print(buffer);
            Serial.print("  ");
            // Error (5 chars)
            snprintf(buffer, sizeof(buffer), "%-5.2f", positionError);
            Serial.println(buffer);

            lastPosition[_motorIndex] = currentPosition;
        }

        taskYIELD();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}
