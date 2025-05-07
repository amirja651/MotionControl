#include "CLI_Manager.h"
#include "ESP32_Manager.h"
#include "Motor_Manager.h"
#include "Object_Manager.h"
#include "Position_Storage.h"
#include <SPI.h>

enum motorState
{
    MOTOR_STOPPED,
    MOTOR_MOVING,
    MOTOR_ERROR
};

double     _motorLoadPosition = 0;
float      lastPosition       = 0;
bool       commandReceived    = false;
motorState motorLastState     = motorState::MOTOR_STOPPED;
uint8_t    _motorIndex        = 0;

double motor1LowerLimit = 550.0;  // Lower limit in pixels
double motor1UpperLimit = 900.0;  // Upper limit in pixels
double motor1Offset     = 680.0;  // Offset in pixels

// Task handles
TaskHandle_t motorUpdateTaskHandle = NULL;
TaskHandle_t serialReadTaskHandle  = NULL;
TaskHandle_t serialPrintTaskHandle = NULL;

// Command history support
#define HISTORY_SIZE 10
String commandHistory[HISTORY_SIZE];
int    historyCount = 0;
int    historyIndex = -1;  // -1 means not navigating

void motorUpdateTask(void* pvParameters);
void serialReadTask(void* pvParameters);
void serialPrintTask(void* pvParameters);

void setup()
{
    SPI.begin();

    Serial.begin(115200);
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

    xTaskCreatePinnedToCore(motorUpdateTask, "MotorUpdateTask", 4096, NULL, 5, &motorUpdateTaskHandle, 1);  // Core 1
    xTaskCreatePinnedToCore(serialReadTask, "SerialReadTask", 4096, NULL, 4, &serialReadTaskHandle, 0);     // Core 0
    xTaskCreatePinnedToCore(serialPrintTask, "SerialPrintTask", 4096, NULL, 2, &serialPrintTaskHandle, 0);  // Core 0
}

void loop()
{
    delay(100);
}

void motorUpdateTask(void* pvParameters)
{
    const TickType_t xFrequency    = pdMS_TO_TICKS(5);
    TickType_t       xLastWakeTime = xTaskGetTickCount();

    while (1)
    {
        encoders2[_motorIndex].update();
        float currentPosition = 0;
        bool  isRotational    = motorType[_motorIndex] == MotorType::ROTATIONAL;

        if (isRotational)
        {
            currentPosition = encoders2[_motorIndex].getPositionDegrees();
        }
        else
        {
            currentPosition = encoders2[_motorIndex].getTotalTravelUM();
        }

        double positionError = pids[_motorIndex].getPositionError(currentPosition, isRotational);

        float threshold = isRotational ? 0.5f : 0.3f;  // Different threshold for rotation and linear

        if (fabs(positionError) > threshold && commandReceived)
        {
            motorLastState = motorState::MOTOR_MOVING;

            if (isRotational)
            {
                currentPosition = encoders2[_motorIndex].getPositionDegrees();
                pids[_motorIndex].setInput(currentPosition);
                pids[_motorIndex].pid->Compute();

                if (pids[_motorIndex].output > 0)
                {
                    motorMoveForward(_motorIndex);
                    motorStep(_motorIndex);
                }
                else if (pids[_motorIndex].output < 0)
                {
                    motorMoveReverse(_motorIndex);
                    motorStep(_motorIndex);
                }
                else
                {
                    motorStop(_motorIndex);
                }
            }
            else
            {
                currentPosition = encoders2[_motorIndex].getTotalTravelUM();
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
                    motorLastState = motorState::MOTOR_STOPPED;
                    motorStop(_motorIndex);
                    commandReceived = false;  // Reset command flag when target is reached or no command
                }
            }
        }
        else
        {
            if (motorLastState == motorState::MOTOR_MOVING)
            {
                motorLastState = motorState::MOTOR_STOPPED;
                motorStop(_motorIndex);
                commandReceived = false;  // Reset command flag when target is reached or no command

                // Save the final position to EEPROM when motor stops
                saveMotorPosition(_motorIndex, currentPosition);
            }
        }

        taskYIELD();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void positionClamp(uint8_t motorIndex, double& position)
{
    double lowerLimit = motorType[motorIndex] == MotorType::ROTATIONAL ? 0.1f : motor1LowerLimit;
    double upperLimit = motorType[motorIndex] == MotorType::ROTATIONAL ? 359.9f : motor1UpperLimit;

    // Convert position to pixels if it's in micrometers
    if (motorType[motorIndex] == MotorType::LINEAR)
    {
        position = position * PIXELS_PER_UM;  // Convert μm to pixels (2 pixels = 11 μm)
    }

    // Clamp to limits
    if (position < lowerLimit)
    {
        position = lowerLimit;
        if (motorType[motorIndex] == MotorType::LINEAR)
        {
            position = position / PIXELS_PER_UM;  // Convert back to micrometers for internal use
        }
        Serial.print(F("Position clamped to lower limit: "));
        Serial.println(position, 2);
    }
    else if (position > upperLimit)
    {
        position = upperLimit;
        if (motorType[motorIndex] == MotorType::LINEAR)
        {
            position = position / PIXELS_PER_UM;  // Convert back to micrometers for internal use
        }
        Serial.print(F("Position clamped to upper limit: "));
        Serial.println(position, 2);
    }
    else
    {
        if (motorType[motorIndex] == MotorType::LINEAR)
        {
            // Convert back to micrometers for internal use
            position = position / PIXELS_PER_UM;
        }
    }

    Serial.print(F("Motor "));
    Serial.print(motorIndex + 1);
    Serial.print(F(" moving to "));
    Serial.print(position, 2);
    Serial.println(motorType[motorIndex] != MotorType::ROTATIONAL ? F(" um") : F(" ° "));
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
                    int motorIndex = c.getArgument("n").getValue().toInt() - 1;  // Convert to 0-based index

                    if (motorIndex >= NUM_MOTORS)
                    {
                        Serial.println(F("Invalid motor number"));
                        continue;
                    }

                    commandReceived = false;
                    _motorIndex     = motorIndex;
                }

                // Get motor load position
                if (c.getArgument("c").isSet())
                {
                    Serial.print(F("*"));
                    Serial.print(_motorIndex + 1);
                    Serial.print(F("#"));
                    Serial.print(motorType[_motorIndex] == MotorType::ROTATIONAL ? encoders2[_motorIndex].getPositionDegrees()
                                                                                 : encoders2[_motorIndex].getTotalTravelUM());
                    Serial.println(F("#"));
                    continue;
                }

                // Handle stop command
                if (c.getArgument("s").isSet())
                {
                    Serial.print(F("Motor "));
                    Serial.print(_motorIndex + 1);
                    Serial.println(F(" stopped"));
                    commandReceived = false;
                    motorStop(_motorIndex);
                    continue;
                }

                // Handle offset command
                if (c.getArgument("o").isSet())
                {
                    if (motorType[_motorIndex] == MotorType::LINEAR)  // Only allow for Motor 1
                    {
                        motor1Offset = c.getArgument("o").getValue().toDouble();
                        Serial.print(F("Motor 1 offset set to (px): "));
                        Serial.println(motor1Offset, 2);
                    }
                }

                if (c.getArgument("lo").isSet())
                {
                    if (motorType[_motorIndex] == MotorType::LINEAR)  // Only allow for Motor 1
                    {
                        motor1LowerLimit = c.getArgument("lo").getValue().toDouble();
                        encoders2[_motorIndex].setLowerLimits(motor1LowerLimit);
                        Serial.print(F("Motor 1 lower limit set to (px): "));
                        Serial.println(motor1LowerLimit, 2);
                    }
                }

                if (c.getArgument("up").isSet())
                {
                    if (motorType[_motorIndex] == MotorType::LINEAR)  // Only allow for Motor 1
                    {
                        motor1UpperLimit = c.getArgument("up").getValue().toDouble();
                        encoders2[_motorIndex].setUpperLimits(motor1UpperLimit);
                        Serial.print(F("Motor 1 upper limit set to (px): "));
                        Serial.println(motor1UpperLimit, 2);
                    }
                }

                // Handle position commands
                if (c.getArgument("p").isSet())
                {
                    commandReceived = false;
                    double position = c.getArgument("p").getValue().toDouble();
                    positionClamp(_motorIndex, position);
                    commandReceived = true;  // Set flag only after valid command
                    pids[_motorIndex].setTarget(position);
                }

                // Get motor load position
                if (c.getArgument("l").isSet())
                {
                    commandReceived    = false;
                    _motorLoadPosition = loadMotorPosition(_motorIndex);
                    positionClamp(_motorIndex, _motorLoadPosition);
                    commandReceived = true;  // Set flag only after valid command
                    pids[_motorIndex].setTarget(_motorLoadPosition);
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
                commandReceived = false;
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
        // if (encoders2[_motorIndex].update() && _motorIndex > 0 && _motorIndex < NUM_MOTORS)
        {
            const auto& state           = encoders2[_motorIndex].getState();
            String      direction       = state.direction == Direction::CLOCKWISE ? "CW" : "CCW";
            String      unit            = "";
            float       currentPosition = 0;
            bool        isRotational    = motorType[_motorIndex] == MotorType::ROTATIONAL;

            if (isRotational)
            {
                unit            = "° ";
                currentPosition = encoders2[_motorIndex].getPositionDegrees();
            }
            else
            {
                unit            = "um";
                currentPosition = encoders2[_motorIndex].getTotalTravelUM();
            }

            double positionError  = pids[_motorIndex].getPositionError(currentPosition, isRotational);
            double targetPosition = pids[_motorIndex].getTarget();

            if (fabs(currentPosition - lastPosition) > 1)
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
                snprintf(buffer, sizeof(buffer), "%-13.2f", motor1Offset + (currentPosition / 5.2f));
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

                lastPosition = currentPosition;
            }
        }

        taskYIELD();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}
