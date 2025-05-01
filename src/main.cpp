#include <SPI.h>
#include "CLI_Manager.h"
#include "ESP32_Manager.h"
#include "Motor_Manager.h"
#include "Object_Manager.h"

enum motorState
{
    MOTOR_STOPPED,
    MOTOR_MOVING,
    MOTOR_ERROR
};

float      lastPosition    = 0;
bool       commandReceived = false;
motorState motorLastState  = motorState::MOTOR_STOPPED;
uint8_t    _motorIndex     = 0;

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

        float threshold = isRotational ? 0.5f : 0.5f;  // Different threshold for rotation and linear

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

                if (pids[_motorIndex].output == 0)
                {
                    motorStop(_motorIndex);
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
                        Serial.print("                \r> ");  // Overwrite any old chars
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
                        Serial.print("                \r> ");
                        Serial.print(inputBuffer);
                    }
                    else if (historyIndex == 0)
                    {
                        historyIndex = -1;
                        inputBuffer  = lastInput;
                        Serial.print("\r> ");
                        Serial.print(inputBuffer);
                        Serial.print("                \r> ");
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
                Argument motorNumArg = c.getArgument("n");
                uint8_t  motorIndex  = motorNumArg.getValue().toInt() - 1;  // Convert to 0-based index
                _motorIndex          = motorIndex;

                if (motorIndex >= NUM_MOTORS)
                {
                    Serial.println(F("Invalid motor number"));
                    continue;
                }

                // Handle stop command
                if (c.getArgument("s").isSet())
                {
                    commandReceived = false;  // Explicitly stop movement
                    Serial.print(F("Motor "));
                    Serial.print(_motorIndex + 1);
                    Serial.println(F(" stopped\n"));
                    continue;
                }

                // Handle position commands
                Argument posArg = c.getArgument("p");
                if (posArg.isSet())
                {
                    double position = posArg.getValue().toDouble();

                    if (motorType[_motorIndex] == MotorType::ROTATIONAL)
                    {
                        if (position <= 0)
                        {
                            position = 0.1;
                            Serial.println(F("The   position is clamped to 0.1"));
                        }

                        if (position >= 360)
                        {
                            position = 359.9;
                            Serial.println(F("The position is clamped to 359.9"));
                        }
                    }
                    else
                    {
                        if (position <= 0)
                        {
                            position = 0.002;
                            Serial.println(F("The position is clamped to 0.002"));
                        }

                        if (position >= 15000)
                        {
                            position = 14999.9;
                            Serial.println(F("The position is clamped to 14999.9"));
                        }
                    }

                    commandReceived = true;  // Set flag only after valid command
                    pids[_motorIndex].setTarget(position);

                    Serial.print(F("Motor "));
                    Serial.print(_motorIndex + 1);
                    Serial.print(F(" moving to "));
                    Serial.print(position, 2);

                    if (motorType[_motorIndex] != MotorType::ROTATIONAL)
                    {
                        Serial.println(F(" um\n"));
                    }
                    else
                    {
                        Serial.println(F(" degrees\n"));
                    }

                    // Convert position to degrees based on unit flag
                    if (c.getArgument("d").isSet())
                    {
                    }
                    else if (c.getArgument("u").isSet())
                    {
                    }
                }
            }
            else if (c == cmdHelp)
            {
                Serial.print(F("Help:"));
                Serial.println(cli.toString());
            }
            else if (c == cmdRestart)
            {
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
                unit            = "°";
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
                Serial.print(F("Laps\tPosition ("));
                Serial.print(unit);
                Serial.println(F(")\tDirection\tTarget\tError\tMotor"));
                Serial.print(state.laps);
                Serial.print(F("\t"));
                Serial.print(currentPosition, 2);
                Serial.print(F("\t\t"));
                Serial.print(direction);
                Serial.print(F("\t\t"));
                Serial.print(targetPosition, 2);
                Serial.print(F("\t"));
                Serial.print(positionError, 2);
                Serial.print(F("\t"));
                Serial.println(_motorIndex + 1);
                lastPosition = currentPosition;
            }
        }

        taskYIELD();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}
