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
        bool   isRotational    = motorType[_motorIndex] == MotorType::ROTATIONAL;
        float  positionDegrees = encoders2[_motorIndex].getPositionDegrees();
        float  totalTravelUM   = encoders2[_motorIndex].getTotalTravelUM();
        double currentPosition = isRotational ? positionDegrees : totalTravelUM;
        double positionError   = pids[_motorIndex].getPositionError(currentPosition, isRotational);
        if (commandReceived)
        {
            Serial.print(F("Command received: "));
            Serial.print(positionError);
            Serial.print(F(", "));
            Serial.println(commandReceived);
        }

        if (positionError > 0.5 && commandReceived)  // Only move if command was received
        {
            motorLastState = motorState::MOTOR_MOVING;
            motorStep(_motorIndex);

            currentPosition = isRotational ? positionDegrees : totalTravelUM;

            pids[_motorIndex].setInput(currentPosition);
            pids[_motorIndex].pid->Compute();

            (pids[_motorIndex].output > 0)   ? motorMoveForward(_motorIndex)
            : (pids[_motorIndex].output < 0) ? motorMoveReverse(_motorIndex)
                                             : motorStop(_motorIndex);
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

    while (1)
    {
        while (Serial.available())
        {
            char c = Serial.read();
            if (c == '\n')
            {
                if (inputBuffer.length() > 0)
                {
                    Serial.print(F("# "));
                    Serial.println(inputBuffer.c_str());
                    cli.parse(inputBuffer);
                    inputBuffer = "";  // Clear the buffer
                }
            }
            else
            {
                inputBuffer += c;  // Add character to buffer
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

                    commandReceived = true;  // Set flag only after valid command
                    pids[_motorIndex].setTarget(position);

                    Serial.print(F("Motor "));
                    Serial.print(_motorIndex + 1);
                    Serial.print(F(" moving to "));
                    Serial.print(position, 2);

                    if (_motorIndex == 0)
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
        // if (encoders2[_motorIndex].update())
        {
            const auto& state           = encoders2[_motorIndex].getState();
            String      direction       = state.direction == Direction::CLOCKWISE ? "CW" : "CCW";
            bool        isRotational    = motorType[_motorIndex] == MotorType::ROTATIONAL;
            String      unit            = isRotational ? "°" : "um";
            float       positionDegrees = encoders2[_motorIndex].getPositionDegrees();
            float       totalTravelUM   = encoders2[_motorIndex].getTotalTravelUM();
            double      currentPosition = isRotational ? positionDegrees : totalTravelUM;
            double      positionError   = pids[_motorIndex].getPositionError(currentPosition, isRotational);
            double      targetPosition  = pids[_motorIndex].getTarget();

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

        /**if (!driverCommunicationTest(_motorIndex, false))
        {
            Serial.print(F("Motor "));
            Serial.print(_motorIndex+1);
            Serial.println(F(" communication test: FAILED"));
            commandReceived = false;  // Stop movement if communication fails
        }**/

        taskYIELD();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}
