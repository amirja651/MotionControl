#include <ArduinoLog.h>
#include <SimpleCLI.h>
#include <esp_system.h>  // Add this include for esp_reset_reason
#include "Config/TMC5160T_Driver.h"
#include "MAE3Encoder2.h"
#include "ObjectInstances.h"

uint8_t _MotorIndex = 0;

// Task handles
TaskHandle_t motorUpdateTaskHandle_0 = NULL;
TaskHandle_t serialReadTaskHandle    = NULL;
TaskHandle_t serialPrintTaskHandle   = NULL;

SimpleCLI cli;
Command   cmdMotor;
Command   cmdHelp;
Command   cmdRestart;  // New restart command
Command   cmdX1;
Command   cmdX2;
Command   cmdX3;
Command   cmdX4;

float lastPosition    = 0;
bool  commandReceived = false;  // Safety flag to prevent accidental movement

const char* getResetReasonString(esp_reset_reason_t reason)
{
    switch (reason)
    {
        case ESP_RST_UNKNOWN:
            return "Unknown";
        case ESP_RST_POWERON:
            return "Power On";
        case ESP_RST_EXT:
            return "Reset by external pin ";
        case ESP_RST_SW:
            return "Software reset";
        case ESP_RST_PANIC:
            return "Software reset due to exception/panic";
        case ESP_RST_INT_WDT:
            return "Interrupt Watchdog";
        case ESP_RST_TASK_WDT:
            return "Task Watchdog";
        case ESP_RST_WDT:
            return "Other watchdogs";
        case ESP_RST_DEEPSLEEP:
            return "Deep Sleep";
        case ESP_RST_BROWNOUT:
            return "Brownout";
        case ESP_RST_SDIO:
            return "SDIO";
        default:
            return "Unknown";
    }
}

void initializeCLI()
{
    cmdMotor = cli.addCmd("motor");
    cmdMotor.addArg("n", "1");     // motor number argument
    cmdMotor.addArg("p", "30.0");  // positional argument
    cmdMotor.addFlagArg("d");      // degree flag
    cmdMotor.addFlagArg("m");      // mm     flag
    cmdMotor.addFlagArg("u");      // um     flag
    cmdMotor.addFlagArg("s");      // stop flag
    cmdMotor.setDescription(" motor1 p 3.0 [n: motor number] [-d: degree] [-m: millimeters] [-u: micrometers] [-s: stop]");

    cmdHelp = cli.addCmd("help");
    cmdHelp.setDescription("Show help information");

    cmdRestart = cli.addCmd("restart");
    cmdRestart.setDescription("Restart the ESP32 system");

    cmdX1 = cli.addCmd("x1");
    cmdX2 = cli.addCmd("x2");
    cmdX3 = cli.addCmd("x3");
    cmdX4 = cli.addCmd("x4");
}

void motorStop()
{
    motors[_MotorIndex].stop();
    commandReceived = false;  // Reset command flag when target is reached or no command
}

void motorUpdateTask_0(void* pvParameters)
{
    const TickType_t xFrequency    = pdMS_TO_TICKS(5);
    TickType_t       xLastWakeTime = xTaskGetTickCount();

    while (1)
    {
        encoders2[_MotorIndex].update();

        bool  isRotational    = motors[_MotorIndex].isRotational();
        float currentPosition = encoders2[_MotorIndex].getPosition(isRotational);

        pids[_MotorIndex].setInput(currentPosition);
        pids[_MotorIndex].pid->Compute();

        double positionError = pids[_MotorIndex].getPositionError(currentPosition, isRotational);

        if (positionError > 0.5 && commandReceived)  // Only move if command was received
        {
            motors[_MotorIndex].step();

            currentPosition = encoders2[_MotorIndex].getPosition(isRotational);
            pids[_MotorIndex].setInput(currentPosition);
            pids[_MotorIndex].pid->Compute();

            (pids[_MotorIndex].output > currentPosition)   ? motors[_MotorIndex].moveForward()
            : (pids[_MotorIndex].output < currentPosition) ? motors[_MotorIndex].moveReverse()
                                                           : motorStop();
        }
        else
        {
            motorStop();
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
                    Log.noticeln(F("# %s"), inputBuffer.c_str());
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

                if (motorIndex >= NUM_MOTORS)
                {
                    Log.errorln(F("Invalid motor number"));
                    continue;
                }

                // Handle stop command
                if (c.getArgument("s").isSet())
                {
                    _MotorIndex = motorIndex;
                    motors[motorIndex].stop();
                    commandReceived = false;  // Explicitly stop movement
                    Serial.printf("Motor %d stopped\n", motorIndex + 1);
                    continue;
                }

                // Handle position commands
                Argument posArg = c.getArgument("p");
                if (posArg.isSet())
                {
                    double position = posArg.getValue().toDouble();

                    // Convert position to degrees based on unit flag
                    if (c.getArgument("d").isSet())
                    {
                        _MotorIndex = motorIndex;
                        pids[motorIndex].setTarget(position);
                        Serial.printf("Motor %d moving to %.2f degrees\n", motorIndex + 1, position);
                        commandReceived = true;  // Set flag only after valid command
                    }
                    else if (c.getArgument("m").isSet())
                    {
                        _MotorIndex = motorIndex;
                        pids[motorIndex].setTarget(position * 1000);
                        Serial.printf("Motor %d moving to %.2f um\n", motorIndex + 1, position);
                        commandReceived = true;  // Set flag only after valid command
                    }
                    else if (c.getArgument("u").isSet())
                    {
                        _MotorIndex = motorIndex;
                        pids[motorIndex].setTarget(position);
                        Serial.printf("Motor %d moving to %.2f um\n", motorIndex + 1, position);
                        commandReceived = true;  // Set flag only after valid command
                    }
                    else
                    {
                        Log.errorln(F("Please specify a unit flag (-d, -m, or -u)"));
                        commandReceived = false;  // Ensure no movement without proper unit
                    }
                }
                else
                {
                    Serial.printf("Motor %d selected\n", motorIndex + 1);
                    commandReceived = false;  // No position command, no movement
                }
            }
            else if (c == cmdHelp)
            {
                Serial.println("Help:");
                Serial.println(cli.toString());
            }
            else if (c == cmdRestart)
            {
                Serial.println("System restarting...");
                // Ensure motors are stopped before restart
                for (int i = 0; i < NUM_MOTORS; i++)
                {
                    motors[i].stop();
                }
                delay(100);  // Give time for motors to stop
                ESP.restart();
            }
            else if (c == cmdX1)
            {
                _MotorIndex = 0;
                pids[0].setTarget(100);
                Serial.printf("Motor %d moving to %.2f um\n", 1, 100.0f);
                commandReceived = true;  // Set flag only after valid command
            }
            else if (c == cmdX2)
            {
                _MotorIndex = 0;
                pids[0].setTarget(20);
                Serial.printf("Motor %d moving to %.2f um\n", 1, 20.0f);
                commandReceived = true;  // Set flag only after valid command
            }
            else if (c == cmdX3)
            {
                _MotorIndex = 1;
                pids[1].setTarget(500);
                Serial.printf("Motor %d moving to %.2f um\n", 1, 500.0f);
                commandReceived = true;  // Set flag only after valid command
            }
            else if (c == cmdX4)
            {
                _MotorIndex = 1;
                pids[1].setTarget(70);
                Serial.printf("Motor %d moving to %.2f um\n", 1, 70.0f);
                commandReceived = true;  // Set flag only after valid command
            }
        }

        if (cli.errored())
        {
            CommandError cmdError = cli.getError();
            Serial.print("ERROR: ");
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
        {
            bool        isRotational    = motors[_MotorIndex].isRotational();
            String      unit            = isRotational ? "°" : "um";
            const auto& state           = encoders2[_MotorIndex].getState();
            float       currentPosition = encoders2[_MotorIndex].getPosition(isRotational);
            String      direction       = state.direction == Direction::CLOCKWISE ? "CW" : "CCW";
            double      targetPosition  = pids[_MotorIndex].getTarget();
            double      positionError   = pids[_MotorIndex].getPositionError(currentPosition, isRotational);

            if (fabs(currentPosition - lastPosition) > 0.5)
            {
                Serial.printf(
                    "Laps\tPosition (%s)\tDirection\tTarget\tError\n"
                    "%d\t%.2f\t\t%s\t\t%.2f\t%.2f\n",
                    unit.c_str(), state.laps, currentPosition, direction.c_str(), targetPosition, positionError);

                lastPosition = currentPosition;
            }
        }

        if (!motors[_MotorIndex].testCommunication())
        {
            Log.errorln(F("Motor 1 communication test: FAILED"));
            commandReceived = false;  // Stop movement if communication fails
        }

        taskYIELD();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void setup()
{
    SPI.begin();

    Serial.begin(115200);
    delay(1000);
    while (!Serial)
    {
        delay(10);
    }

    Log.begin(LOG_LEVEL_VERBOSE, &Serial);
    Log.noticeln(F("System Initialization..."));
    Log.noticeln(F("CPU Frequency : %d MHz" CR), F_CPU / 1000000);
    Log.noticeln(F("Free RAM: %d bytes"), ESP.getFreeHeap());
    Log.noticeln(F("Reset Reason: %s"), getResetReasonString(esp_reset_reason()));

    initializeSystem();
    initializeCLI();
    //    xTaskCreate(motorUpdateTask_0, "MotorUpdateTask_0", 4096, NULL, 5, &motorUpdateTaskHandle_0);
    //   xTaskCreate(serialReadTask, "SerialReadTask", 4096, NULL, 4, &serialReadTaskHandle);
    //   xTaskCreate(serialPrintTask, "SerialPrintTask", 4096, NULL, 2, &serialPrintTaskHandle);

    xTaskCreatePinnedToCore(motorUpdateTask_0, "MotorUpdateTask", 4096, NULL, 5, &motorUpdateTaskHandle_0, 1);  // Core 1
    xTaskCreatePinnedToCore(serialReadTask, "SerialReadTask", 4096, NULL, 4, &serialReadTaskHandle, 0);         // Core 0
    xTaskCreatePinnedToCore(serialPrintTask, "SerialPrintTask", 4096, NULL, 2, &serialPrintTaskHandle, 0);      // Core 0
}

void loop()
{
    vTaskDelay(pdMS_TO_TICKS(10));  // Prevent watchdog timer issues
}