#include <ArduinoLog.h>
#include <SimpleCLI.h>
#include <esp_system.h>  // Add this include for esp_reset_reason
#include "Config/TMC5160T_Driver.h"
#include "MAE3Encoder2.h"
#include "ObjectInstances.h"

// Task handles
TaskHandle_t motorUpdateTaskHandle_0 = NULL;
TaskHandle_t serialReadTaskHandle    = NULL;
TaskHandle_t serialPrintTaskHandle   = NULL;

SimpleCLI cli;

Command cmdMotor;
Command cmdHelp;
Command cmdRestart;  // New restart command

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
}

double teloranceError[4] = {0.5, 0.5, 0.5, 0.5};

void motorUpdateTask_0(void* pvParameters)
{
    const TickType_t xFrequency    = pdMS_TO_TICKS(5);
    TickType_t       xLastWakeTime = xTaskGetTickCount();

    while (1)
    {
        double currentPosition = motors[0].isRotational() ? encoders2[0].getPositionDegrees() : encoders2[0].getPositionMM();
        double positionError   = pids[0].getPositionError(currentPosition, motors[0].isRotational());

        if (positionError > teloranceError[0] && commandReceived)  // Only move if command was received
        {
            motors[0].step();
            encoders2[0].update();

            currentPosition = motors[0].isRotational() ? encoders2[0].getPositionDegrees() : encoders2[0].getPositionMM();

            pids[0].setInput(currentPosition);
            pids[0].pid->Compute();

            (pids[0].output > 0) ? motors[0].moveForward() : (pids[0].output < 0) ? motors[0].moveReverse() : motors[0].stop();
        }
        else
        {
            motors[0].stop();
            commandReceived = false;  // Reset command flag when target is reached or no command
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

            // int argNum = c.countArgs();

            // Serial.print("> ");
            // Serial.print(c.getName());
            // Serial.print(' ');

            // Log.noticeln(F("Command: %s"), c.getName().c_str());

            // for (int i = 0; i < argNum; ++i)
            // {
            //     Argument arg = c.getArgument(i);
            //     Log.noticeln(F("Argument: %s"), arg.toString().c_str());
            // }

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
                        pids[motorIndex].setTarget(position);
                        Serial.printf("Motor %d moving to %.2f degrees\n", motorIndex + 1, position);
                        commandReceived = true;  // Set flag only after valid command
                    }
                    else if (c.getArgument("m").isSet())
                    {
                        double degrees = position;  // 1:1 conversion for now
                        pids[motorIndex].setTarget(degrees);
                        Serial.printf("Motor %d moving to %.2f mm (%.2f degrees)\n", motorIndex + 1, position, degrees);
                        commandReceived = true;  // Set flag only after valid command
                    }
                    else if (c.getArgument("u").isSet())
                    {
                        double degrees = position / 1000.0;  // 1000um = 1 degree
                        pids[motorIndex].setTarget(degrees);
                        Serial.printf("Motor %d moving to %.2f um (%.2f degrees)\n", motorIndex + 1, position, degrees);
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
        }

        if (cli.errored())
        {
            CommandError cmdError = cli.getError();

            Serial.print("ERROR: ");
            Serial.println(cmdError.toString());

            if (cmdError.hasCommand())
            {
                Serial.print("Did you mean \"");
                Serial.print(cmdError.getCommand().toString());
                Serial.println("\"?");
            }
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
            const auto& state           = encoders2[0].getState();
            float       positionDegrees = encoders2[0].getPositionDegrees();
            float       positionUM      = encoders2[0].getPositionUM();
            float       totalTravelMM   = encoders2[0].getTotalTravelMM();
            double      targetPosition  = pids[0].getTarget();
            double      positionTemp    = motors[0].isRotational() ? positionDegrees : positionUM;
            double      positionError   = pids[0].getPositionError(positionTemp);

            if (fabs(positionTemp - lastPosition) > 1)
            {
                Serial.printf(
                    "Current Pulse: %d,   Laps: %d,   Position: %.2f%s   Total Travel: %.3f mm,    "
                    "Direction: "
                    "%s,   Target: %.2f,   Error: %.2f\n\n\n\n\n\n\n\n\n\n\n\n",
                    state.currentPulse, state.laps, positionTemp, motors[0].isRotational() ? "°" : "um", totalTravelMM,
                    state.direction == Direction::CLOCKWISE ? "CW" : "CCW", targetPosition, positionError);
                lastPosition = positionTemp;
            }
        }

        if (!motors[0].testCommunication())
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