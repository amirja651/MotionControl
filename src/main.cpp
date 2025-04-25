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

float lastPosition     = 0;
bool  commandReceived  = false;  // Safety flag to prevent accidental movement
bool  isMicrometerUnit = false;

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

    // Add new command to show current position
    Command cmdPosition = cli.addCmd("position");
    cmdPosition.addArg("n", "1");  // motor number argument
    cmdPosition.setDescription("Show current position of specified motor [n: motor number]");
}

void motorUpdateTask_0(void* pvParameters)
{
    const TickType_t xFrequency    = pdMS_TO_TICKS(5);
    TickType_t       xLastWakeTime = xTaskGetTickCount();

    while (1)
    {
        float  positionDegrees = encoders2[0].getPositionDegrees();
        float  totalTravelMM   = encoders2[0].getTotalTravelMM();
        double currentPosition =
            motors[0].isRotational() ? positionDegrees : (isMicrometerUnit ? totalTravelMM * 1000.0f : totalTravelMM);
        double positionError  = pids[0].getPositionError(currentPosition, motors[0].isRotational());
        double teloranceError = motors[0].isRotational() ? 0.5 : (isMicrometerUnit ? 2 : 0.002);
        double targetPosition = pids[0].getTarget();

        if (positionError > teloranceError && commandReceived)  // Only move if command was received
        {
            motors[0].step();
            encoders2[0].update();

            currentPosition =
                motors[0].isRotational() ? positionDegrees : (isMicrometerUnit ? totalTravelMM * 1000.0f : totalTravelMM);

            pids[0].setInput(currentPosition);
            pids[0].pid->Compute();

            (targetPosition > currentPosition)   ? motors[0].moveForward()
            : (targetPosition < currentPosition) ? motors[0].moveReverse()
                                                 : motors[0].stop();
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
                        isMicrometerUnit = false;
                        pids[motorIndex].setTarget(position);
                        Serial.printf("Motor %d moving to %.2f degrees\n", motorIndex + 1, position);
                        commandReceived = true;  // Set flag only after valid command
                    }
                    else if (c.getArgument("m").isSet())
                    {
                        isMicrometerUnit = false;
                        pids[motorIndex].setTarget(position);
                        Serial.printf("Motor %d moving to %.2f mm\n", motorIndex + 1, position);
                        commandReceived = true;  // Set flag only after valid command
                    }
                    else if (c.getArgument("u").isSet())
                    {
                        isMicrometerUnit = true;
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
            else if (c.getName() == "position")
            {
                // Get motor number
                Argument motorNumArg = c.getArgument("n");
                uint8_t  motorIndex  = motorNumArg.getValue().toInt() - 1;  // Convert to 0-based index

                if (motorIndex >= NUM_MOTORS)
                {
                    Log.errorln(F("Invalid motor number"));
                    continue;
                }

                // Get current position
                float positionDegrees = encoders2[motorIndex].getPositionDegrees();
                float totalTravelMM   = encoders2[motorIndex].getTotalTravelMM();

                // Format output based on motor type
                if (motors[motorIndex].isRotational())
                {
                    Serial.printf("Motor %d current position: %.2f degrees\n", motorIndex + 1, positionDegrees);
                }
                else
                {
                    Serial.printf("Motor %d current position: %.3f mm (%.1f um)\n", motorIndex + 1, totalTravelMM,
                                  totalTravelMM * 1000.0f);
                }
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
            float       totalTravelMM   = encoders2[0].getTotalTravelMM();
            double      currentPosition =
                motors[0].isRotational() ? positionDegrees : (isMicrometerUnit ? totalTravelMM * 1000.0f : totalTravelMM);

            String unit           = motors[0].isRotational() ? "°" : (isMicrometerUnit ? "um" : "mm");
            double targetPosition = pids[0].getTarget();
            double positionError  = pids[0].getPositionError(currentPosition, motors[0].isRotational());
            String direction      = state.direction == Direction::CLOCKWISE ? "CW" : "CCW";

            double teloranceError = motors[0].isRotational() ? 0.5 : (isMicrometerUnit ? 2 : 0.002);

            if (fabs(currentPosition - lastPosition) > teloranceError)
            {
                Serial.printf(
                    "Pulse\tPosition (%s)\tDirection\tTarget\tError\n"
                    "%d\t%.2f\t\t%s\t\t%.2f\t%.2f\n",
                    unit.c_str(), state.currentPulse, currentPosition, direction.c_str(), targetPosition, positionError);

                Serial.printf("Output: %f, Current Position: %f\n\n\n\n\n\n\n", pids[0].output, currentPosition);
                lastPosition = currentPosition;
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