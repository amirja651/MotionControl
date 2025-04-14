#include <ArduinoLog.h>
#include <ESP32TimerInterrupt.h>
#include <SimpleCLI.h>
#include "MotorInstances.h"

TaskHandle_t serialTaskHandle       = NULL;
TaskHandle_t motorUpdateTaskHandle0 = NULL;
TaskHandle_t motorUpdateTaskHandle1 = NULL;
TaskHandle_t motorUpdateTaskHandle2 = NULL;
TaskHandle_t motorUpdateTaskHandle3 = NULL;
SimpleCLI    cli;
Command      cmdMove1;
Command      cmdStop1;
Command      cmdEcho;
Command      cmdRm;
Command      cmdLs;
Command      cmdBoundless;
Command      cmdSingle;
Command      cmdHelp;

// Task for handling serial commands
void serialTask(void* pvParameters)
{
    while (1)
    {
        if (Serial.available())
        {
            String input = Serial.readStringUntil('\n');

            if (input.length() > 0)
            {
                Serial.print("# ");
                Serial.println(input);
                cli.parse(input);
            }
        }

        if (cli.available())
        {
            Command c = cli.getCmd();

            int argNum = c.countArgs();

            Serial.print("> ");
            Serial.print(c.getName());
            Serial.print(' ');

            for (int i = 0; i < argNum; ++i)
            {
                Argument arg = c.getArgument(i);
                // if(arg.isSet()) {
                Serial.print(arg.toString());
                Serial.print(' ');
                // }
            }

            Serial.println();

            if (c == cmdMove1)
            {
                Argument a = c.getArgument("p");
                // bool     set = a.isSet();
                if (a.isSet())
                {
                    String p = c.getArgument("p").getValue();
                    float  f = p.toFloat();
                    motors[0].moveForward();
                    Serial.print(p + " deg Rotated!");
                }
                else
                {
                    Serial.println("Parameter p (degrees) is not set");
                }
            }
            else if (c == cmdStop1)
            {
                motors[0].stop();
            }
            else if (c == cmdEcho)
            {
                Argument str = c.getArgument(0);
                Serial.println(str.getValue());
            }
            else if (c == cmdRm)
            {
                Serial.println("Remove directory " + c.getArgument(0).getValue());
            }
            else if (c == cmdLs)
            {
                Argument a = c.getArgument("a");
                // bool     set = a.isSet();
                if (a.isSet())
                {
                    Serial.println("Listing all directories");
                }
                else
                {
                    Serial.println("Listing directories");
                }
            }
            else if (c == cmdBoundless)
            {
                Serial.print("Boundless: ");

                for (int i = 0; i < argNum; ++i)
                {
                    Argument arg = c.getArgument(i);
                    if (i > 0)
                        Serial.print(",");
                    Serial.print("\"");
                    Serial.print(arg.getValue());
                    Serial.print("\"");
                }
            }
            else if (c == cmdSingle)
            {
                Serial.println("Single \"" + c.getArg(0).getValue() + "\"");
            }
            else if (c == cmdHelp)
            {
                Serial.println("Help:");
                Serial.println(cli.toString());
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
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// Task for updating motor states
void motorUpdateTask0(void* pvParameters)
{
    const TickType_t xFrequency    = pdMS_TO_TICKS(10);  // Changed from 1ms to 10ms
    TickType_t       xLastWakeTime = xTaskGetTickCount();

    while (1)
    {
        motors[0].update();
        taskYIELD();  // Allow other tasks to run between motor updates
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// Task for updating motor states
void motorUpdateTask1(void* pvParameters)
{
    const TickType_t xFrequency    = pdMS_TO_TICKS(10);  // Changed from 1ms to 10ms
    TickType_t       xLastWakeTime = xTaskGetTickCount();

    while (1)
    {
        motors[1].update();
        taskYIELD();  // Allow other tasks to run between motor updates
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// Task for updating motor states
void motorUpdateTask2(void* pvParameters)
{
    const TickType_t xFrequency    = pdMS_TO_TICKS(10);  // Changed from 1ms to 10ms
    TickType_t       xLastWakeTime = xTaskGetTickCount();

    while (1)
    {
        motors[2].update();
        taskYIELD();  // Allow other tasks to run between motor updates
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// Task for updating motor states
void motorUpdateTask3(void* pvParameters)
{
    const TickType_t xFrequency    = pdMS_TO_TICKS(10);  // Changed from 1ms to 10ms
    TickType_t       xLastWakeTime = xTaskGetTickCount();

    while (1)
    {
        motors[3].update();
        taskYIELD();  // Allow other tasks to run between motor updates
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void initializeCLI()
{
    cmdMove1 = cli.addCmd("move1");
    cmdMove1.addArg("p", "30.0");
    cmdMove1.setDescription(" Rotate motor[1] 1 by 360 degrees");

    cmdStop1 = cli.addCmd("stop1");
    cmdStop1.setDescription(" Stop motor[1]");

    cmdEcho = cli.addCmd("echo");
    cmdEcho.addPosArg("text", "something");
    cmdEcho.setDescription(" Echos what you said");

    cmdRm = cli.addCmd("rm");
    cmdRm.addPosArg("file");
    cmdRm.setDescription(" Removes specified file (but not actually)");

    cmdLs = cli.addCmd("ls");
    cmdLs.addFlagArg("a");
    cmdLs.setDescription(" Lists files in directory (-a for all)");

    cmdBoundless = cli.addBoundlessCmd("boundless");
    cmdBoundless.setDescription(" A boundless command that echos your input");

    cmdSingle = cli.addSingleArgCmd("single");
    cmdSingle.setDescription(" A single command that echos your input");

    cmdHelp = cli.addCommand("help");
    cmdHelp.setDescription(" Get help!");
}

void setup()
{
    // Initialize serial communication
    Serial.begin(Config::System::SERIAL_BAUD_RATE);
    delay(Config::System::STARTUP_DELAY_MS);
    while (!Serial)
    {
        delay(10);
    }
    Log.begin(LOG_LEVEL_VERBOSE, &Serial);
    Log.noticeln(F("Hello, World!"));
    Log.noticeln(F("CPU Frequency : %d MHz" CR), F_CPU / 1000000);

    delay(500);
    initializeMotors();
    initializeCLI();
    xTaskCreate(serialTask, "SerialTask", 4096, NULL, 2, &serialTaskHandle);
    xTaskCreate(motorUpdateTask0, "MotorUpdateTask0", 4096, NULL, 3, &motorUpdateTaskHandle0);
    xTaskCreate(motorUpdateTask1, "MotorUpdateTask1", 4096, NULL, 3, &motorUpdateTaskHandle1);
    xTaskCreate(motorUpdateTask2, "MotorUpdateTask2", 4096, NULL, 3, &motorUpdateTaskHandle2);
    xTaskCreate(motorUpdateTask3, "MotorUpdateTask3", 4096, NULL, 3, &motorUpdateTaskHandle3);
}

void loop()
{
    delay(10);
}
