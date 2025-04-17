#include <ArduinoLog.h>
#include <SimpleCLI.h>
#include <algorithm>
#include "Config/TMC5160T_Driver.h"
#include "ObjectInstances.h"

static bool  wasAtTarget    = false;
const double PID_STEP_SMALL = 0.01;

// Task handles
TaskHandle_t motorUpdateTaskHandle0 = NULL;
TaskHandle_t serialReadTaskHandle0  = NULL;

SimpleCLI cli;
Command   cmdMove1;
Command   cmdStop1;
Command   cmdEcho;
Command   cmdRm;
Command   cmdLs;
Command   cmdBoundless;
Command   cmdSingle;
Command   cmdHelp;

// Task for updating motor states
void motorUpdateTask0(void* pvParameters)
{
    const TickType_t xFrequency    = pdMS_TO_TICKS(10);  // Changed from 1ms to 10ms
    TickType_t       xLastWakeTime = xTaskGetTickCount();

    while (1)
    {
        // bool testOK = motors[0].testCommunication();

        motors[0].update();
        encoders[0].update();
        pids[0].update();

        // Check if we just reached target
        if (!wasAtTarget && pids[0].isAtTarget())
        {
            Serial.println("Target reached!");
            motors[0].step();
            wasAtTarget = true;
        }
        else if (!pids[0].isAtTarget())
        {
            wasAtTarget = false;
        }

        taskYIELD();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// Task for handling serial input and PID tuning
void serialTask(void* pvParameters)
{
    const TickType_t     xFrequency    = pdMS_TO_TICKS(500);  // 50ms update rate
    TickType_t           xLastWakeTime = xTaskGetTickCount();
    static unsigned long lastPrintTime = 0;

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
                    pids[0].setTarget(f);  // Target 180 degrees
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

        // Print status periodically (every 500ms)
        if (millis() - lastPrintTime >= 800)
        {
            if (!pids[0].isAtTarget())
            {
                bool testOK = motors[0].testCommunication();
                if (testOK)
                {
                    float currentPosition = encoders[0].getPositionDegrees();
                    float currentVelocity = encoders[0].getVelocityDPS();
                    float targetPosition  = pids[0].getTarget();
                    float positionError   = abs(currentPosition - targetPosition);

                    if (positionError > 180.0f)
                    {
                        positionError = 360.0f - positionError;
                    }

                    Serial.print("Position: ");
                    Serial.print(currentPosition, 2);
                    Serial.print("°, Velocity: ");
                    Serial.print(currentVelocity, 2);
                    Serial.print("°/s, Target: ");
                    Serial.print(targetPosition, 2);
                    Serial.print("°, Error: ");
                    Serial.print(positionError, 2);
                    Serial.println("°");
                }
            }

            lastPrintTime = millis();
        }

        taskYIELD();
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
    SPI.begin();
    Serial.begin(CONFIG::SYSTEM::SERIAL_BAUD_RATE);
    delay(CONFIG::SYSTEM::STARTUP_DELAY_MS);
    while (!Serial)
    {
        delay(10);
    }

    Log.begin(LOG_LEVEL_VERBOSE, &Serial);
    Log.noticeln(F("System Initialization..."));
    Log.noticeln(F("CPU Frequency : %d MHz" CR), F_CPU / 1000000);

    initializeSystem();
    initializeCLI();
    xTaskCreate(motorUpdateTask0, "MotorUpdateTask0", 4096, NULL, 3, &motorUpdateTaskHandle0);
    xTaskCreate(serialTask, "SerialReadTask0", 4096, NULL, 3, &serialReadTaskHandle0);

    pids[0].setTarget(5.0f);
    pids[0].setOutputLimits(-180.0f, 180.0f);
}

void loop()
{
    vTaskDelay(pdMS_TO_TICKS(10));  // Prevent watchdog timer issues
}