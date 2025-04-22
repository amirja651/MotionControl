#include <ArduinoLog.h>
#include <SimpleCLI.h>
#include "Config/TMC5160T_Driver.h"
#include "ObjectInstances.h"

#if NUM_MOTORS == 1
static bool lastShowMotorStatus[1] = {false};
#elif NUM_MOTORS == 2
static bool lastShowMotorStatus[2] = {false, false};
#elif NUM_MOTORS == 3
static bool lastShowMotorStatus[3] = {false, false, false};
#elif NUM_MOTORS == 4
static bool lastShowMotorStatus[4] = {false, false, false, false};
#endif

// Task handles
TaskHandle_t motorUpdateTaskHandle0 = NULL;
TaskHandle_t serialReadTaskHandle0  = NULL;

SimpleCLI cli;
Command   cmdMove1;
Command   cmdMove2;
Command   cmdMove3;
Command   cmdMove4;
Command   cmdStop1;
Command   cmdStop2;
Command   cmdStop3;
Command   cmdStop4;
Command   cmdEcho;
Command   cmdRm;
Command   cmdLs;
Command   cmdBoundless;
Command   cmdSingle;
Command   cmdHelp;

uint8_t _motorIndex = 0;

// Task for updating motor states
void motorUpdateTask0(void* pvParameters)
{
    const TickType_t xFrequency    = pdMS_TO_TICKS(10);  // Changed from 1ms to 10ms
    TickType_t       xLastWakeTime = xTaskGetTickCount();

    while (1)
    {
        double currentPosition = encoders[0].getPositionDegrees();
        // Serial.println(currentPosition);
        double positionError = pids[0].getPositionError(currentPosition);
        if (positionError > 0)
            Serial.println(positionError);

        if (positionError > 0.2)
        {
            motors[0].step();
            encoders[0].update();
            currentPosition = encoders[0].getPositionDegrees();
            pids[0].setInput(currentPosition);
            pids[0].pid->Compute();

            (pids[0].output > 0)   ? motors[0].moveForward()
            : (pids[0].output < 0) ? motors[0].moveReverse()
                                   : motors[0].stop();
        }

        taskYIELD();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void parseSerialInput()
{
    if (Serial.available())
    {
        String input = Serial.readStringUntil('\n');

        if (input.length() > 0)
        {
            Log.noticeln(F("# %s"), input.c_str());
            cli.parse(input);
        }
    }
}

void parseCLIInput()
{
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
                double f = p.toDouble();
                pids[0].setTarget(f);
                Serial.println(p + " deg Rotated!");
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
}

void showMotorStatus(uint8_t motorIndex)
{
    double currentPosition = encoders[motorIndex].getPositionDegrees();
    double targetPosition  = pids[motorIndex].getTarget();
    double positionError   = pids[motorIndex].getPositionError(currentPosition);

    if (positionError > 180.0)
    {
        positionError = 360.0 - positionError;
    }

    if (positionError != 0.0)
    {
        Log.noticeln(F("Position: %s°, Target: %s°, Error: %s°"), String(currentPosition).c_str(),
                     String(targetPosition).c_str(), String(positionError).c_str());
    }
}

void showStatus()
{
    if (!motors[0].testCommunication())
    {
        Log.errorln(F("Motor 1 communication test: FAILED"));
    }
    else
    {
        double currentPosition = encoders[0].getPositionDegrees();
        double positionError   = pids[0].getPositionError(currentPosition);
        showMotorStatus(0);

        if (positionError > 0.2)
        {
            lastShowMotorStatus[0] = false;
        }
        else
        {
            if (!lastShowMotorStatus[0])
            {
                lastShowMotorStatus[0] = true;
            }
        }
    }
}

// Task for handling serial input and PID tuning
void serialTask(void* pvParameters)
{
    const TickType_t     xFrequency     = pdMS_TO_TICKS(50);  // 50ms update rate
    TickType_t           xLastWakeTime  = xTaskGetTickCount();
    static unsigned long lastnoticeTime = 0;

    while (1)
    {
        parseSerialInput();
        parseCLIInput();

        // notice status periodically (every 800ms)
        if (millis() - lastnoticeTime >= 800)
        {
            showStatus();
            lastnoticeTime = millis();
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

    initializeSystem();
    initializeCLI();
    xTaskCreate(motorUpdateTask0, "MotorUpdateTask0", 4096, NULL, 3, &motorUpdateTaskHandle0);
    xTaskCreate(serialTask, "SerialReadTask0", 4096, NULL, 3, &serialReadTaskHandle0);
}

void loop()
{
    vTaskDelay(pdMS_TO_TICKS(10));  // Prevent watchdog timer issues
}