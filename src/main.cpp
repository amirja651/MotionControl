#include <algorithm>
#include "Config/TMC5160T_Driver.h"
#include "ObjectInstances.h"

static double lastKP         = CONFIG::PID::KP;
static double lastKI         = CONFIG::PID::KI;
static double lastKD         = CONFIG::PID::KD;
static bool   wasAtTarget    = false;
const double  PID_STEP_SMALL = 0.01;

// Task handles
TaskHandle_t motorUpdateTaskHandle0 = NULL;
TaskHandle_t serialReadTaskHandle0  = NULL;

// Task for updating motor states
void motorUpdateTask0(void* pvParameters)
{
    const TickType_t xFrequency    = pdMS_TO_TICKS(10);  // Changed from 1ms to 10ms
    TickType_t       xLastWakeTime = xTaskGetTickCount();

    while (1)
    {
        bool testOK = motors[0].testCommunication();
        if (testOK)
        {
            motors[0].update();
            encoders[0].update();
            pids[0].update();

            // Check if we just reached target
            if (!wasAtTarget && pids[0].isAtTarget())
            {
                Serial.println("Target reached!");
                wasAtTarget = true;
            }
            else if (!pids[0].isAtTarget())
            {
                wasAtTarget = false;
            }
        }

        taskYIELD();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// Task for handling serial input and PID tuning
void serialReadTask0(void* pvParameters)
{
    const TickType_t     xFrequency    = pdMS_TO_TICKS(500);  // 50ms update rate
    TickType_t           xLastWakeTime = xTaskGetTickCount();
    static unsigned long lastPrintTime = 0;

    while (1)
    {
        if (Serial.available() > 0)
        {
            char input = Serial.read();

            switch (input)
            {
                case 'p':
                case 'P':  // Kp adjustment
                case 'i':
                case 'I':  // Ki adjustment
                case 'd':
                case 'D':  // Kd adjustment
                {
                    // Adjust the appropriate gain
                    switch (input)
                    {
                        case 'p':
                            lastKP = std::max(0.0, lastKP - PID_STEP_SMALL);
                            break;
                        case 'P':
                            lastKP += PID_STEP_SMALL;
                            break;
                        case 'i':
                            lastKI = std::max(0.0, lastKI - PID_STEP_SMALL);
                            break;
                        case 'I':
                            lastKI += PID_STEP_SMALL;
                            break;
                        case 'd':
                            lastKD = std::max(0.0, lastKD - PID_STEP_SMALL);
                            break;
                        case 'D':
                            lastKD += PID_STEP_SMALL;
                            break;
                    }

                    // Apply new gains
                    pids[0].setGains(lastKP, lastKI, lastKD);

                    // Print updated values immediately
                    Serial.print("PID gains updated: Kp=");
                    Serial.print(lastKP, 3);
                    Serial.print(", Ki=");
                    Serial.print(lastKI, 3);
                    Serial.print(", Kd=");
                    Serial.println(lastKD, 3);
                    break;
                }
                case 's':  // Toggle step size
                    Serial.print("Step size changed to: ");
                    Serial.println(PID_STEP_SMALL, 3);
                    break;
                case '?':  // Show help
                    Serial.println("\nPID Control Commands:");
                    Serial.println("p/P - Decrease/Increase Kp");
                    Serial.println("i/I - Decrease/Increase Ki");
                    Serial.println("d/D - Decrease/Increase Kd");
                    Serial.println("s   - Toggle step size (0.01/0.1)");
                    Serial.println("?   - Show this help");
                    break;
            }
        }

        // Print status periodically (every 500ms)
        if (millis() - lastPrintTime >= 500)
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

                lastPrintTime = millis();
            }
        }

        taskYIELD();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void setup()
{
    Serial.begin(CONFIG::SYSTEM::SERIAL_BAUD_RATE);
    Serial.println("PID Motor Control System");

    initializeSystem();

    xTaskCreate(motorUpdateTask0, "MotorUpdateTask0", 4096, NULL, 3, &motorUpdateTaskHandle0);
    xTaskCreate(serialReadTask0, "SerialReadTask0", 4096, NULL, 3, &serialReadTaskHandle0);

    // Set initial target position
    pids[0].setTarget(45.5);  // Target 180 degrees
}

void loop()
{
    vTaskDelay(pdMS_TO_TICKS(10));  // Prevent watchdog timer issues
}