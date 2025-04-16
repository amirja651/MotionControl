#include "MAE3Encoder.h"
#include "MotorController.h"
#include "MotorInstances.h"
#include "PIDController.h"

// Create instances
MAE3Encoder encoder(36);

static bool   wasAtTarget    = false;
const double  PID_STEP_SMALL = 0.01;
static double lastKp = 2.0, lastKi = 0.5, lastKd = 0.1;
PIDController pidController(&motors[0], &encoder, lastKp, lastKi, lastKd);

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
        motors[0].update();
        encoder.update();
        pidController.update();

        // Check if we just reached target
        if (!wasAtTarget && pidController.isAtTarget())
        {
            Serial.println("Target reached!");
            wasAtTarget = true;
        }
        else if (!pidController.isAtTarget())
        {
            wasAtTarget = false;
        }

        taskYIELD();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// Task for handling serial input and PID tuning
void serialReadTask0(void* pvParameters)
{
    const TickType_t     xFrequency    = pdMS_TO_TICKS(300);  // 50ms update rate
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
                            lastKp = max(0.0, lastKp - PID_STEP_SMALL);
                            break;
                        case 'P':
                            lastKp += PID_STEP_SMALL;
                            break;
                        case 'i':
                            lastKi = max(0.0, lastKi - PID_STEP_SMALL);
                            break;
                        case 'I':
                            lastKi += PID_STEP_SMALL;
                            break;
                        case 'd':
                            lastKd = max(0.0, lastKd - PID_STEP_SMALL);
                            break;
                        case 'D':
                            lastKd += PID_STEP_SMALL;
                            break;
                    }

                    // Apply new gains
                    pidController.setGains(lastKp, lastKi, lastKd);

                    // Print updated values immediately
                    Serial.print("PID gains updated: Kp=");
                    Serial.print(lastKp, 3);
                    Serial.print(", Ki=");
                    Serial.print(lastKi, 3);
                    Serial.print(", Kd=");
                    Serial.println(lastKd, 3);
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
            float currentPosition = encoder.getPositionDegrees();
            float currentVelocity = encoder.getVelocityDPS();
            float targetPosition  = pidController.getTarget();
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

        taskYIELD();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void setup()
{
    Serial.begin(CONFIG::SYSTEM::SERIAL_BAUD_RATE);
    Serial.println("PID Motor Control System");

    // Initialize devices
    encoder.begin();
    initializeMotors();
    pidController.begin();

    // Set position threshold
    pidController.setPositionThreshold(0.5f);  // 0.5 degrees threshold

    xTaskCreate(motorUpdateTask0, "MotorUpdateTask0", 4096, NULL, 3, &motorUpdateTaskHandle0);
    xTaskCreate(serialReadTask0, "SerialReadTask0", 4096, NULL, 3, &serialReadTaskHandle0);
    // Set initial target position
    pidController.setTarget(45.5);  // Target 180 degrees
}

void loop()
{
    vTaskDelay(pdMS_TO_TICKS(10));  // Prevent watchdog timer issues
}