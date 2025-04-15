#include "MAE3Encoder.h"
#include "MotorController.h"
#include "PIDController.h"

// Create instances
MAE3Encoder     encoder(36);
MotorController motor("MainMotor", Config::SPI::MOTOR1_CS, Config::TMC5160T_Driver::MOTOR1_STEP_PIN,
                      Config::TMC5160T_Driver::MOTOR1_DIR_PIN, Config::TMC5160T_Driver::MOTOR1_EN_PIN);

// Create PID controller with initial gains
PIDController pidController(&motor, &encoder, 2.0, 0.5, 0.1);

TaskHandle_t motorUpdateTaskHandle0 = NULL;

// Task for updating motor states
void motorUpdateTask0(void* pvParameters)
{
    const TickType_t xFrequency    = pdMS_TO_TICKS(10);  // Changed from 1ms to 10ms
    TickType_t       xLastWakeTime = xTaskGetTickCount();

    while (1)
    {
        motor.update();
        taskYIELD();  // Allow other tasks to run between motor updates
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void setup()
{
    Serial.begin(Config::System::SERIAL_BAUD_RATE);
    Serial.println("PID Motor Control System");

    // Initialize devices
    encoder.begin();
    motor.begin();
    pidController.begin();

    xTaskCreate(motorUpdateTask0, "MotorUpdateTask0", 4096, NULL, 3, &motorUpdateTaskHandle0);

    // Set initial target position
    pidController.setTarget(180.0);  // Target 180 degrees
}

void loop()
{
    // Update encoder position
    if (encoder.update())
    {
        float currentPosition = encoder.getPositionDegrees();
        float currentVelocity = encoder.getVelocityDPS();

        // Update PID controller
        pidController.update();

        // Print status every 100ms
        static unsigned long lastPrintTime = 0;
        if (millis() - lastPrintTime >= 100)
        {
            Serial.print("Position: ");
            Serial.print(currentPosition, 2);
            Serial.print(" degrees, ");

            Serial.print("Velocity: ");
            Serial.print(currentVelocity, 2);
            Serial.print(" deg/s, ");

            Serial.print("Target: ");
            Serial.print(pidController.getTarget(), 2);
            Serial.println(" degrees");

            lastPrintTime = millis();
        }
    }
}
