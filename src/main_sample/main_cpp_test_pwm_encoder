#include "MAE3Encoder.h"
#include "MotorController.h"

// Create instances
MAE3Encoder encoder(36);
// MotorController motor(/* motor parameters */);

void setup()
{
    Serial.begin(115200);
    Serial.println("Encoder-Motor Control Test");

    // Initialize both devices
    encoder.begin();
    // motor.begin();
}

void loop()
{
    static float lastPosition = 0.0f;

    // Update encoder position
    if (encoder.update())
    {
        float currentPosition = encoder.getPositionDegrees();
        float currentVelocity = encoder.getVelocityDPS();

        // Calculate position change
        float positionChange = currentPosition - lastPosition;
        lastPosition         = currentPosition;

        // Simple position control logic
        if (currentPosition < 90.0f)
        {
            // Move forward if below 90 degrees
            // motor.moveForward();
        }
        else if (currentPosition > 270.0f)
        {
            // Move reverse if above 270 degrees
            // motor.moveReverse();
        }
        else
        {
            // Stop if in the middle range
            // motor.stop();
        }

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

            Serial.print("Change: ");
            Serial.print(positionChange, 2);
            Serial.println(" degrees");

            lastPrintTime = millis();
        }
    }
}
