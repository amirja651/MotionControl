#include "MAE3Encoder.h"
#include "Config/System_Config.h"
// Array to store encoder instances for interrupt handling
static MAE3Encoder* encoderInstances[NUM_MOTORS] = {nullptr};  // Support up to [NUM_MOTORS] encoders
static uint8_t      numEncoders                  = 0;

// Static interrupt handler that dispatches to the correct instance
static void encoderInterruptHandler()
{
    for (uint8_t i = 0; i < numEncoders; i++)
    {
        if (encoderInstances[i])
        {
            encoderInstances[i]->handleInterrupt();
        }
    }
}

// Floating-point map function
float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

MAE3Encoder::MAE3Encoder(uint8_t signalPin, uint8_t interruptPin)
    : signalPin(signalPin),
      interruptPin(interruptPin),
      currentPulseWidth(0),
      newPulseAvailable(false),
      pulseStartTime(0),
      filterIndex(0),
      lastValidPosition(0.0f),
      currentVelocity(0.0f),
      velocityUpdateTime(0),
      lastUpdateTime(0)
{
    // Initialize filter array
    for (uint8_t i = 0; i < FILTER_SIZE; i++)
    {
        pulseFilter[i] = 0;
    }
}

void MAE3Encoder::begin()
{
    pinMode(signalPin, INPUT);
    pinMode(interruptPin, INPUT);

    // Register this instance for interrupt handling
    if (numEncoders < NUM_MOTORS)
    {
        encoderInstances[numEncoders++] = this;
        if (numEncoders == 1)
        {
            // Only attach interrupt once
            attachInterrupt(digitalPinToInterrupt(interruptPin), encoderInterruptHandler, CHANGE);
        }
    }
}

void MAE3Encoder::handleInterrupt()
{
    unsigned long currentTime = micros();
    if (digitalRead(interruptPin) == HIGH)
    {
        pulseStartTime = currentTime;
    }
    else
    {
        if (pulseStartTime > 0)
        {
            uint32_t pulseWidth = currentTime - pulseStartTime;
            if (pulseWidth >= MIN_PULSE_WIDTH && pulseWidth <= MAX_PULSE_WIDTH)
            {
                currentPulseWidth = pulseWidth;
                newPulseAvailable = true;
            }
        }
    }
}

bool MAE3Encoder::update()
{
    if (!newPulseAvailable)
    {
        return false;
    }

    uint32_t pulseWidth;
    {
        portDISABLE_INTERRUPTS();
        pulseWidth        = currentPulseWidth;
        newPulseAvailable = false;
        portENABLE_INTERRUPTS();
    }

    // Apply median filter
    pulseFilter[filterIndex] = pulseWidth;
    filterIndex              = (filterIndex + 1) % FILTER_SIZE;

    uint32_t filteredPulse = medianFilter();

    // Update position and velocity
    float newPosition = filteredPulse * PULSE_TO_DEGREE;

    // Check if position change exceeds threshold
    float positionChange = abs(newPosition - lastValidPosition);

    // Handle wrap-around at 0/360 degrees
    if (positionChange > 180.0f)
    {
        positionChange = 360.0f - positionChange;
    }

    // Only update if position change exceeds threshold
    if (positionChange >= POSITION_THRESHOLD)
    {
        unsigned long currentTime = micros();
        float         deltaTime   = (currentTime - velocityUpdateTime) * 1e-6f;

        if (deltaTime > 0)
        {
            // Calculate velocity based on the shortest path
            float shortestPath = positionChange;
            if (abs(newPosition - lastValidPosition) > 180.0f)
            {
                shortestPath = 360.0f - shortestPath;
            }

            // Determine direction of movement
            float direction = (newPosition > lastValidPosition) ? 1.0f : -1.0f;
            if (abs(newPosition - lastValidPosition) > 180.0f)
            {
                direction *= -1.0f;
            }

            currentVelocity    = (shortestPath * direction) / deltaTime;
            lastValidPosition  = newPosition;
            velocityUpdateTime = currentTime;
            return true;
        }
    }

    return false;
}

uint32_t MAE3Encoder::getPulseWidth() const
{
    return pulseFilter[filterIndex];
}

float MAE3Encoder::getPositionDegrees() const
{
    return lastValidPosition;
}

float MAE3Encoder::getVelocityDPS() const
{
    return currentVelocity;
}

uint32_t MAE3Encoder::convertToPulseWidth(float degree)
{
    return static_cast<uint32_t>(degree * DEGREE_TO_PULSE);
}

uint32_t MAE3Encoder::medianFilter()
{
    uint32_t sorted[FILTER_SIZE];
    memcpy(sorted, pulseFilter, sizeof(pulseFilter));

    // Simple insertion sort for small arrays
    for (uint8_t i = 1; i < FILTER_SIZE; i++)
    {
        uint32_t key = sorted[i];
        int8_t   j   = i - 1;
        while (j >= 0 && sorted[j] > key)
        {
            sorted[j + 1] = sorted[j];
            j--;
        }
        sorted[j + 1] = key;
    }
    return sorted[FILTER_SIZE / 2];
}
