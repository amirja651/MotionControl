#include "MAE3Encoder.h"
#include "ObjectInstances.h"

// Array to store encoder instances for interrupt handling
static MAE3Encoder* encoderInstances[4] = {nullptr};  // Support up to NUM_MOTORS encoders
static uint8_t      numEncoders         = 0;

// Static interrupt handlers for each encoder
static void encoderInterruptHandler0()
{
    if (encoderInstances[0])
        encoderInstances[0]->handleInterrupt();
}
static void encoderInterruptHandler1()
{
    if (encoderInstances[1])
        encoderInstances[1]->handleInterrupt();
}
static void encoderInterruptHandler2()
{
    if (encoderInstances[2])
        encoderInstances[2]->handleInterrupt();
}
static void encoderInterruptHandler3()
{
    if (encoderInstances[3])
        encoderInstances[3]->handleInterrupt();
}

// Array of interrupt handlers (explicitly sized to match NUM_MOTORS)
static void (*interruptHandlers[4])() = {encoderInterruptHandler0, encoderInterruptHandler1, encoderInterruptHandler2,
                                         encoderInterruptHandler3};

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
        uint8_t instanceIndex           = numEncoders++;
        encoderInstances[instanceIndex] = this;
        attachInterrupt(digitalPinToInterrupt(interruptPin), interruptHandlers[instanceIndex], CHANGE);
    }
}

bool flag1 = false;
bool flag2 = false;
bool flag3 = false;
bool flag4 = false;
bool flag5 = false;
bool flag6 = false;

void MAE3Encoder::handleInterrupt()
{
    flag1                     = true;
    unsigned long currentTime = micros();
    if (digitalRead(interruptPin) == HIGH)
    {
        pulseStartTime = currentTime;
        flag2          = true;
    }
    else
    {
        flag3 = true;
        if (pulseStartTime > 0)
        {
            flag4               = true;
            uint32_t pulseWidth = currentTime - pulseStartTime;
            if (pulseWidth >= MIN_PULSE_WIDTH && pulseWidth <= MAX_PULSE_WIDTH)
            {
                flag5             = true;
                currentPulseWidth = pulseWidth;
                newPulseAvailable = true;
            }
            flag6 = true;
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
    float positionChange = fabs(newPosition - lastValidPosition);

    // Handle wrap-around at 0/360 degrees
    if (positionChange > 180.0f)
    {
        positionChange = 360.0f - positionChange;
    }

    // Only update if position change exceeds threshold
    if (positionChange >= 0.1f)
    {
        unsigned long currentTime = micros();
        float         deltaTime   = (currentTime - velocityUpdateTime) * 1e-6f;

        if (deltaTime > 0)
        {
            // Calculate velocity based on the shortest path
            float shortestPath = positionChange;
            if (fabs(newPosition - lastValidPosition) > 180.0f)
            {
                shortestPath = 360.0f - shortestPath;
            }

            // Determine direction of movement
            float direction = (newPosition > lastValidPosition) ? 1.0f : -1.0f;
            if (fabs(newPosition - lastValidPosition) > 180.0f)
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

uint32_t MAE3Encoder::medianFilter()
{
    // Simple insertion sort for small arrays
    uint32_t sorted[FILTER_SIZE];
    memcpy(sorted, pulseFilter, sizeof(pulseFilter));

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
