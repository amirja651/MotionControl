#include "MAE3Encoder2.h"
#include "UnitConversion.h"
#include <algorithm>

// Initialize static member
MAE3Encoder2* MAE3Encoder2::encoderInstances[MAX_ENCODERS] = {nullptr};

// Individual interrupt handlers for each encoder
void IRAM_ATTR MAE3Encoder2::interruptHandler0()
{
    if (encoderInstances[0])
    {
        encoderInstances[0]->processInterrupt();
    }
}

void IRAM_ATTR MAE3Encoder2::interruptHandler1()
{
    if (encoderInstances[1])
    {
        encoderInstances[1]->processInterrupt();
    }
}

void IRAM_ATTR MAE3Encoder2::interruptHandler2()
{
    if (encoderInstances[2])
    {
        encoderInstances[2]->processInterrupt();
    }
}

void IRAM_ATTR MAE3Encoder2::interruptHandler3()
{
    if (encoderInstances[3])
    {
        try
        {
            encoderInstances[3]->processInterrupt();
        }
        catch (...)
        {
            // Log error in a way that's safe for interrupts
            volatile static uint32_t error_count = 0;
            error_count++;
        }
    }
}

MAE3Encoder2::MAE3Encoder2(uint8_t signalPin, uint8_t interruptPin, uint8_t encoderId, EncoderResolution resolution)
    : signalPin(signalPin),
      interruptPin(interruptPin),
      encoderId(encoderId),
      state{},
      lastPulseTime(0),
      newPulseAvailable(false),
      pulseStartTime(0),
      resolution(resolution),
      constants(getConstants(resolution))
{
}

bool MAE3Encoder2::begin()
{
    if (encoderId >= MAX_ENCODERS)
    {
        return false;
    }

    lowerLimitMm = pxToMm(LINEAR_LOWER_LIMIT_PX);
    upperLimitMm = pxToMm(LINEAR_UPPER_LIMIT_PX);
    // Configure pins
    pinMode(signalPin, INPUT);
    pinMode(interruptPin, INPUT);

    // Store instance for interrupt handling
    encoderInstances[encoderId] = this;

    // Configure interrupt based on encoder ID
    switch (encoderId)
    {
        case 0:
            attachInterrupt(digitalPinToInterrupt(interruptPin), interruptHandler0, CHANGE);
            break;
        case 1:
            attachInterrupt(digitalPinToInterrupt(interruptPin), interruptHandler1, CHANGE);
            break;
        case 2:
            attachInterrupt(digitalPinToInterrupt(interruptPin), interruptHandler2, CHANGE);
            break;
        case 3:
            attachInterrupt(digitalPinToInterrupt(interruptPin), interruptHandler3, CHANGE);
            break;
    }

    // Initialize state
    state.currentPulse = 0;
    state.laps         = 0;
    state.direction    = Direction::UNKNOWN;
    state.lastPulse    = 0;

    return true;
}

void MAE3Encoder2::processInterrupt()
{
    unsigned long currentTime = micros();

    if (digitalRead(signalPin) == HIGH)
    {
        pulseStartTime = currentTime;
        return;
    }

    // Calculate t_on and total period
    uint32_t t_on         = currentTime - pulseStartTime;
    uint32_t total_period = constants.PULSE_PERIOD;
    uint32_t position     = 0;  // Implement PWM formula based on resolution

    // 10-bit PWM formula: x = ((t_on * 1026) / (t_on + t_off)) - 1
    // 12-bit PWM formula: x = ((t_on * 4098) / (t_on + t_off)) - 1
    uint32_t x = ((t_on * constants.PULSE_PERIOD) / total_period) - 1;

    if (x < constants.MIN_PULSE_WIDTH || x > constants.MAX_PULSE_WIDTH)
    {
        return;  // Ignore invalid pulse widths
    }

    // Apply 10-bit position mapping rules
    // Apply 12-bit position mapping rules
    if (x <= constants.PULSE_PER_REV - 2)
    {
        position = x;
    }
    else if (x == constants.PULSE_PER_REV)
    {
        position = constants.PULSE_PER_REV - 1;
    }
    else
    {
        position = x;
    }

    if (position != 0)
    {
        state.currentPulse = position;
        newPulseAvailable  = true;
    }
}

bool MAE3Encoder2::update()
{
    if (!newPulseAvailable)
    {
        return false;
    }

    volatile int32_t diff = static_cast<int32_t>(state.currentPulse) - static_cast<int32_t>(state.lastPulse);

    // Skip noise check if large jump (possible wrap-around)
    if (abs(diff) < (int32_t)(constants.PULSE_PER_REV / 2))
    {
        volatile uint32_t noiseThreshold = constants.PULSE_PER_REV * 3 / 1000;
        if (abs(diff) < (int32_t)noiseThreshold)
            return false;
    }

    // Calculate direction before updating state
    volatile Direction newDirection = detectDirection();

    // Handle lap counting
    if (newDirection != Direction::UNKNOWN)
    {
        // Check for clockwise lap completion
        if (newDirection == Direction::CLOCKWISE)
        {
            if (state.lastPulse > (constants.PULSE_PER_REV * 3 / 4) && state.currentPulse < (constants.PULSE_PER_REV / 4))
            {
                state.laps++;
            }
        }
        // Check for counter-clockwise lap completion
        else if (newDirection == Direction::COUNTER_CLOCKWISE)
        {
            if (state.lastPulse < (constants.PULSE_PER_REV / 4) && state.currentPulse > (constants.PULSE_PER_REV * 3 / 4))
            {
                state.laps--;
            }
        }
    }

    // Update state
    state.direction   = newDirection;
    state.lastPulse   = state.currentPulse;
    newPulseAvailable = false;
    return true;
}

Direction MAE3Encoder2::detectDirection()
{
    uint32_t noiseThreshold = constants.PULSE_PER_REV * 6 / 1000;
    if (std::abs(static_cast<int32_t>(state.currentPulse - state.lastPulse)) < noiseThreshold)
    {
        return state.direction;
    }

    // Handle wrap-around cases
    uint32_t diff;
    if (state.currentPulse > state.lastPulse)
    {
        diff = state.currentPulse - state.lastPulse;
        if (diff > (constants.PULSE_PER_REV / 2))
        {
            return Direction::COUNTER_CLOCKWISE;
        }
        return Direction::CLOCKWISE;
    }
    else
    {
        diff = state.lastPulse - state.currentPulse;
        if (diff > (constants.PULSE_PER_REV / 2))
        {
            return Direction::CLOCKWISE;
        }
        return Direction::COUNTER_CLOCKWISE;
    }
}

void MAE3Encoder2::reset()
{
    state.currentPulse = 0;
    state.laps         = 0;
    state.direction    = Direction::UNKNOWN;
    state.lastPulse    = 0;
}

void MAE3Encoder2::setLowerLimits(double lowerLimitMm)
{
    this->lowerLimitMm = lowerLimitMm;
}

void MAE3Encoder2::setUpperLimits(double upperLimitMm)
{
    this->upperLimitMm = upperLimitMm;
}

// Calculate laps from current pulse and current position in pixels
void MAE3Encoder2::calculateLaps(double currentPositionMm)
{
    delay(1000);
    update();
    delay(1000);
    double pulsePositionMm = state.currentPulse * getMillimetersPerPulse();
    double laps            = (currentPositionMm - pulsePositionMm) / LEAD_SCREW_PITCH_MM;

    // Print for debugging
    Serial.printf("Current Position (μm): %.12f\n", currentPositionMm);
    Serial.printf("Pulse Position (μm): %.12f\n", pulsePositionMm);
    Serial.printf("Difference (μm): %.12f\n", currentPositionMm - pulsePositionMm);
    Serial.printf("Laps (raw): %.12f\n", laps);

    // Round and update state
    state.laps = static_cast<int32_t>(round(laps));
    Serial.printf("State laps: %d\n", state.laps);
}
