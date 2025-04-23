#include "MAE3Encoder2.h"
#include <algorithm>

// Static array to store encoder instances for interrupt handling
static MAE3Encoder2* encoderInstances[MAX_ENCODERS] = {nullptr};

// Static interrupt handler
static void IRAM_ATTR staticInterruptHandler()
{
    for (auto& instance : encoderInstances)
    {
        if (instance)
        {
            instance->processInterrupt();
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

    // Configure pins
    pinMode(signalPin, INPUT);
    pinMode(interruptPin, INPUT);

    // Store instance for interrupt handling
    encoderInstances[encoderId] = this;

    // Configure interrupt
    attachInterrupt(digitalPinToInterrupt(interruptPin), staticInterruptHandler, CHANGE);

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
    }
    else
    {
        // Calculate t_on and total period
        uint32_t t_on         = currentTime - pulseStartTime;
        uint32_t total_period = constants.PULSE_PERIOD;

        // Implement PWM formula based on resolution
        uint32_t position;
        if (resolution == EncoderResolution::BITS_10)
        {
            // 10-bit PWM formula: x = ((t_on * 1026) / (t_on + t_off)) - 1
            uint32_t x = ((t_on * constants.PULSE_PERIOD) / total_period) - 1;

            // Apply 10-bit position mapping rules
            if (x <= 1022)
            {
                position = x;
            }
            else if (x == 1024)
            {
                position = 1023;
            }
            else
            {
                position = x;
            }
        }
        else
        {
            // 12-bit PWM formula: x = ((t_on * 4098) / (t_on + t_off)) - 1
            uint32_t x = ((t_on * constants.PULSE_PERIOD) / total_period) - 1;

            // Apply 12-bit position mapping rules
            if (x <= 4094)
            {
                position = x;
            }
            else if (x == 4096)
            {
                position = 4095;
            }
            else
            {
                position = x;
            }
        }

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

    // Adjust noise threshold based on resolution (0.6% of full scale)
    uint32_t noiseThreshold = constants.PULSE_PER_REV * 6 / 1000;
    if (std::abs(static_cast<int32_t>(state.currentPulse - state.lastPulse)) < noiseThreshold)
    {
        return false;
    }

    // Calculate direction before updating state
    Direction newDirection = detectDirection();

    // Handle lap counting
    if (newDirection != Direction::UNKNOWN)
    {
        // Check for clockwise lap completion
        if (newDirection == Direction::CLOCKWISE)
        {
            if (state.lastPulse > (constants.PULSE_PER_REV * 3 / 4) &&
                state.currentPulse < (constants.PULSE_PER_REV / 4))
            {
                state.laps++;
            }
        }
        // Check for counter-clockwise lap completion
        else if (newDirection == Direction::COUNTER_CLOCKWISE)
        {
            if (state.lastPulse < (constants.PULSE_PER_REV / 4) &&
                state.currentPulse > (constants.PULSE_PER_REV * 3 / 4))
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
