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

MAE3Encoder2::MAE3Encoder2(uint8_t signalPin, uint8_t interruptPin, uint8_t encoderId)
    : signalPin(signalPin),
      interruptPin(interruptPin),
      encoderId(encoderId),
      state{},
      velocityDPS(0.0f),
      filterIndex(0),
      lastPulseTime(0),
      lastVelocityUpdate(0),
      errorCount(0),
      validPulseCount(0),
      currentPulseWidth(0),
      newPulseAvailable(false),
      pulseStartTime(0)
{
    // Initialize filter array
    pulseFilter.fill(0);
}

bool MAE3Encoder2::begin()
{
    if (encoderId >= MAX_ENCODERS)
    {
        handleError(EncoderError::COMMUNICATION_ERROR);
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
    state.currentPulse   = 0;
    state.totalPulses    = 0;
    state.laps           = 0;
    state.direction      = Direction::UNKNOWN;
    state.error          = EncoderError::NONE;
    state.lastValidPulse = 0;
    state.lastUpdateTime = micros();

    return true;
}

bool MAE3Encoder2::update()
{
    if (!newPulseAvailable)
    {
        return false;
    }

    // Get filtered pulse width
    uint32_t filteredPulse = medianFilter();

    // Validate pulse
    if (!validatePulse(filteredPulse))
    {
        handleError(EncoderError::INVALID_PULSE_WIDTH);
        return false;
    }

    // Calculate current pulse value (0-4095)
    uint32_t newPulse = map(filteredPulse, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH, 0, PULSE_PER_REV - 1);

    // Detect direction
    state.direction = detectDirection(newPulse);

    // Update state
    state.lastValidPulse = state.currentPulse;
    state.currentPulse   = newPulse;
    state.totalPulses++;

    // Update lap count
    if (state.direction == Direction::CLOCKWISE)
    {
        if (newPulse < state.lastValidPulse && (state.lastValidPulse - newPulse) > (PULSE_PER_REV / 2))
        {
            state.laps++;
        }
    }
    else if (state.direction == Direction::COUNTER_CLOCKWISE)
    {
        if (newPulse > state.lastValidPulse && (newPulse - state.lastValidPulse) > (PULSE_PER_REV / 2))
        {
            state.laps--;
        }
    }

    // Calculate velocity
    unsigned long currentTime = micros();
    if (currentTime - lastVelocityUpdate >= 100000)
    {  // Update every 100ms
        float deltaTime    = (currentTime - lastVelocityUpdate) / 1000000.0f;
        float deltaAngle   = (state.currentPulse - state.lastValidPulse) * DEGREES_PER_PULSE;
        velocityDPS        = deltaAngle / deltaTime;
        lastVelocityUpdate = currentTime;
    }

    state.lastUpdateTime = currentTime;
    newPulseAvailable    = false;
    validPulseCount++;

    return true;
}

uint32_t MAE3Encoder2::medianFilter()
{
    // Sort the array
    std::array<uint32_t, 5> sorted = pulseFilter;
    std::sort(sorted.begin(), sorted.end());

    // Return median value
    return sorted[2];
}

Direction MAE3Encoder2::detectDirection(uint32_t newPulse)
{
    if (newPulse == state.lastValidPulse)
    {
        return state.direction;
    }

    // Handle wrap-around cases
    uint32_t diff;
    if (newPulse > state.lastValidPulse)
    {
        diff = newPulse - state.lastValidPulse;
        if (diff > (PULSE_PER_REV / 2))
        {
            return Direction::COUNTER_CLOCKWISE;
        }
        return Direction::CLOCKWISE;
    }
    else
    {
        diff = state.lastValidPulse - newPulse;
        if (diff > (PULSE_PER_REV / 2))
        {
            return Direction::CLOCKWISE;
        }
        return Direction::COUNTER_CLOCKWISE;
    }
}

bool MAE3Encoder2::validatePulse(uint32_t pulse)
{
    // Check pulse width range
    if (pulse < MIN_PULSE_WIDTH || pulse > MAX_PULSE_WIDTH)
    {
        errorCount++;
        return false;
    }

    // Check for sudden changes (noise detection)
    if (state.lastValidPulse > 0)
    {
        uint32_t diff = std::abs(static_cast<int32_t>(pulse - state.lastValidPulse));
        if (diff > (MAX_PULSE_WIDTH - MIN_PULSE_WIDTH) / 4)
        {
            errorCount++;
            return false;
        }
    }

    return true;
}

void MAE3Encoder2::handleError(EncoderError error)
{
    state.error = error;
    errorCount++;

    // Implement error recovery strategies
    switch (error)
    {
        case EncoderError::INVALID_PULSE_WIDTH:
            // Use last valid pulse
            state.currentPulse = state.lastValidPulse;
            break;
        case EncoderError::SIGNAL_LOST:
            // Attempt to recover by reinitializing
            begin();
            break;
        default:
            break;
    }
}

void MAE3Encoder2::reset()
{
    state.currentPulse   = 0;
    state.totalPulses    = 0;
    state.laps           = 0;
    state.direction      = Direction::UNKNOWN;
    state.error          = EncoderError::NONE;
    state.lastValidPulse = 0;
    errorCount           = 0;
    validPulseCount      = 0;
    velocityDPS          = 0.0f;
    pulseFilter.fill(0);
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
        uint32_t total_period = PULSE_PERIOD;  // 4098 μs

        // Implement the PWM formula: x = ((t_on * 4098) / (t_on + t_off)) - 1
        uint32_t x = ((t_on * PULSE_PERIOD) / total_period) - 1;

        // Apply position mapping rules
        uint32_t position;
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

        currentPulseWidth        = position;
        pulseFilter[filterIndex] = position;
        filterIndex              = (filterIndex + 1) % pulseFilter.size();
        newPulseAvailable        = true;
    }
}