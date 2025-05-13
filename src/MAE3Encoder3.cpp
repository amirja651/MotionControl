#include "MAE3Encoder3.h"
#include <algorithm>

// Initialize static member
MAE3Encoder3* MAE3Encoder3::encoderInstances[MAX_ENCODERS] = {nullptr};

// Individual interrupt handlers for each encoder
void IRAM_ATTR MAE3Encoder3::interruptHandler0()
{
    if (encoderInstances[0])
    {
        encoderInstances[0]->processInterrupt();
    }
}

void IRAM_ATTR MAE3Encoder3::interruptHandler1()
{
    if (encoderInstances[1])
    {
        encoderInstances[1]->processInterrupt();
    }
}

void IRAM_ATTR MAE3Encoder3::interruptHandler2()
{
    if (encoderInstances[2])
    {
        encoderInstances[2]->processInterrupt();
    }
}

void IRAM_ATTR MAE3Encoder3::interruptHandler3()
{
    if (encoderInstances[3])
    {
        encoderInstances[3]->processInterrupt();
    }
}

MAE3Encoder3::MAE3Encoder3(uint8_t signalPin, uint8_t interruptPin, uint8_t encoderId, EncoderResolution resolution)
    : signalPin(signalPin),
      interruptPin(interruptPin),
      encoderId(encoderId),
      state{},
      error_state{},
      lastPulseTime(0),
      filterConfig{16, true, 0, 0, 0, true},  // Default: 16-sample window, enabled, adaptive filtering
      historyIndex(0),
      historyCount(0),
      validPulseCount(0),
      totalPulseCount(0),
      lastPerformanceCheck(0),
      newPulseAvailable(false),
      pulseStartTime(0),
      lastInterruptTime(0),
      resolution(resolution),
      constants(getConstants(resolution)),
      HALF_REV(constants.PULSE_PER_REV / 2),
      NOISE_THRESHOLD(constants.PULSE_PER_REV / 333),
      DIRECTION_THRESHOLD(constants.PULSE_PER_REV / 167),
      QUARTER_REV(constants.PULSE_PER_REV / 4),
      THREE_QUARTER_REV(QUARTER_REV * 3),
      VELOCITY_THRESHOLD(constants.PULSE_PER_REV * 10),      // 10 revs/second max
      ACCELERATION_THRESHOLD(constants.PULSE_PER_REV * 100)  // 100 revs/second² max
{
    resetFilter();
}

bool MAE3Encoder3::begin()
{
    if (encoderId >= MAX_ENCODERS)
    {
        return false;
    }

    // Configure pins with internal pull-up resistors for better noise immunity
    pinMode(signalPin, INPUT_PULLUP);
    pinMode(interruptPin, INPUT_PULLUP);

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
    state.currentPulse   = 0;
    state.laps           = 0;
    state.direction      = Direction::UNKNOWN;
    state.lastPulse      = 0;
    state.pulseCount     = 0;
    state.lastUpdateTime = micros();
    state.velocity       = 0.0f;
    state.acceleration   = 0.0f;

    return true;
}

void MAE3Encoder3::resetFilter()
{
    historyIndex = 0;
    historyCount = 0;
    pulseHistory.fill(0);
}

uint32_t MAE3Encoder3::applyFilter(uint32_t newPulse)
{
    if (!filterConfig.enabled || filterConfig.windowSize == 0)
    {
        return newPulse;
    }

    // Add new pulse to history
    pulseHistory[historyIndex] = newPulse;
    historyIndex               = (historyIndex + 1) % pulseHistory.size();
    if (historyCount < pulseHistory.size())
    {
        historyCount++;
    }

    // Calculate moving average using optimized loop
    uint64_t      sum      = 0;
    const uint8_t count    = (historyCount < filterConfig.windowSize) ? historyCount : filterConfig.windowSize;
    const uint8_t startIdx = (historyIndex - count) % pulseHistory.size();

    // Unrolled loop for better performance
    for (uint8_t i = 0; i < count; i += 4)
    {
        if (i + 3 < count)
        {
            sum += pulseHistory[(startIdx + i) % pulseHistory.size()];
            sum += pulseHistory[(startIdx + i + 1) % pulseHistory.size()];
            sum += pulseHistory[(startIdx + i + 2) % pulseHistory.size()];
            sum += pulseHistory[(startIdx + i + 3) % pulseHistory.size()];
        }
        else
        {
            for (uint8_t j = i; j < count; j++)
            {
                sum += pulseHistory[(startIdx + j) % pulseHistory.size()];
            }
            break;
        }
    }

    return static_cast<uint32_t>(sum / count);
}

bool MAE3Encoder3::isNoise(uint32_t newPulse) const
{
    if (!filterConfig.enabled || filterConfig.noiseThreshold == 0)
    {
        return false;
    }

    // Calculate difference from last valid pulse
    const uint32_t diff = std::abs(static_cast<int32_t>(newPulse - state.lastPulse));

    // Check if the difference exceeds the noise threshold
    return diff > filterConfig.noiseThreshold;
}

bool MAE3Encoder3::isVelocityValid(float velocity) const
{
    if (!filterConfig.enabled || filterConfig.velocityThreshold == 0)
    {
        return true;
    }

    return std::abs(velocity) <= filterConfig.velocityThreshold;
}

bool MAE3Encoder3::isAccelerationValid(float acceleration) const
{
    if (!filterConfig.enabled || filterConfig.accelerationThreshold == 0)
    {
        return true;
    }

    return std::abs(acceleration) <= filterConfig.accelerationThreshold;
}

void IRAM_ATTR MAE3Encoder3::processInterrupt()
{
    const unsigned long currentTime = micros();

    // Check for timing errors (interrupts too close together)
    if (currentTime - lastInterruptTime < 2)  // Minimum 2μs between interrupts
    {
        error_state.timing_errors++;
        return;
    }
    lastInterruptTime = currentTime;

    if (digitalRead(signalPin) == HIGH)
    {
        pulseStartTime = currentTime;
        return;
    }

    // Calculate t_on and total period
    const uint32_t t_on         = currentTime - pulseStartTime;
    const uint32_t total_period = constants.PULSE_PERIOD;

    // Optimize PWM calculation
    const uint32_t x = ((t_on * constants.PULSE_PERIOD) / total_period) - 1;

    if (x < constants.MIN_PULSE_WIDTH || x > constants.MAX_PULSE_WIDTH)
    {
        error_state.invalid_pulses++;
        return;
    }

    // Optimize position mapping
    uint32_t position;
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
        // Apply noise filtering
        if (isNoise(position))
        {
            error_state.filter_rejections++;
            return;
        }

        // Apply digital filter
        position = applyFilter(position);

        state.currentPulse = position;
        newPulseAvailable  = true;
        totalPulseCount++;
    }
}

bool MAE3Encoder3::update()
{
    if (!newPulseAvailable)
    {
        return false;
    }

    const unsigned long currentTime = micros();
    const int32_t       diff        = static_cast<int32_t>(state.currentPulse) - static_cast<int32_t>(state.lastPulse);
    const int32_t       absDiff     = (diff < 0) ? -diff : diff;  // Branchless abs

    // Fast path for small movements
    if (absDiff < HALF_REV)
    {
        if (absDiff < NOISE_THRESHOLD)
        {
            state.lastPulse   = state.currentPulse;
            newPulseAvailable = false;
            return false;
        }
    }

    // Optimize direction detection with branchless operations
    Direction newDirection;
    if (absDiff < DIRECTION_THRESHOLD)
    {
        newDirection = state.direction;
    }
    else
    {
        // Branchless direction detection
        const bool isClockwise = (diff > 0) ^ (absDiff > HALF_REV);
        newDirection           = isClockwise ? Direction::CLOCKWISE : Direction::COUNTER_CLOCKWISE;
    }

    // Corrected lap counting logic
    if (newDirection != Direction::UNKNOWN)
    {
        // Calculate raw position difference
        int32_t rawDiff = static_cast<int32_t>(state.currentPulse) - static_cast<int32_t>(state.lastPulse);

        // Adjust for wrap-around
        if (rawDiff > HALF_REV)
        {
            rawDiff -= constants.PULSE_PER_REV;
        }
        else if (rawDiff < -HALF_REV)
        {
            rawDiff += constants.PULSE_PER_REV;
        }

        // Update laps based on direction and position change
        if (newDirection == Direction::CLOCKWISE)
        {
            // For clockwise rotation, increment when crossing from high to low
            if (state.lastPulse > state.currentPulse && absDiff > HALF_REV)
            {
                state.laps++;
            }
        }
        else if (newDirection == Direction::COUNTER_CLOCKWISE)
        {
            // For counter-clockwise rotation, decrement when crossing from low to high
            if (state.lastPulse < state.currentPulse && absDiff > HALF_REV)
            {
                state.laps--;
            }
        }
    }

    // Update velocity and acceleration
    updateVelocityAndAcceleration();

    // Update state atomically
    state.direction      = newDirection;
    state.lastPulse      = state.currentPulse;
    state.lastUpdateTime = currentTime;
    newPulseAvailable    = false;
    validPulseCount++;

    return true;
}

void MAE3Encoder3::updateVelocityAndAcceleration()
{
    const unsigned long currentTime = micros();
    const float         deltaTime   = (currentTime - state.lastUpdateTime) / 1000000.0f;  // Convert to seconds

    if (deltaTime > 0)
    {
        const float newVelocity = (state.currentPulse - state.lastPulse) / deltaTime;

        if (isVelocityValid(newVelocity))
        {
            const float newAcceleration = (newVelocity - state.velocity) / deltaTime;

            if (isAccelerationValid(newAcceleration))
            {
                state.velocity     = newVelocity;
                state.acceleration = newAcceleration;
            }
            else
            {
                error_state.acceleration_errors++;
            }
        }
        else
        {
            error_state.velocity_errors++;
        }
    }
}

Direction MAE3Encoder3::detectDirection()
{
    const int32_t diff = static_cast<int32_t>(state.currentPulse - state.lastPulse);

    if (std::abs(diff) < DIRECTION_THRESHOLD)
    {
        return Direction::STATIONARY;
    }

    // Optimize direction detection
    const uint32_t halfRev = HALF_REV;

    if (diff > 0)
    {
        return (diff > halfRev) ? Direction::COUNTER_CLOCKWISE : Direction::CLOCKWISE;
    }
    else
    {
        return (std::abs(diff) > halfRev) ? Direction::CLOCKWISE : Direction::COUNTER_CLOCKWISE;
    }
}

void MAE3Encoder3::reset()
{
    state.currentPulse   = 0;
    state.laps           = 0;
    state.direction      = Direction::UNKNOWN;
    state.lastPulse      = 0;
    state.pulseCount     = 0;
    state.lastUpdateTime = micros();
    state.velocity       = 0.0f;
    state.acceleration   = 0.0f;
    resetFilter();
}

void MAE3Encoder3::setFilterConfig(const FilterConfig& config)
{
    filterConfig = config;
    resetFilter();
}

FilterConfig MAE3Encoder3::getFilterConfig() const
{
    return filterConfig;
}

void MAE3Encoder3::enableFilter(bool enable)
{
    filterConfig.enabled = enable;
    if (!enable)
    {
        resetFilter();
    }
}

void MAE3Encoder3::setAdaptiveFiltering(bool enable)
{
    filterConfig.adaptiveFiltering = enable;
}

bool MAE3Encoder3::hasErrors() const
{
    return error_state.interrupt_errors > 0 || error_state.invalid_pulses > 0 || error_state.direction_errors > 0 ||
           error_state.filter_rejections > 0 || error_state.timing_errors > 0 || error_state.velocity_errors > 0 ||
           error_state.acceleration_errors > 0 || error_state.overflow_errors > 0;
}

void MAE3Encoder3::reportErrors()
{
    if (hasErrors())
    {
        Serial.print(F("\nEncoder "));
        Serial.print(encoderId);
        Serial.print(F(" Errors - Interrupt: "));
        Serial.print(error_state.interrupt_errors);
        Serial.print(F(" Invalid Pulses: "));
        Serial.print(error_state.invalid_pulses);
        Serial.print(F(" Direction: "));
        Serial.print(error_state.direction_errors);
        Serial.print(F(" Filter Rejections: "));
        Serial.print(error_state.filter_rejections);
        Serial.print(F(" Timing: "));
        Serial.print(error_state.timing_errors);
        Serial.print(F(" Velocity: "));
        Serial.print(error_state.velocity_errors);
        Serial.print(F(" Acceleration: "));
        Serial.print(error_state.acceleration_errors);
        Serial.print(F(" Overflow: "));
        Serial.println(error_state.overflow_errors);
    }
}

void MAE3Encoder3::clearErrors()
{
    error_state.interrupt_errors    = 0;
    error_state.invalid_pulses      = 0;
    error_state.direction_errors    = 0;
    error_state.filter_rejections   = 0;
    error_state.timing_errors       = 0;
    error_state.velocity_errors     = 0;
    error_state.acceleration_errors = 0;
    error_state.overflow_errors     = 0;
}

uint32_t MAE3Encoder3::getUpdateRate() const
{
    const unsigned long currentTime = micros();
    const unsigned long elapsed     = currentTime - lastPerformanceCheck;

    if (elapsed >= 1000000)  // 1 second
    {
        return (validPulseCount * 1000000) / elapsed;
    }

    return 0;
}

float MAE3Encoder3::getFilterEfficiency() const
{
    if (totalPulseCount == 0)
    {
        return 1.0f;
    }

    return static_cast<float>(validPulseCount) / totalPulseCount;
}

uint32_t MAE3Encoder3::getValidPulseCount() const
{
    return validPulseCount;
}