#include "MAE3Encoder.h"
#include <algorithm>
#include <esp_timer.h>
#include <map>

// Initialize static member
MAE3Encoder* MAE3Encoder::encoderInstances[MAX_ENCODERS] = {nullptr};

// Individual interrupt handlers for each encoder
void IRAM_ATTR MAE3Encoder::interruptHandler0()
{
    if (encoderInstances[0] && encoderInstances[0]->enabled)
    {
        encoderInstances[0]->processInterrupt();
    }
}

void IRAM_ATTR MAE3Encoder::interruptHandler1()
{
    if (encoderInstances[1] && encoderInstances[1]->enabled)
    {
        encoderInstances[1]->processInterrupt();
    }
}

void IRAM_ATTR MAE3Encoder::interruptHandler2()
{
    if (encoderInstances[2] && encoderInstances[2]->enabled)
    {
        encoderInstances[2]->processInterrupt();
    }
}

void IRAM_ATTR MAE3Encoder::interruptHandler3()
{
    if (encoderInstances[3] && encoderInstances[3]->enabled)
    {
        encoderInstances[3]->processInterrupt();
    }
}

MAE3Encoder::MAE3Encoder(uint8_t signalPin, uint8_t encoderId)
    : signalPin(signalPin),
      encoderId(encoderId),
      state{},
      enabled(false),
      max_t(DEFAULT_MAX_T),
      lastPulseTime(0),
      lastFallingEdgeTime(0),
      lastRisingEdgeTime(0),
      newPulseAvailable(false),
      bufferUpdated(false),
      calibrationBuffer{},
      calibrationIndex(0),
      widthLBuffer{},
      widthHBuffer{},
      pulseBufferIndex(0)
{
}

MAE3Encoder::~MAE3Encoder()
{
    disable();
    if (encoderId < MAX_ENCODERS)
    {
        encoderInstances[encoderId] = nullptr;
    }
}

bool MAE3Encoder::begin()
{
    if (encoderId >= MAX_ENCODERS)
    {
        return false;
    }

    // Configure pins
    pinMode(signalPin, INPUT);

    // Store instance for interrupt handling
    encoderInstances[encoderId] = this;

    // Initialize state
    reset();

    // Initialize calibration buffer
    calibrationBuffer.fill(0);
    calibrationIndex = 0;

    // Encoder starts disabled by default
    enabled = false;

    return true;
}

bool MAE3Encoder::calibrate()
{
    if (enabled)
    {
        disable();  // Temporarily disable encoder
    }

    // Enable encoder for calibration
    enabled = true;
    attachInterruptHandler();

    // Clear calibration buffer
    calibrationBuffer.fill(0);
    calibrationIndex = 0;

    // Wait for buffer to fill
    uint64_t startTime = millis();
    while (calibrationIndex < CALIBRATION_BUFFER_SIZE && (millis() - startTime) < 5000)  // 5 second timeout
    {
        if (newPulseAvailable)
        {
            calibrationBuffer[calibrationIndex++] = state.period;
            newPulseAvailable                     = false;
        }
        delay(1);  // Small delay to prevent CPU hogging
    }

    // Disable encoder after calibration
    disable();

    if (calibrationIndex < CALIBRATION_BUFFER_SIZE)
    {
        return false;  // Calibration failed - didn't get enough samples
    }

    // Find most frequent value and update max_t
    max_t = findMostFrequentValue(calibrationBuffer);
    return true;
}

uint32_t MAE3Encoder::findMostFrequentValue(const std::array<uint32_t, CALIBRATION_BUFFER_SIZE>& buffer)
{
    std::map<uint32_t, uint32_t> frequencyMap;

    // Count frequency of each value
    for (uint32_t value : buffer)
    {
        if (value > 0)  // Ignore zero values
        {
            frequencyMap[value]++;
        }
    }

    // Find value with highest frequency
    uint32_t mostFrequentValue = DEFAULT_MAX_T;
    uint32_t maxFrequency      = 0;

    for (const auto& pair : frequencyMap)
    {
        if (pair.second > maxFrequency)
        {
            maxFrequency      = pair.second;
            mostFrequentValue = pair.first;
        }
    }

    return mostFrequentValue;
}

void MAE3Encoder::enable()
{
    if (!enabled)
    {
        enabled = true;
        attachInterruptHandler();
    }
}

void MAE3Encoder::disable()
{
    if (enabled)
    {
        enabled = false;
        detachInterruptHandler();
    }
}

void MAE3Encoder::attachInterruptHandler()
{
    switch (encoderId)
    {
        case 0:
            attachInterrupt(digitalPinToInterrupt(signalPin), interruptHandler0, CHANGE);
            break;
        case 1:
            attachInterrupt(digitalPinToInterrupt(signalPin), interruptHandler1, CHANGE);
            break;
        case 2:
            attachInterrupt(digitalPinToInterrupt(signalPin), interruptHandler2, CHANGE);
            break;
        case 3:
            attachInterrupt(digitalPinToInterrupt(signalPin), interruptHandler3, CHANGE);
            break;
    }
}

void MAE3Encoder::detachInterruptHandler()
{
    detachInterrupt(digitalPinToInterrupt(signalPin));
}

void IRAM_ATTR MAE3Encoder::processInterrupt()
{
    if (!enabled)
        return;

    volatile uint64_t currentTime = esp_timer_get_time();

    if (digitalRead(signalPin) == HIGH)
    {
        // Rising edge
        lastRisingEdgeTime = currentTime;
        if (lastFallingEdgeTime != 0)
        {
            volatile uint64_t width_high = lastRisingEdgeTime - lastFallingEdgeTime;
            state.width_high             = width_high;
            if (width_high > max_t)
            {
                return;
            }
            // Measure low pulse width (falling to rising)
            widthLBuffer[pulseBufferIndex] = width_high;
        }
    }
    else
    {
        // Falling edge
        lastFallingEdgeTime = currentTime;
        if (lastRisingEdgeTime != 0)
        {
            volatile uint64_t width_low = lastFallingEdgeTime - lastRisingEdgeTime;
            state.width_low             = width_low;
            if (width_low > max_t)
            {
                return;
            }
            // Measure high pulse width (rising to falling)
            widthHBuffer[pulseBufferIndex] = width_low;
        }
        // Increment the index of the rotating buffer in a circular manner only after each complete cycle (falling edge)
        pulseBufferIndex = (pulseBufferIndex + 1) % PULSE_BUFFER_SIZE;
        bufferUpdated    = true;
    }
}

// New method for processing PWM signal
void MAE3Encoder::processPWM()
{
    if (!enabled || !bufferUpdated)
        return;

    // Last valid index (last complete cycle)
    // size_t   lastIdx = (pulseBufferIndex + PULSE_BUFFER_SIZE - 1) % PULSE_BUFFER_SIZE;
    uint64_t width_h = getMedianWidthHigh();  // widthHBuffer[lastIdx];
    uint64_t width_l = getMedianWidthLow();   // widthLBuffer[lastIdx];
    uint64_t period  = width_h + width_l;
    state.period     = period;

    if (period == 0)
        return;

    // Optimized calculation for x_measured
    uint64_t x_measured = (width_h * (max_t + 2)) / period - 1;

    // Boundary check with early return
    if (x_measured <= (max_t - 2))
    {
    }
    else if (x_measured == max_t)
    {
        x_measured = max_t - 1;
    }
    else
    {
        return;
    }
    state.last_angle    = getPositionDegrees();  // it should be before state.current_Pulse
    state.current_Pulse = x_measured;
    bufferUpdated       = false;
    newPulseAvailable   = true;
    lastPulseTime       = esp_timer_get_time();
}

void MAE3Encoder::updateDirectionAndLaps()
{
    if (!enabled || !newPulseAvailable)
        return;

    // Detect direction and count laps
    float currentAngle = getPositionDegrees();
    state.delta        = currentAngle - state.last_angle;

    // Normalize delta to -180 to 180 degrees
    if (state.delta > 180)
        state.delta -= 360;
    if (state.delta < -180)
        state.delta += 360;

    const float threshold = 0.3f;

    if (abs(state.delta) < threshold)
    {
        state.direction = Direction::UNKNOWN;
    }
    else if (state.delta > 0)
    {
        // Clockwise
        state.direction = Direction::CLOCKWISE;
        if (state.last_angle > 300 && currentAngle < 60)
            state.laps++;
    }
    else if (state.delta < 0)
    {
        // Counter-clockwise
        state.direction = Direction::COUNTER_CLOCKWISE;
        if (state.last_angle < 60 && currentAngle > 300)
            state.laps--;
    }

    newPulseAvailable = false;
}

void MAE3Encoder::reset()
{
    state.current_Pulse     = 0;
    state.current_Pulse     = 0;
    state.width_high        = 0;
    state.width_low         = 0;
    state.period            = 0;
    state.laps              = 0;
    state.absolute_position = 0;
    state.direction         = Direction::UNKNOWN;
    state.last_angle        = 0.0f;
    state.delta             = 0.0f;
    newPulseAvailable       = false;
    lastPulseTime           = 0;
    lastFallingEdgeTime     = 0;
    lastRisingEdgeTime      = 0;
}

uint64_t MAE3Encoder::getMedianWidthHigh() const
{
    std::array<uint64_t, PULSE_BUFFER_SIZE> temp = widthHBuffer;
    std::sort(temp.begin(), temp.end());
    return temp[PULSE_BUFFER_SIZE / 2];
}

uint64_t MAE3Encoder::getMedianWidthLow() const
{
    std::array<uint64_t, PULSE_BUFFER_SIZE> temp = widthLBuffer;
    std::sort(temp.begin(), temp.end());
    return temp[PULSE_BUFFER_SIZE / 2];
}
