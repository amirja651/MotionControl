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
    int64_t startTime = millis();
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

    volatile int64_t currentTime = esp_timer_get_time();

    if (digitalRead(signalPin) == HIGH)
    {
        // Rising edge
        lastRisingEdgeTime = currentTime;
        if (lastFallingEdgeTime != 0)
        {
            volatile int64_t width_high = lastRisingEdgeTime - lastFallingEdgeTime;
            state.width_high            = width_high;
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
            volatile int64_t width_low = lastFallingEdgeTime - lastRisingEdgeTime;
            state.width_low            = width_low;
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

    int64_t width_h = getMedianWidthHigh();
    int64_t width_l = getMedianWidthLow();
    int64_t period  = width_h + width_l;
    state.period    = period;

    if (period == 0)
        return;

    // Optimized calculation for x_measured
    int64_t x_measured = (width_h * (max_t + 2)) / period - 1;

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

    state.current_pulse = x_measured;
    newPulseAvailable   = true;
    bufferUpdated       = false;
    lastPulseTime       = esp_timer_get_time();

    // Update sector tracking
    updateSectorTracking();
}

void MAE3Encoder::updateSectorTracking()
{
    // Calculate current sector (0-59)
    uint8_t new_sector = (state.current_pulse * EncoderState::NUM_SECTORS) / max_t;

    // If sector changed
    if (new_sector != state.current_sector)
    {
        int8_t sector_diff = 0;

        state.last_sector    = state.current_sector;
        state.current_sector = new_sector;

        sector_diff = new_sector - state.last_sector;

        // Handle wraparound
        if (sector_diff > EncoderState::NUM_SECTORS / 2)
            sector_diff -= EncoderState::NUM_SECTORS;
        else if (sector_diff < -EncoderState::NUM_SECTORS / 2)
            sector_diff += EncoderState::NUM_SECTORS;

        state.direction = (sector_diff > 0) ? Direction::CLOCKWISE : Direction::COUNTER_CLOCKWISE;

        // Check if this sector was previously untouched
        int prev = (new_sector == 0) ? 63 : new_sector - 1;
        int next = (new_sector == 63) ? 0 : new_sector + 1;

        uint64_t sector_bit = (1ULL << new_sector) | (1ULL << prev) | (1ULL << next);

        if (state.direction == Direction::CLOCKWISE)
        {
            if (state.touched_sectors_CCW & sector_bit)
            {
                state.touched_sectors_CCW &= ~sector_bit;
                // فقط اگر واقعاً چیزی پاک شده، count کم کن
                state.touched_count_CCW = __builtin_popcountll(state.touched_sectors_CCW);
            }

            if (!(state.touched_sectors_CW & sector_bit))
            {
                state.touched_sectors_CW |= sector_bit;
                state.touched_count_CW = __builtin_popcountll(state.touched_sectors_CW);
            }
        }

        if (state.direction == Direction::COUNTER_CLOCKWISE)
        {
            if (state.touched_sectors_CW & sector_bit)
            {
                state.touched_sectors_CW &= ~sector_bit;
                state.touched_count_CW = __builtin_popcountll(state.touched_sectors_CW);
            }

            if (!(state.touched_sectors_CCW & sector_bit))
            {
                state.touched_sectors_CCW |= sector_bit;
                state.touched_count_CCW = __builtin_popcountll(state.touched_sectors_CCW);
            }
        }

        // Update direction based on sector sequence
        if (state.touched_count_CW > 0)  // Need at least 2 sectors for direction
        {
        }

        // Check for full rotation
        if (checkFullRotation())
        {
            state.laps += (state.direction == Direction::CLOCKWISE) ? 1 : -1;

            // Reset sector tracking for next rotation
            state.touched_sectors_CW  = 0;
            state.touched_count_CW    = 0;
            state.touched_sectors_CCW = 0;
            state.touched_count_CCW   = 0;
        }

        Serial.print(F("current pulse:  \t"));
        Serial.println(state.current_pulse);
        Serial.print(F("new sector:     \t"));
        Serial.println(new_sector);
        Serial.print(F("current sector: \t"));
        Serial.println(state.current_sector);
        Serial.print(F("sector bit:     \t"));
        Serial.println(sector_bit);
        Serial.print(F("touched sectors:\t"));
        Serial.println(state.touched_sectors_CW);
        Serial.print(F("touched count CW:\t"));
        Serial.println(state.touched_count_CW);
        Serial.print(F("touched sectors:\t"));
        Serial.println(state.touched_sectors_CCW);
        Serial.print(F("touched count CCW\t"));
        Serial.println(state.touched_count_CCW);
        Serial.print(F("sector diff:    \t"));
        Serial.println(sector_diff);
        Serial.print(F("direction:      \t"));
        Serial.println((int)state.direction);
        Serial.print(F("laps:           \t"));
        Serial.println(state.laps);
        Serial.println();
        Serial.println();
    }
}

bool MAE3Encoder::checkFullRotation() const
{
    // Check if the 3 highest sectors are touched
    uint64_t highest_sectors = (1ULL << (EncoderState::NUM_SECTORS - 1)) | (1ULL << (EncoderState::NUM_SECTORS - 64));
    //  | (1ULL << (EncoderState::NUM_SECTORS - 3));

    return (state.touched_sectors_CW & highest_sectors) == highest_sectors;
}

void MAE3Encoder::updateDirectionAndLaps()
{
    return;
}

void MAE3Encoder::reset()
{
    state.current_pulse     = 0;
    state.last_pulse        = 0;
    state.width_high        = 0;
    state.width_low         = 0;
    state.period            = 0;
    state.laps              = 0;
    state.absolute_position = 0;
    state.direction         = Direction::UNKNOWN;
    state.last_pulse        = 0.0f;
    state.delta             = 0.0f;
    newPulseAvailable       = false;
    lastPulseTime           = 0;
    lastFallingEdgeTime     = 0;
    lastRisingEdgeTime      = 0;

    // Reset sector tracking
    state.touched_sectors_CW = 0;
    state.current_sector     = 0;
    state.last_sector        = 0;
    state.touched_count_CW   = 0;
}

int64_t MAE3Encoder::getMedianWidthHigh() const
{
    std::array<int64_t, PULSE_BUFFER_SIZE> temp = widthHBuffer;
    std::sort(temp.begin(), temp.end());
    return temp[PULSE_BUFFER_SIZE / 2];
}

int64_t MAE3Encoder::getMedianWidthLow() const
{
    std::array<int64_t, PULSE_BUFFER_SIZE> temp = widthLBuffer;
    std::sort(temp.begin(), temp.end());
    return temp[PULSE_BUFFER_SIZE / 2];
}
