#include "MAE3Encoder.h"
#include <algorithm>
#include <esp_timer.h>
#include <map>
#include <stdint.h>

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
      lastPulseTime(0),
      lastFallingEdgeTime(0),
      lastRisingEdgeTime(0),
      bufferUpdated(false),
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
        return false;

    // Configure pins
    pinMode(signalPin, INPUT);

    // Store instance for interrupt handling
    encoderInstances[encoderId] = this;

    // Initialize state
    reset();

    // Encoder starts disabled by default
    enabled = false;

    return true;
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
            state.width_high = lastRisingEdgeTime - lastFallingEdgeTime;
            if (state.width_high > EncoderState::DEFAULT_MAX_T)
            {
                return;
            }
            // Measure low pulse width (falling to rising)
            widthLBuffer[pulseBufferIndex] = state.width_high;
        }
    }
    else
    {
        // Falling edge
        lastFallingEdgeTime = currentTime;
        if (lastRisingEdgeTime != 0)
        {
            state.width_low = lastFallingEdgeTime - lastRisingEdgeTime;
            if (state.width_low > EncoderState::DEFAULT_MAX_T)
            {
                return;
            }
            // Measure high pulse width (rising to falling)
            widthHBuffer[pulseBufferIndex] = state.width_low;
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
    state.period    = width_h + width_l;

    if (state.period == 0)
        return;

    // Optimized calculation for x_measured
    int64_t x_measured = (width_h * (EncoderState::DEFAULT_MAX_T + 2)) / state.period - 1;

    // Boundary check with early return
    if (x_measured <= (EncoderState::DEFAULT_MAX_T - 2))
    {
    }
    else if (x_measured == EncoderState::DEFAULT_MAX_T)
    {
        x_measured = EncoderState::DEFAULT_MAX_T - 1;
    }
    else
    {
        return;
    }

    state.current_pulse = x_measured;
    bufferUpdated       = false;
    lastPulseTime       = esp_timer_get_time();

    // Update sector tracking
    updateSectorTracking();
}

void MAE3Encoder::updateSectorTracking()
{
    static uint8_t last_sector  = 0;
    static bool    laps_update  = false;
    static int8_t  delta_sector = 0;

    // Calculate current sector (0-63)
    uint8_t new_sector = (state.current_pulse * EncoderState::NUM_SECTORS) / EncoderState::DEFAULT_MAX_T;

    if (new_sector != last_sector)
    {
        Serial.print(F("\ncurrent pulse:   \t"));
        Serial.println(state.current_pulse);
        Serial.print(F("new sector:      \t"));
        Serial.println(new_sector);

        delta_sector = new_sector - last_sector;

        Serial.print(F("delta_sector:    \t"));
        Serial.println(delta_sector);

        last_sector = new_sector;

        if (!laps_update && abs(delta_sector) > 10)
        {
            if (delta_sector > 0)
                state.laps++;
            else
                state.laps--;

            laps_update = true;
        }
    }

    if (abs(delta_sector) <= 1 || (state.current_sector > 0 && state.current_sector < 63))
    {
        laps_update = false;
    }

    // If sector changed
    if (new_sector == state.current_sector)
        return;

    Serial.print(F("laps:            \t"));
    Serial.println(state.laps);

    state.last_sector    = state.current_sector;
    state.current_sector = new_sector;

    uint64_t sector_bit  = 1ULL << state.current_sector;
    int8_t   sector_diff = 0;

    if (!(state.touched_sectors & sector_bit))
    {
        state.touched_sectors |= sector_bit;
        state.touched_count = __builtin_popcountll(state.touched_sectors);
    }

    int64_t raw_diff = state.current_sector - state.last_sector;
    sector_diff      = (raw_diff + EncoderState::NUM_SECTORS) % EncoderState::NUM_SECTORS;

    Serial.print(F("sector diff:     \t"));
    Serial.print(sector_diff);

    if (sector_diff > EncoderState::NUM_SECTORS / 2)
        sector_diff -= EncoderState::NUM_SECTORS;

    Serial.print(F("\t"));
    Serial.print(sector_diff);

    // Handle wraparound
    if (sector_diff > EncoderState::NUM_SECTORS / 2)
        sector_diff -= EncoderState::NUM_SECTORS;
    else if (sector_diff < -EncoderState::NUM_SECTORS / 2)
        sector_diff += EncoderState::NUM_SECTORS;

    Serial.print(F("\t"));
    Serial.println(sector_diff);

    state.direction = (sector_diff == 0)  ? Direction::UNKNOWN
                      : (sector_diff > 0) ? Direction::CLOCKWISE
                                          : Direction::COUNTER_CLOCKWISE;

    Serial.print(F("direction:       \t"));
    Serial.println((int64_t)state.direction);

    uint64_t highest_sectors = (1ULL << (EncoderState::NUM_SECTORS - 2)) | (1ULL << (EncoderState::NUM_SECTORS - 3));

    if ((state.touched_sectors & highest_sectors) == highest_sectors)
    {
        // Reset sector tracking for next rotation
        laps_update           = false;
        state.touched_sectors = 0;
        state.touched_count   = 0;
    }
}

void MAE3Encoder::printBinary64(uint64_t value)
{
    for (int i = 63; i >= 0; i--)
    {
        Serial.print((value >> i) & 1);
        if (i % 8 == 0)
            Serial.print(' ');
    }
    Serial.println();
}

void MAE3Encoder::reset()
{
    state.current_pulse = 0;
    state.last_pulse    = 0;
    state.width_high    = 0;
    state.width_low     = 0;
    state.period        = 0;
    state.laps          = 0;
    state.direction     = Direction::UNKNOWN;
    state.last_pulse    = 0.0f;
    lastPulseTime       = 0;
    lastFallingEdgeTime = 0;
    lastRisingEdgeTime  = 0;

    // Reset sector tracking
    state.touched_sectors = 0;
    state.current_sector  = 0;
    state.last_sector     = 0;
    state.touched_count   = 0;
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
