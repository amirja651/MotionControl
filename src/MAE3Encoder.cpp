#include "MAE3Encoder.h"
#include <algorithm>
#include <bitset>
#include <esp_timer.h>

// For pins above 31 (e.g. GPIO32 to GPIO39), GPIO.in1.data is used
#define READ_FAST(pin) ((pin < 32) ? ((GPIO.in >> pin) & 0x1) : ((GPIO.in1.data >> (pin - 32)) & 0x1))

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
      newPulseAvailable(false),
      bufferUpdated(false),
      width_l_buffer{},
      width_h_buffer{},
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

    // Encoder starts disabled by default
    enabled = false;

    return true;
}

void MAE3Encoder::enable()
{
    if (enabled)
        return;

    enabled = true;
    attachInterruptHandler();
}

void MAE3Encoder::disable()
{
    if (!enabled)
        return;

    enabled = false;
    detachInterruptHandler();
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

    int64_t currentTime = esp_timer_get_time();
    portENTER_CRITICAL_ISR(&mux);

    if (READ_FAST(signalPin))
    {
        // Rising edge
        lastRisingEdgeTime = currentTime;

        if (lastFallingEdgeTime != 0)
        {
            int64_t pulse_width = lastRisingEdgeTime - lastFallingEdgeTime;

            if (pulse_width < 1 || pulse_width > 4097)
            {
                portEXIT_CRITICAL_ISR(&mux);
                return;
            }

            width_l_buffer[pulseBufferIndex] = pulse_width;
        }
    }
    else
    {
        // Falling edge
        lastFallingEdgeTime = currentTime;

        if (lastRisingEdgeTime != 0)
        {
            int64_t pulse_width = lastFallingEdgeTime - lastRisingEdgeTime;

            if (pulse_width < 1 || pulse_width > 4097)
            {
                portEXIT_CRITICAL_ISR(&mux);
                return;
            }

            width_h_buffer[pulseBufferIndex] = pulse_width;
        }

        int64_t period = width_h_buffer[pulseBufferIndex] + width_l_buffer[pulseBufferIndex];
        if (period < 1 || period > 4098)
        {
            portEXIT_CRITICAL_ISR(&mux);
            return;
        }

        state.width_high = width_h_buffer[pulseBufferIndex];
        state.width_low  = width_l_buffer[pulseBufferIndex];

        pulseBufferIndex = (pulseBufferIndex + 1) % PULSE_BUFFER_SIZE;
        bufferUpdated    = true;
    }

    portEXIT_CRITICAL_ISR(&mux);
}

// New method for processing PWM signal
void MAE3Encoder::processPWM()
{
    bool updated;

    portENTER_CRITICAL(&mux);
    updated = bufferUpdated;
    portEXIT_CRITICAL(&mux);

    if (!enabled || !updated)
        return;

    int64_t width_h = get_median_width_high();
    int64_t width_l = get_median_width_low();
    int64_t period  = width_h + width_l;
    state.period    = period;

    if (period == 0)
        return;

    // Optimized calculation for x_measured
    int64_t x_measured = ((width_h * 4098) / period) - 1;

    // Validate based on documentation
    if (x_measured > 4096)
        return;
    state.current_pulse = (x_measured >= 4095) ? 4095 : x_measured;

    newPulseAvailable = true;
    portENTER_CRITICAL(&mux);
    bufferUpdated = false;
    portEXIT_CRITICAL(&mux);
    lastPulseTime = esp_timer_get_time();

    // در انتهای processPWM:
    int64_t delta_pulse = state.current_pulse - state.last_pulse;

    if (delta_pulse > 2048)
        delta_pulse -= 4096;
    else if (delta_pulse < -2048)
        delta_pulse += 4096;

    // Direction update بلادرنگ
    if (delta_pulse > 0)
        state.direction = Direction::CLOCKWISE;
    else if (delta_pulse < 0)
        state.direction = Direction::COUNTER_CLOCKWISE;

    state.delta = delta_pulse;
    state.accumulated_steps += delta_pulse;

    // Laps update دقیق
    if (state.accumulated_steps >= 4096)
    {
        state.laps += 1;
        state.accumulated_steps -= 4096;
    }
    else if (state.accumulated_steps <= -4096)
    {
        state.laps -= 1;
        state.accumulated_steps += 4096;
    }

    state.last_pulse = state.current_pulse;

    // Update sector tracking
    updateSectorTracking();
}

void MAE3Encoder::updateSectorTracking()
{
    uint16_t new_sector = state.current_pulse / EncoderState::STEPS_PER_SECTOR;

    // Always compute sector_diff
    int16_t sector_diff = new_sector - state.current_sector;

    if (sector_diff > EncoderState::NUM_SECTORS / 2)
        sector_diff -= EncoderState::NUM_SECTORS;
    else if (sector_diff < -EncoderState::NUM_SECTORS / 2)
        sector_diff += EncoderState::NUM_SECTORS;

    // Update direction always
    if (sector_diff > 0)
        state.direction = Direction::CLOCKWISE;
    else if (sector_diff < 0)
        state.direction = Direction::COUNTER_CLOCKWISE;
    // if sector_diff == 0 → keep previous direction

    // Now update sectors tracking only if sector actually changed
    if (new_sector != state.current_sector)
    {
        state.last_sector    = state.current_sector;
        state.current_sector = new_sector;

        // If sector was untouched
        if (!state.touched_sectors.test(new_sector))
        {
            state.touched_sectors.set(new_sector);
            state.touched_count++;
        }

        // Check full rotation
        if (checkFullRotation())
        {
            state.laps += (state.direction == Direction::CLOCKWISE) ? 1 : -1;
            state.touched_sectors.reset();
            state.touched_count = 0;
        }
    }
}

bool MAE3Encoder::checkFullRotation() const
{
    constexpr float FULL_ROTATION_THRESHOLD = 0.95f;  // 95% of sectors touched

    float touched_ratio = static_cast<float>(state.touched_count) / static_cast<float>(EncoderState::NUM_SECTORS);

    return touched_ratio >= FULL_ROTATION_THRESHOLD;
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
    state.touched_sectors = 0;
    state.current_sector  = 0;
    state.last_sector     = 0;
    state.touched_count   = 0;
}

int64_t MAE3Encoder::get_median_width_high() const
{
    std::array<int64_t, PULSE_BUFFER_SIZE> temp;
    portENTER_CRITICAL(&mux);
    temp = width_h_buffer;
    portEXIT_CRITICAL(&mux);
    std::sort(temp.begin(), temp.end());
    return temp[PULSE_BUFFER_SIZE / 2];
}

int64_t MAE3Encoder::get_median_width_low() const
{
    std::array<int64_t, PULSE_BUFFER_SIZE> temp;
    portENTER_CRITICAL(&mux);
    temp = width_l_buffer;
    portEXIT_CRITICAL(&mux);
    std::sort(temp.begin(), temp.end());
    return temp[PULSE_BUFFER_SIZE / 2];
}
