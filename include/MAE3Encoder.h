#ifndef MAE3_ENCODER2_H
#define MAE3_ENCODER2_H

#include <Arduino.h>
#include <array>
#include <bitset>
#include <cstdlib>  // For abs with uint32_t
#include <esp_timer.h>
#include <functional>  // For callback support

// #define DEBUG_ENCODER true

// Maximum number of encoders supported
const uint8_t MAX_ENCODERS = 4;
const int8_t  MAX_LAPS     = 20;
const int8_t  LAPS_OFFSET  = 10;

const int64_t DIR_THRESHOLD       = 2;     // For example, if the difference is more than 2 pulses → change direction
const int64_t FULL_SCALE          = 4096;  // 0..4095
const int64_t HIGH_WRAP_THRESHOLD = 1000;
const int64_t LOW_WRAP_THRESHOLD  = -1000;

// Linear motion constants
const float LEAD_SCREW_PITCH_MM = 0.2f;  // Lead screw pitch in mm

// Direction enum
enum class Direction
{
    CLOCKWISE,
    COUNTER_CLOCKWISE,
    UNKNOWN
};

struct EncoderState
{
    volatile int32_t current_pulse;  // Current pulse value
    volatile int64_t width_high;     // High pulse width (rising to falling)
    volatile int64_t width_low;      // Low pulse width (falling to rising)
    volatile int32_t laps;           // Number of complete rotations
    Direction        direction;      // Current direction of rotation
};

struct RPulse
{
    volatile int64_t low  = 0;
    volatile int64_t high = 0;
};

class MAE3Encoder
{
public:
    MAE3Encoder(uint8_t signalPin, uint8_t encoderId);
    ~MAE3Encoder();

    bool begin();
    void enable();
    void disable();
    void reset();
    void processPWM();

    inline bool isEnabled() const
    {
        return enabled;
    }

    inline bool isDisabled() const
    {
        return !enabled;
    }

    inline const EncoderState& getState() const
    {
        return state;
    }

    // Degrees per pulse
    inline float getDegreesPerPulse() const
    {
        float p = getAveragePeriod(encoderId, state.laps);
        return 360.0f / p;
    }

    // Millimeters per pulse
    inline float getMMPerPulse() const
    {
        float p = getAveragePeriod(encoderId, state.laps);
        return LEAD_SCREW_PITCH_MM / p;
    }

    // Micrometers per pulse
    inline float getUMPerPulse() const
    {
        return getMMPerPulse() * 1000.0f;
    }

    // Position in degrees
    inline float getPositionDegrees() const
    {
        return state.current_pulse * getDegreesPerPulse();
    }

    // Position in mm
    inline float getPositionMM() const
    {
        return state.current_pulse * getMMPerPulse();
    }

    // Position in μm
    inline float getPositionUM() const
    {
        return getPositionMM() * 1000.0f;
    }

    // Total travel in mm
    float getTotalTravelMM() const
    {
        float totalDistance = (state.laps * LEAD_SCREW_PITCH_MM) + getPositionMM();
        return totalDistance;
    }

    float getTotalTravelMM_precise() const
    {
        float totalDistance = 0.0f;

        int32_t lap_start = (state.laps >= 0) ? -LAPS_OFFSET : state.laps;
        int32_t lap_end   = (state.laps >= 0) ? state.laps : -LAPS_OFFSET;
        int32_t lap_step  = (state.laps >= 0) ? 1 : -1;

        // Sum travel in full laps
        for (int32_t i = lap_start; i != lap_end; i += lap_step)
        {
            float avgPeriod = getAveragePeriod(encoderId, i);
            if (avgPeriod == 0)
                continue;
            float mmPerPulse = LEAD_SCREW_PITCH_MM / avgPeriod;
            // Multiply the direction of movement:
            totalDistance += lap_step * FULL_SCALE * mmPerPulse;
        }

        float avgPeriodCurrent = getAveragePeriod(encoderId, state.laps);
        if (avgPeriodCurrent > 0)
        {
            float mmPerPulse = LEAD_SCREW_PITCH_MM / avgPeriodCurrent;
            totalDistance += ((state.laps >= 0) ? 1.0f : -1.0f) * state.current_pulse * mmPerPulse;
        }

        return totalDistance;
    }

    float getTotalTravelUM_precise() const
    {
        return getTotalTravelMM_precise() * 1000.0f;
    }

    // Total travel in μm
    float getTotalTravelUM() const
    {
        return getTotalTravelMM() * 1000.0f;
    }

    /**
     * @brief Check if the encoder is considered stopped (no pulses for a while)
     * @param threshold_us Time in microseconds to consider as stopped
     */
    bool isStopped(int64_t threshold_us = 500000 /* 500ms */) const
    {
        return (esp_timer_get_time() - lastPulseTime) > threshold_us;
    }

    /**
     * @brief Set callback function to notify when new pulse data is available
     * @param cb Function to call with EncoderState reference
     */
    void setOnPulseUpdatedCallback(std::function<void(const EncoderState&)> cb)
    {
        onPulseUpdated = cb;
    }

    inline int64_t get_period(uint8_t encoderID, int32_t lapIndex) const
    {
        return period[encoderID][lapIndex + LAPS_OFFSET];
    }

    inline float getAveragePeriod(uint8_t encoderID, int32_t lapIndex) const
    {
        int64_t sum   = period_sum[encoderID][lapIndex + LAPS_OFFSET];
        int64_t count = period_count[encoderID][lapIndex + LAPS_OFFSET];
        if (count == 0)
            return 0;
        return static_cast<float>(sum) / count;
    }

private:
    // Pin assignments
    const uint8_t signalPin;
    const uint8_t encoderId;

    // State management
    EncoderState  state;
    volatile bool enabled = false;

    RPulse r_pulse;

    // Filtering and timing
    int64_t          lastPulseTime       = 0;
    volatile int64_t lastFallingEdgeTime = 0;
    volatile int64_t lastRisingEdgeTime  = 0;

    // Interrupt handling
    volatile bool newPulseAvailable = false;

    // Flag to indicate buffer was updated (set in processInterrupt, cleared after processPWM)
    volatile bool bufferUpdated = false;

    // Static array to store encoder instances for interrupt handling
    static MAE3Encoder* encoderInstances[MAX_ENCODERS];

    // --- Pulse width ring buffers ---
    static constexpr size_t PULSE_BUFFER_SIZE = 5;

    std::array<int64_t, PULSE_BUFFER_SIZE> width_l_buffer{};
    std::array<int64_t, PULSE_BUFFER_SIZE> width_h_buffer{};

    size_t pulseBufferIndex = 0;

    mutable portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

    std::function<void(const EncoderState&)> onPulseUpdated;  // NEW: callback support

    volatile float    period[MAX_ENCODERS][MAX_LAPS]       = {0};
    volatile int64_t  period_sum[MAX_ENCODERS][MAX_LAPS]   = {0};
    volatile uint32_t period_count[MAX_ENCODERS][MAX_LAPS] = {0};

    int32_t last_pulse;
    bool    initialized;

    void processInterrupt();
    void attachInterruptHandler();
    void detachInterruptHandler();

    int64_t get_median_width_high() const;
    int64_t get_median_width_low() const;

    static void IRAM_ATTR interruptHandler0();
    static void IRAM_ATTR interruptHandler1();
    static void IRAM_ATTR interruptHandler2();
    static void IRAM_ATTR interruptHandler3();

    inline void setPeriod(uint8_t encoderID, int32_t lapIndex, int64_t _period)
    {
        period[encoderID][lapIndex + LAPS_OFFSET] = _period;
    }

    inline float getPeriod(uint8_t encoderID, int32_t lapIndex) const
    {
        return static_cast<float>(period[encoderID][lapIndex + LAPS_OFFSET]);
    }
};

#endif  // MAE3_ENCODER2_H