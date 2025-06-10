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
    Direction        direction;      // Current direction of rotation

    float avgPeriod;
    float avgPeriodCurrent;
    float totalDistance_1;
    float mmPerPulse_1;
    float totalDistance_2;
    float mmPerPulse_2;
};

struct LapState
{
    volatile int32_t id;
    float            period[MAX_LAPS]       = {0};
    int64_t          period_sum[MAX_LAPS]   = {0};
    uint32_t         period_count[MAX_LAPS] = {0};
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

    inline const EncoderState& gState() const
    {
        return state;
    }

    inline const LapState& gLap() const
    {
        return lap;
    }

    inline float getAveragePeriod(int32_t lapIndex) const
    {
        int64_t sum   = lap.period_sum[lapIndex + LAPS_OFFSET];
        int64_t count = lap.period_count[lapIndex + LAPS_OFFSET];
        if (count == 0)
            return FULL_SCALE;
        return static_cast<float>(sum) / count;
    }

    // Degrees per pulse
    inline float getDegreesPerPulse(int32_t lapIndex) const
    {
        float period = getAveragePeriod(lapIndex);
        return 360.0f / period;
    }

    // Position in degrees
    inline float getPositionDegrees(int32_t lapIndex) const
    {
        return state.current_pulse * getDegreesPerPulse(lapIndex);
    }

    // Millimeters per pulse
    inline float getMMPerPulse(int32_t lapIndex) const
    {
        float period = getAveragePeriod(lapIndex);
        return LEAD_SCREW_PITCH_MM / period;
    }

    // Position in mm
    inline float getPositionMM(int32_t lapIndex) const
    {
        return state.current_pulse * getMMPerPulse(lapIndex);
    }

    // Total travel in mm
    float getTotalTravelMM(int32_t lapIndex) const
    {
        float totalDistance = (lapIndex * LEAD_SCREW_PITCH_MM) + getPositionMM(lapIndex);
        return totalDistance;
    }

    // Total travel in μm
    float getTotalTravelUM(int32_t lapIndex) const
    {
        return getTotalTravelMM(lapIndex) * 1000.0f;
    }

    bool isStopped(int64_t threshold_us = 500000 /* 500ms */) const
    {
        return (esp_timer_get_time() - lastPulseTime) > threshold_us;
    }

    void setOnPulseUpdatedCallback(std::function<void(const EncoderState&)> cb)
    {
        onPulseUpdated = cb;
    }

    inline float getPeriod(int32_t lapIndex) const
    {
        return static_cast<float>(lap.period[lapIndex + LAPS_OFFSET]);
    }

private:
    // Pin assignments
    const uint8_t signalPin;
    const uint8_t encoderId;

    // State management
    EncoderState state;
    LapState     lap;

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

    inline void setPeriod(int32_t lapIndex, int64_t _period, bool reset_count = false)
    {
        lap.period[lapIndex + LAPS_OFFSET] = _period;

        if (reset_count)
        {
            lap.period_sum[lap.id + LAPS_OFFSET]   = _period;
            lap.period_count[lap.id + LAPS_OFFSET] = 1;
        }
        else
        {
            lap.period_sum[lap.id + LAPS_OFFSET] += _period;
            lap.period_count[lap.id + LAPS_OFFSET]++;
        }
    }

    void resetAllPeriods()
    {
        memset(lap.period, 0, sizeof(lap.period));
        memset(lap.period_sum, 0, sizeof(lap.period_sum));
        memset(lap.period_count, 0, sizeof(lap.period_count));
    }
};

#endif  // MAE3_ENCODER2_H