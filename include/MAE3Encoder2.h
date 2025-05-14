#ifndef MAE3_ENCODER2_H
#define MAE3_ENCODER2_H

#include "Constants.h"
#include <Arduino.h>
#include <array>
#include <cstdlib>

// Maximum number of encoders supported
constexpr uint8_t MAX_ENCODERS = 4;

// Enhanced encoder resolution configuration
enum class EncoderResolution
{
    BITS_10 = 10,
    BITS_12 = 12
};

// Enhanced constants for both 10-bit and 12-bit versions
struct EncoderConstants
{
    uint32_t MIN_PULSE_WIDTH;  // Minimum valid pulse width in microseconds
    uint32_t MAX_PULSE_WIDTH;  // Maximum valid pulse width in microseconds
    uint32_t PULSE_PERIOD;     // Total period in microseconds
    uint32_t PULSE_PER_REV;    // Resolution (1024 for 10-bit, 4096 for 12-bit)
    float    PWM_FREQUENCY;    // PWM frequency in Hz
    uint32_t MIN_UPDATE_RATE;  // Minimum update rate in microseconds
    uint32_t MAX_UPDATE_RATE;  // Maximum update rate in microseconds
};

// Constants for 10-bit encoder (MAE3-P10)
constexpr EncoderConstants ENCODER_10BIT = {
    .MIN_PULSE_WIDTH = 1,       // 1 μs
    .MAX_PULSE_WIDTH = 1025,    // 1025 μs
    .PULSE_PERIOD    = 1026,    // Total period
    .PULSE_PER_REV   = 1024,    // 10-bit resolution (2^10)
    .PWM_FREQUENCY   = 0.975f,  // 975 Hz
    .MIN_UPDATE_RATE = 100,     // 100 μs minimum update rate
    .MAX_UPDATE_RATE = 1000     // 1000 μs maximum update rate
};

// Constants for 12-bit encoder (MAE3-P12)
constexpr EncoderConstants ENCODER_12BIT = {
    .MIN_PULSE_WIDTH = 1,       // 1 μs
    .MAX_PULSE_WIDTH = 4098,    // 4098 μs
    .PULSE_PERIOD    = 4098,    // Total period
    .PULSE_PER_REV   = 4096,    // 12-bit resolution (2^12)
    .PWM_FREQUENCY   = 0.244f,  // 244 Hz
    .MIN_UPDATE_RATE = 100,     // 100 μs minimum update rate
    .MAX_UPDATE_RATE = 1000     // 1000 μs maximum update rate
};

// Enhanced direction enum with additional states
enum class Direction
{
    CLOCKWISE,
    COUNTER_CLOCKWISE,
    UNKNOWN,
    STATIONARY
};

// Enhanced state structure with aligned members and additional metrics
struct alignas(8) EncoderState
{
    volatile uint32_t  currentPulse;    // Current pulse value
    volatile int32_t   laps;            // Number of complete rotations
    volatile Direction direction;       // Current direction of rotation
    volatile uint32_t  lastPulse;       // Last valid pulse value
    volatile uint32_t  pulseCount;      // Total pulse count since last reset
    volatile uint32_t  updateTime;      // update timestamp
    volatile uint32_t  lastUpdateTime;  // Last update timestamp
    volatile float     velocity;        // Current velocity in pulses/second
    volatile float     acceleration;    // Current acceleration in pulses/second²
    volatile int32_t   diff;            // Difference between current and last pulse
};

// Enhanced digital filter configuration
struct FilterConfig
{
    uint8_t  windowSize;             // Size of the moving average window
    bool     enabled;                // Whether the filter is enabled
    uint32_t noiseThreshold;         // Threshold for noise rejection
    uint32_t velocityThreshold;      // Threshold for velocity-based filtering
    uint32_t accelerationThreshold;  // Threshold for acceleration-based filtering
    bool     adaptiveFiltering;      // Whether to use adaptive filtering
};

// Enhanced error state tracking
struct ErrorState
{
    volatile uint32_t interrupt_errors;     // Count of interrupt-related errors
    volatile uint32_t invalid_pulses;       // Count of invalid pulse readings
    volatile uint32_t direction_errors;     // Count of direction detection errors
    volatile uint32_t filter_rejections;    // Count of pulses rejected by filter
    volatile uint32_t timing_errors;        // Count of timing-related errors
    volatile uint32_t velocity_errors;      // Count of velocity-related errors
    volatile uint32_t acceleration_errors;  // Count of acceleration-related errors
    volatile uint32_t overflow_errors;      // Count of counter overflow errors

    // Add assignment operator
    ErrorState& operator=(const ErrorState& other)
    {
        interrupt_errors    = other.interrupt_errors;
        invalid_pulses      = other.invalid_pulses;
        direction_errors    = other.direction_errors;
        filter_rejections   = other.filter_rejections;
        timing_errors       = other.timing_errors;
        velocity_errors     = other.velocity_errors;
        acceleration_errors = other.acceleration_errors;
        overflow_errors     = other.overflow_errors;
        return *this;
    }
};

class MAE3Encoder2
{
public:
    MAE3Encoder2(uint8_t signalPin, uint8_t interruptPin, uint8_t encoderId,
                 EncoderResolution resolution = EncoderResolution::BITS_12);

    bool begin();
    void update();
    void reset();
    void processInterrupt();

    // Enhanced getters using volatile references
    const volatile EncoderState& getState() const
    {
        return state;
    }
    const volatile ErrorState& getErrorState() const
    {
        return error_state;
    }
    // Optimized getters for critical values
    inline uint32_t getCurrentPulse() const
    {
        return state.currentPulse;
    }
    inline int64_t getTotalPulses() const
    {
        return (static_cast<int64_t>(state.laps) * constants.PULSE_PER_REV) + state.currentPulse;
    }
    inline int64_t getTotalLastPulses() const
    {
        return (static_cast<int64_t>(state.laps) * constants.PULSE_PER_REV) + state.lastPulse;
    }
    inline Direction getDirection() const
    {
        return state.direction;
    }
    inline int32_t getLaps() const
    {
        return state.laps;
    }
    inline float getVelocity() const
    {
        return state.velocity;
    }
    inline float getAcceleration() const
    {
        return state.acceleration;
    }

    // Enhanced filter configuration methods
    void         setFilterConfig(const FilterConfig& config);
    FilterConfig getFilterConfig() const;
    void         enableFilter(bool enable);
    void         setAdaptiveFiltering(bool enable);

    // Error handling methods
    bool hasErrors() const;
    void reportErrors();
    void clearErrors();

    // Performance monitoring methods
    uint32_t  getUpdateRate() const;
    float     getFilterEfficiency() const;
    uint32_t  getValidPulseCount() const;
    Direction detectDirection();

private:
    void updateVelocityAndAcceleration();

    // Enhanced filter methods
    void     resetFilter();
    uint32_t applyFilter(uint32_t newPulse);
    bool     isNoise(uint32_t newPulse) const;
    bool     isVelocityValid(float velocity) const;
    bool     isAccelerationValid(float acceleration) const;

    // Pin assignments
    const uint8_t signalPin;
    const uint8_t interruptPin;
    const uint8_t encoderId;

    // State management
    volatile EncoderState state;
    volatile ErrorState   error_state;

    // Enhanced filtering and timing
    volatile unsigned long lastPulseTime;
    FilterConfig           filterConfig;
    alignas(8) std::array<uint32_t, 32> pulseHistory;  // Increased buffer size for better filtering
    volatile uint8_t historyIndex;
    volatile uint8_t historyCount;

    // Performance monitoring
    volatile uint32_t validPulseCount;
    volatile uint32_t totalPulseCount;
    volatile uint32_t lastPerformanceCheck;

    // Interrupt handling
    volatile unsigned long pulseStartTime;
    volatile unsigned long lastInterruptTime;

    // Encoder configuration
    const EncoderResolution resolution;
    const EncoderConstants& constants;

    // Enhanced thresholds
    uint32_t HALF_REV;
    uint32_t NOISE_THRESHOLD;
    uint32_t DIRECTION_THRESHOLD;
    uint32_t QUARTER_REV;
    uint32_t THREE_QUARTER_REV;
    uint32_t VELOCITY_THRESHOLD;
    uint32_t ACCELERATION_THRESHOLD;

    // Helper method to get constants based on resolution
    static constexpr const EncoderConstants& getConstants(EncoderResolution res)
    {
        return (res == EncoderResolution::BITS_10) ? ENCODER_10BIT : ENCODER_12BIT;
    }

    // Individual interrupt handlers for each encoder
    static void IRAM_ATTR interruptHandler0();
    static void IRAM_ATTR interruptHandler1();
    static void IRAM_ATTR interruptHandler2();
    static void IRAM_ATTR interruptHandler3();

    // Static array to store encoder instances for interrupt handling
    static MAE3Encoder2* encoderInstances[MAX_ENCODERS];
};

#endif  // MAE3_ENCODER2_H