#ifndef MAE3_ENCODER2_H
#define MAE3_ENCODER2_H

#include <Arduino.h>
#include <array>
#include <cstdlib>  // For abs with uint32_t

// Maximum number of encoders supported
constexpr uint8_t MAX_ENCODERS = 4;

// Constants for MAE3-P10-157-220-12-1
constexpr uint32_t MIN_PULSE_WIDTH   = 1;     // Minimum valid pulse width in microseconds
constexpr uint32_t MAX_PULSE_WIDTH   = 4098;  // Maximum valid pulse width in microseconds
constexpr uint32_t PULSE_PERIOD      = 4098;  // Total period in microseconds
constexpr uint32_t PULSE_PER_REV     = 4096;  // 12-bit resolution (2^12)
constexpr float    DEGREES_PER_PULSE = 360.0f / PULSE_PER_REV;

// Error codes
enum class EncoderError
{
    NONE,
    INVALID_PULSE_WIDTH,
    SIGNAL_LOST,
    NOISE_DETECTED,
    COMMUNICATION_ERROR
};

// Direction enum
enum class Direction
{
    CLOCKWISE,
    COUNTER_CLOCKWISE,
    UNKNOWN
};

struct EncoderState
{
    uint32_t      currentPulse;    // Current pulse value (0-4095)
    uint32_t      totalPulses;     // Total number of pulses since start
    int32_t       laps;            // Number of complete rotations
    Direction     direction;       // Current direction of rotation
    EncoderError  error;           // Current error state
    uint32_t      lastValidPulse;  // Last valid pulse value
    unsigned long lastUpdateTime;  // Timestamp of last update
};

class MAE3Encoder2
{
public:
    /**
     * @brief Constructor for MAE3Encoder2
     * @param signalPin GPIO pin connected to encoder PWM output
     * @param interruptPin GPIO pin for interrupt
     * @param encoderId Unique identifier for this encoder (0-3)
     */
    MAE3Encoder2(uint8_t signalPin, uint8_t interruptPin, uint8_t encoderId);

    /**
     * @brief Initialize the encoder
     * @return true if initialization successful, false otherwise
     */
    bool begin();

    /**
     * @brief Update encoder state and process new measurements
     * @return true if update successful, false if error occurred
     */
    bool update();

    /**
     * @brief Get current encoder state
     * @return Current encoder state
     */
    const EncoderState& getState() const
    {
        return state;
    }

    /**
     * @brief Get current position in degrees (0-360)
     * @return Current position in degrees
     */
    float getPositionDegrees() const
    {
        return state.currentPulse * DEGREES_PER_PULSE;
    }

    /**
     * @brief Get total rotation in degrees (including multiple laps)
     * @return Total rotation in degrees
     */
    float getTotalRotationDegrees() const
    {
        return (state.laps * 360.0f) + getPositionDegrees();
    }

    /**
     * @brief Get current velocity in degrees per second
     * @return Current velocity in degrees per second
     */
    float getVelocityDPS() const
    {
        return velocityDPS;
    }

    /**
     * @brief Get current error state
     * @return Current error code
     */
    EncoderError getError() const
    {
        return state.error;
    }

    /**
     * @brief Reset encoder state
     */
    void reset();

    /**
     * @brief Process interrupt for this encoder instance
     */
    void processInterrupt();

private:
    // Median filter implementation for noise reduction
    uint32_t medianFilter();

    // Direction detection
    Direction detectDirection(uint32_t newPulse);

    // Error detection and handling
    bool validatePulse(uint32_t pulse);
    void handleError(EncoderError error);

    // Performance optimization
    void optimizeInterrupt();

    // Pin assignments
    const uint8_t signalPin;
    const uint8_t interruptPin;
    const uint8_t encoderId;

    // State management
    EncoderState state;
    float        velocityDPS;

    // Filtering and timing
    std::array<uint32_t, 5> pulseFilter;  // Median filter buffer
    uint8_t                 filterIndex;
    unsigned long           lastPulseTime;
    unsigned long           lastVelocityUpdate;

    // Performance monitoring
    uint32_t errorCount;
    uint32_t validPulseCount;

    // Interrupt handling
    volatile uint32_t      currentPulseWidth;
    volatile bool          newPulseAvailable;
    volatile unsigned long pulseStartTime;
};

#endif  // MAE3_ENCODER2_H