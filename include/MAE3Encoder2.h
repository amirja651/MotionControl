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

// Linear motion constants
constexpr float    LEAD_SCREW_PITCH = 0.5f;   // Lead screw pitch in mm
constexpr float    TOTAL_TRAVEL_MM  = 30.0f;  // Total travel distance in mm
constexpr uint8_t  MICROSTEPS       = 16;     // Microstepping configuration
constexpr uint16_t STEPS_PER_REV    = 200;    // Motor steps per revolution

// Calculate linear distance per pulse
constexpr float MM_PER_PULSE = LEAD_SCREW_PITCH / (PULSE_PER_REV * MICROSTEPS);
constexpr float UM_PER_PULSE = MM_PER_PULSE * 1000.0f;  // Convert to micrometers

// Direction enum
enum class Direction
{
    CLOCKWISE,
    COUNTER_CLOCKWISE,
    UNKNOWN
};

struct EncoderState
{
    uint32_t  currentPulse;  // Current pulse value (0-4095)
    int32_t   laps;          // Number of complete rotations
    Direction direction;     // Current direction of rotation
    uint32_t  lastPulse;     // Last valid pulse value
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
     * @brief Get current position in millimeters
     * @return Current position in millimeters
     */
    float getPositionMM() const
    {
        return state.currentPulse * MM_PER_PULSE;
    }

    /**
     * @brief Get current position in micrometers
     * @return Current position in micrometers
     */
    float getPositionUM() const
    {
        return state.currentPulse * UM_PER_PULSE;
    }

    /**
     * @brief Get total travel distance in millimeters (including multiple laps)
     * @return Total travel distance in millimeters
     */
    float getTotalTravelMM() const
    {
        float totalDistance = (state.laps * LEAD_SCREW_PITCH) + getPositionMM();
        // Ensure we don't exceed total travel
        return std::min(totalDistance, TOTAL_TRAVEL_MM);
    }

    /**
     * @brief Get total travel distance in micrometers (including multiple laps)
     * @return Total travel distance in micrometers
     */
    float getTotalTravelUM() const
    {
        return getTotalTravelMM() * 1000.0f;
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
    Direction detectDirection();

    // Performance optimization
    void optimizeInterrupt();

    // Pin assignments
    const uint8_t signalPin;
    const uint8_t interruptPin;
    const uint8_t encoderId;

    // State management
    EncoderState state;

    // Filtering and timing
    unsigned long lastPulseTime;

    // Interrupt handling
    volatile bool          newPulseAvailable;
    volatile unsigned long pulseStartTime;
};

#endif  // MAE3_ENCODER2_H