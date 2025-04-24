#ifndef MAE3_ENCODER2_H
#define MAE3_ENCODER2_H

#include <Arduino.h>
#include <array>
#include <cstdlib>  // For abs with uint32_t

// Maximum number of encoders supported
constexpr uint8_t MAX_ENCODERS = 4;

// Encoder resolution configuration
enum class EncoderResolution
{
    BITS_10 = 10,
    BITS_12 = 12
};

// Constants for both 10-bit and 12-bit versions
struct EncoderConstants
{
    uint32_t MIN_PULSE_WIDTH;  // Minimum valid pulse width in microseconds
    uint32_t MAX_PULSE_WIDTH;  // Maximum valid pulse width in microseconds
    uint32_t PULSE_PERIOD;     // Total period in microseconds
    uint32_t PULSE_PER_REV;    // Resolution (1024 for 10-bit, 4096 for 12-bit)
    float    PWM_FREQUENCY;    // PWM frequency in kHz
};

// Constants for 10-bit encoder (MAE3-P10)
constexpr EncoderConstants ENCODER_10BIT = {
    .MIN_PULSE_WIDTH = 1,      // 1 μs
    .MAX_PULSE_WIDTH = 1025,   // 1025 μs
    .PULSE_PERIOD    = 1026,   // Total period
    .PULSE_PER_REV   = 1024,   // 10-bit resolution (2^10)
    .PWM_FREQUENCY   = 0.975f  // 975 Hz
};

// Constants for 12-bit encoder (MAE3-P12)
constexpr EncoderConstants ENCODER_12BIT = {
    .MIN_PULSE_WIDTH = 1,      // 1 μs
    .MAX_PULSE_WIDTH = 4098,   // 4098 μs
    .PULSE_PERIOD    = 4098,   // Total period
    .PULSE_PER_REV   = 4096,   // 12-bit resolution (2^12)
    .PWM_FREQUENCY   = 0.244f  // 244 Hz
};

// Linear motion constants
constexpr float    LEAD_SCREW_PITCH = 0.5f;   // Lead screw pitch in mm
constexpr float    TOTAL_TRAVEL_MM  = 30.0f;  // Total travel distance in mm
constexpr uint8_t  MICROSTEPS       = 16;     // Microstepping configuration
constexpr uint16_t STEPS_PER_REV    = 200;    // Motor steps per revolution

// Calculate linear distance per pulse
constexpr float MM_PER_PULSE = LEAD_SCREW_PITCH / (ENCODER_10BIT.PULSE_PER_REV * MICROSTEPS);
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
    uint32_t  currentPulse;  // Current pulse value
    int32_t   laps;          // Number of complete rotations
    Direction direction;     // Current direction of rotation
    uint32_t  lastPulse;     // Last valid pulse value
};

class MAE3Encoder2
{
public:
    MAE3Encoder2(uint8_t signalPin, uint8_t interruptPin, uint8_t encoderId,
                 EncoderResolution resolution = EncoderResolution::BITS_10);

    bool begin();

    bool update();

    const EncoderState& getState() const
    {
        return state;
    }

    float getPositionDegrees() const
    {
        return state.currentPulse * getDegreesPerPulse();
    }

    float getPositionMM() const
    {
        return state.currentPulse * MM_PER_PULSE;
    }

    float getPositionUM() const
    {
        return state.currentPulse * UM_PER_PULSE;
    }

    float getTotalTravelMM() const
    {
        float totalDistance = (state.laps * LEAD_SCREW_PITCH) + getPositionMM();
        // Ensure we don't exceed total travel
        return std::min(totalDistance, TOTAL_TRAVEL_MM);
    }

    float getTotalTravelUM() const
    {
        return getTotalTravelMM() * 1000.0f;
    }

    void reset();

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

    // Encoder configuration
    const EncoderResolution resolution;
    const EncoderConstants& constants;

    // Helper method to get constants based on resolution
    static const EncoderConstants& getConstants(EncoderResolution res)
    {
        return (res == EncoderResolution::BITS_10) ? ENCODER_10BIT : ENCODER_12BIT;
    }

    // Calculate degrees per pulse based on resolution
    float getDegreesPerPulse() const
    {
        return 360.0f / constants.PULSE_PER_REV;
    }
};

#endif  // MAE3_ENCODER2_H