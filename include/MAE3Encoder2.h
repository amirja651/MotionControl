#ifndef MAE3_ENCODER2_H
#define MAE3_ENCODER2_H

#include "Constants.h"
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
    float    PWM_FREQUENCY;    // PWM frequency in Hz
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
constexpr double LEAD_SCREW_PITCH_MM = 0.2f;  // Lead screw pitch in mm
constexpr double TOTAL_TRAVEL_MM     = 5.0f;  // Total travel distance in mm

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
                 EncoderResolution resolution = EncoderResolution::BITS_12);

    bool begin();

    bool update();

    const EncoderState& getState() const
    {
        return state;
    }

    // Degrees per pulse
    float getDegreesPerPulse() const
    {
        return 360.0f / constants.PULSE_PER_REV;
    }

    // Millimeters per pulse
    double getMillimetersPerPulse() const
    {
        return LEAD_SCREW_PITCH_MM / constants.PULSE_PER_REV;
    }

    // Pulses per revolution
    uint32_t getPulsesPerRevolution() const
    {
        return state.currentPulse;
    }

    // Position in degrees
    float getPositionDeg() const
    {
        return state.currentPulse * getDegreesPerPulse();
    }

    // Position in mm
    double getPositionMm() const
    {
        return state.currentPulse * getMillimetersPerPulse();
    }

    // Total travel in mm
    double getTotalTravelMm() const
    {
        double totalDistance = (state.laps * LEAD_SCREW_PITCH_MM) + getPositionMm();
        return std::min(totalDistance, TOTAL_TRAVEL_MM);
    }

    void reset();
    void processInterrupt();
    void setLowerLimits(double lowerLimitMm);
    void setUpperLimits(double upperLimitMm);

    double getLowerLimits() const
    {
        return lowerLimitMm;
    }

    double getUpperLimits() const
    {
        return upperLimitMm;
    }

    void calculateLaps(double currentPositionMm);

    // Add error state tracking
    struct ErrorState
    {
        uint32_t interrupt_errors;
        uint32_t invalid_pulses;
        uint32_t direction_errors;
    } error_state;

    // Add error checking method
    bool hasErrors() const
    {
        return error_state.interrupt_errors > 0 || error_state.invalid_pulses > 0 || error_state.direction_errors > 0;
    }

    // Add error reporting method
    void reportErrors()
    {
        if (hasErrors())
        {
            Serial.print(F("Encoder "));
            Serial.print(encoderId);
            Serial.print(F(" Errors - Interrupt: "));
            Serial.print(error_state.interrupt_errors);
            Serial.print(F(" Invalid Pulses: "));
            Serial.print(error_state.invalid_pulses);
            Serial.print(F(" Direction: "));
            Serial.println(error_state.direction_errors);
        }
    }

private:
    // Direction detection
    Direction detectDirection();

    // Pin assignments
    const uint8_t signalPin;
    const uint8_t interruptPin;
    const uint8_t encoderId;

    // Limits
    double lowerLimitMm;
    double upperLimitMm;

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

    // Individual interrupt handler for this encoder
    static void IRAM_ATTR interruptHandler0();
    static void IRAM_ATTR interruptHandler1();
    static void IRAM_ATTR interruptHandler2();
    static void IRAM_ATTR interruptHandler3();

    // Static array to store encoder instances for interrupt handling
    static MAE3Encoder2* encoderInstances[4];
};

#endif  // MAE3_ENCODER2_H