#ifndef MAE3_ENCODER2_H
#define MAE3_ENCODER2_H

#include <Arduino.h>
#include <array>
#include <cstdlib>  // For abs with uint32_t
#include <esp_timer.h>

// Maximum number of encoders supported
constexpr uint8_t MAX_ENCODERS            = 4;
constexpr uint8_t CALIBRATION_BUFFER_SIZE = 20;

// Linear motion constants
constexpr float    LEAD_SCREW_PITCH_MM = 0.5f;      // Lead screw pitch in mm
constexpr float    TOTAL_TRAVEL_MM     = 30.0f;     // Total travel distance in mm
constexpr float    LEAD_SCREW_PITCH_UM = 500.0f;    // 0.5mm = 500μm
constexpr float    TOTAL_TRAVEL_UM     = 30000.0f;  // 30mm = 30000μm
constexpr uint32_t DEFAULT_MAX_T       = 4096;      // Default max_t value

// Direction enum
enum class Direction
{
    CLOCKWISE,
    COUNTER_CLOCKWISE,
    UNKNOWN
};

struct EncoderState
{
    volatile uint64_t current_Pulse;  // Current pulse value
    int32_t           laps;           // Number of complete rotations
    Direction         direction;      // Current direction of rotation
    volatile uint64_t last_pulse;     // Last valid pulse value
    volatile uint64_t width_high;     // High pulse width (rising to falling)
    volatile uint64_t width_low;      // Low pulse width (falling to rising)
    volatile uint64_t period;         // Total pulse width (t_on + t_off)
};

class MAE3Encoder
{
public:
    MAE3Encoder(uint8_t signalPin, uint8_t encoderId);
    ~MAE3Encoder();

    bool begin();
    void enable();
    void disable();
    bool isEnabled() const
    {
        return enabled;
    }
    bool calibrate();  // New calibration method

    void update();

    const EncoderState& getState() const
    {
        return state;
    }
    uint32_t getMaxT() const
    {
        return max_t;
    }  // Get current max_t value

    // Degrees per pulse
    float getDegreesPerPulse() const
    {
        return 360.0f / max_t;
    }

    // Millimeters per pulse
    float getMMPerPulse() const
    {
        return LEAD_SCREW_PITCH_MM / max_t;
    }

    // Micrometers per pulse
    float getUMPerPulse() const
    {
        return getMMPerPulse() * 1000.0f;
    }

    // Position in degrees
    float getPositionDegrees() const
    {
        return state.current_Pulse * getDegreesPerPulse();
    }

    // Position in mm
    float getPositionMM() const
    {
        return state.current_Pulse * getMMPerPulse();
    }

    // Position in μm
    float getPositionUM() const
    {
        return state.current_Pulse * getUMPerPulse();
    }

    // Total travel in mm
    float getTotalTravelMM() const
    {
        float totalDistance = (state.laps * LEAD_SCREW_PITCH_MM) + getPositionMM();
        return std::min(totalDistance, TOTAL_TRAVEL_MM);
    }

    // Total travel in μm
    float getTotalTravelUM() const
    {
        float totalDistanceUM = (state.laps * LEAD_SCREW_PITCH_UM) + getPositionUM();
        return std::min(totalDistanceUM, TOTAL_TRAVEL_UM);
    }

    void reset();

    // New method for processing PWM signal
    void processPWM();

    // --- Pulse width ring buffers ---
    static constexpr size_t                        PULSE_BUFFER_SIZE = 10;
    const std::array<uint64_t, PULSE_BUFFER_SIZE>& getWidthHBuffer() const
    {
        return widthHBuffer;
    }
    const std::array<uint64_t, PULSE_BUFFER_SIZE>& getWidthLBuffer() const
    {
        return widthLBuffer;
    }
    size_t getPulseBufferIndex() const
    {
        return pulseBufferIndex;
    }

    // Median filter for width_high
    uint64_t getMedianWidthHigh() const;
    // Median filter for width_low
    uint64_t getMedianWidthLow() const;

private:
    void      processInterrupt();
    uint32_t  medianFilter();
    Direction detectDirection();
    void      optimizeInterrupt();
    void      attachInterruptHandler();
    void      detachInterruptHandler();
    uint32_t  findMostFrequentValue(const std::array<uint32_t, CALIBRATION_BUFFER_SIZE>& buffer);

    // Pin assignments
    const uint8_t signalPin;
    const uint8_t encoderId;

    // State management
    EncoderState  state;
    volatile bool enabled;
    uint32_t      max_t;  // Instance-specific max_t value

    // Filtering and timing
    uint64_t          lastPulseTime;
    volatile uint64_t lastFallingEdgeTime;
    volatile uint64_t lastRisingEdgeTime;

    // Interrupt handling
    volatile bool newPulseAvailable;
    // Flag to indicate buffer was updated (set in processInterrupt, cleared after processPWM)
    volatile bool bufferUpdated;

    // Calibration buffer
    std::array<uint32_t, CALIBRATION_BUFFER_SIZE> calibrationBuffer;
    uint8_t                                       calibrationIndex;

    // Individual interrupt handler for this encoder
    static void IRAM_ATTR interruptHandler0();
    static void IRAM_ATTR interruptHandler1();
    static void IRAM_ATTR interruptHandler2();
    static void IRAM_ATTR interruptHandler3();

    // Static array to store encoder instances for interrupt handling
    static MAE3Encoder* encoderInstances[MAX_ENCODERS];

    // --- Pulse width ring buffers ---
    std::array<uint64_t, PULSE_BUFFER_SIZE> widthLBuffer{};
    std::array<uint64_t, PULSE_BUFFER_SIZE> widthHBuffer{};
    size_t                                  pulseBufferIndex = 0;
};

#endif  // MAE3_ENCODER2_H