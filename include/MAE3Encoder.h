#ifndef MAE3_ENCODER2_H
#define MAE3_ENCODER2_H

#include <Arduino.h>
#include <array>
#include <bitset>
#include <cstdlib>  // For abs with uint32_t
#include <esp_timer.h>

// Maximum number of encoders supported
constexpr uint8_t MAX_ENCODERS = 4;

// Linear motion constants
constexpr float LEAD_SCREW_PITCH_MM = 0.2f;  // Lead screw pitch in mm

// Direction enum
enum class Direction
{
    CLOCKWISE,
    COUNTER_CLOCKWISE,
    UNKNOWN
};

struct EncoderState
{
    volatile int64_t current_pulse;  // Current pulse value
    volatile int64_t last_pulse;     // Current pulse value
    volatile int64_t width_high;     // High pulse width (rising to falling)
    volatile int64_t width_low;      // Low pulse width (falling to rising)
    volatile int64_t period;         // Total pulse width (t_on + t_off)
    volatile int64_t laps;           // Number of complete rotations

    Direction direction = Direction::UNKNOWN;  // Current direction of rotation
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

    bool isEnabled() const
    {
        return enabled;
    }

    bool isDisabled() const
    {
        return !enabled;
    }

    const EncoderState& getState() const
    {
        return state;
    }

    // Degrees per pulse
    float getDegreesPerPulse() const
    {
        return 360.0f / 4096;
    }

    // Millimeters per pulse
    float getMMPerPulse() const
    {
        return LEAD_SCREW_PITCH_MM / 4096;
    }

    // Micrometers per pulse
    float getUMPerPulse() const
    {
        return getMMPerPulse() * 1000.0f;
    }

    // Position in degrees
    float getPositionDegrees() const
    {
        return state.current_pulse * getDegreesPerPulse();
    }

    // Position in mm
    float getPositionMM() const
    {
        return state.current_pulse * getMMPerPulse();
    }

    // Position in μm
    float getPositionUM() const
    {
        return getPositionMM() * 1000.0f;
    }

    // Total travel in mm
    float getTotalTravelMM() const
    {
        float totalDistance = (state.laps * LEAD_SCREW_PITCH_MM) + getPositionMM();
        return totalDistance;
    }

    // Total travel in μm
    float getTotalTravelUM() const
    {
        return getTotalTravelMM() * 1000.0f;
    }

private:
    // Pin assignments
    const uint8_t signalPin;
    const uint8_t encoderId;

    // State management
    EncoderState  state;
    volatile bool enabled = false;

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
    static constexpr size_t PULSE_BUFFER_SIZE = 3;

    std::array<int64_t, PULSE_BUFFER_SIZE> width_l_buffer{};
    std::array<int64_t, PULSE_BUFFER_SIZE> width_h_buffer{};

    size_t pulseBufferIndex = 0;

    mutable portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

    void processInterrupt();
    void attachInterruptHandler();
    void detachInterruptHandler();

    int64_t get_median_width_high() const;
    int64_t get_median_width_low() const;

    static void IRAM_ATTR interruptHandler0();
    static void IRAM_ATTR interruptHandler1();
    static void IRAM_ATTR interruptHandler2();
    static void IRAM_ATTR interruptHandler3();
};

#endif  // MAE3_ENCODER2_H