#ifndef MAE3_ENCODER2_H
#define MAE3_ENCODER2_H

#include <Arduino.h>
#include <array>
#include <cstdlib>  // For abs with uint32_t
#include <esp_timer.h>

// Direction enum
enum class Direction
{
    CLOCKWISE         = 1,
    UNKNOWN           = 0,
    COUNTER_CLOCKWISE = -1
};

// Maximum number of encoders supported
constexpr uint8_t MAX_ENCODERS = 4;

// Linear motion constants
constexpr float LEAD_SCREW_PITCH_MM = 0.2f;  // Lead screw pitch in mm (2  or 5)
constexpr float TOTAL_TRAVEL_MM     = 5.0f;  // Total travel distance in mm

struct EncoderState
{
    volatile int64_t current_pulse;  // Current pulse value
    volatile int64_t last_pulse;     // Current pulse value

    volatile int64_t width_high;  // High pulse width (rising to falling)
    volatile int64_t width_low;   // Low pulse width (falling to rising)

    volatile int64_t period;  // Total pulse width (t_on + t_off)
    volatile int64_t laps;    // Number of complete rotations
    bool             laps_update = true;

    Direction direction = Direction::UNKNOWN;  // Current direction of rotation

    // Sector tracking
    static constexpr int64_t DEFAULT_MAX_T    = 4096LL;  // Default 4096 value
    static constexpr int64_t NUM_SECTORS      = 128LL;
    static constexpr int64_t STEPS_PER_SECTOR = DEFAULT_MAX_T / NUM_SECTORS;

    int64_t current_sector = 0;  // Current sector (0-63)
    int64_t last_sector    = 0;  // Last sector for direction detection
    int8_t  delta_sector   = 0;
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

    const EncoderState& getState() const
    {
        return state;
    }

    // Degrees per pulse
    float get_degrees_per_pulse() const
    {
        return 360.0f / static_cast<float>(EncoderState::DEFAULT_MAX_T);
    }

    // Position in degrees
    float get_position_degrees() const
    {
        return state.current_pulse * get_degrees_per_pulse();
    }

    // Millimeters per pulse
    float get_mm_per_pulse() const
    {
        return LEAD_SCREW_PITCH_MM / static_cast<float>(EncoderState::DEFAULT_MAX_T);
    }

    // Micrometers per pulse
    float get_um_per_pulse() const
    {
        return get_mm_per_pulse() * 1000.0f;
    }

    // Position in mm
    float get_position_mm() const
    {
        return state.current_pulse * get_mm_per_pulse();
    }

    // Position in μm
    float get_position_um() const
    {
        return get_position_mm() * 1000.0f;
    }

    // Total travel in pulse
    float get_total_travel_pulse() const
    {
        float totalPulseCount = (static_cast<float>(state.laps) * static_cast<float>(EncoderState::DEFAULT_MAX_T)) +
                                static_cast<float>(state.current_pulse);
        return totalPulseCount;
    }

    // Total travel in mm
    float get_total_travel_mm() const
    {
        float totalPulseCount = get_total_travel_pulse();
        return totalPulseCount * get_mm_per_pulse();
    }

    // Total travel in μm
    float get_total_travel_um() const
    {
        return get_total_travel_mm() * 1000.0f;
    }

    void reset();

    // New method for processing PWM signal
    void processPWM();

    // Median filter for width_high
    int64_t getMedianWidthHigh() const;

    // Median filter for width_low
    int64_t getMedianWidthLow() const;

    // Sector tracking methods
    uint8_t getCurrentSector() const
    {
        return state.current_sector;
    }

private:
    void processInterrupt();
    void attachInterruptHandler();
    void detachInterruptHandler();

    // Pin assignments
    const uint8_t signalPin;
    const uint8_t encoderId;

    // State management
    EncoderState  state;
    volatile bool enabled;

    // Filtering and timing
    int64_t          lastPulseTime;
    volatile int64_t lastFallingEdgeTime;
    volatile int64_t lastRisingEdgeTime;

    // Flag to indicate buffer was updated (set in processInterrupt, cleared after processPWM)
    volatile bool bufferUpdated;

    // Individual interrupt handler for this encoder
    static void IRAM_ATTR interruptHandler0();
    static void IRAM_ATTR interruptHandler1();
    static void IRAM_ATTR interruptHandler2();
    static void IRAM_ATTR interruptHandler3();

    // Static array to store encoder instances for interrupt handling
    static MAE3Encoder* encoderInstances[MAX_ENCODERS];

    // --- Pulse width ring buffers ---
    static constexpr size_t                PULSE_BUFFER_SIZE = 10;
    std::array<int64_t, PULSE_BUFFER_SIZE> widthLBuffer{};
    std::array<int64_t, PULSE_BUFFER_SIZE> widthHBuffer{};
    size_t                                 pulseBufferIndex = 0;

    std::array<int8_t, 3> last_diffs{};
    uint8_t               diff_index = 0;

    uint8_t last_new_sector = 0;

    void updateSectorTracking();
    void printBinary64(uint64_t value);
};

#endif  // MAE3_ENCODER2_H