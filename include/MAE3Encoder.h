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
constexpr float LEAD_SCREW_PITCH_MM = 0.5f;      // Lead screw pitch in mm
constexpr float TOTAL_TRAVEL_MM     = 30.0f;     // Total travel distance in mm
constexpr float LEAD_SCREW_PITCH_UM = 500.0f;    // 0.5mm = 500μm
constexpr float TOTAL_TRAVEL_UM     = 30000.0f;  // 30mm = 30000μm

struct EncoderState
{
    volatile int64_t current_pulse;  // Current pulse value
    volatile int64_t last_pulse;     // Current pulse value

    volatile int64_t width_high;  // High pulse width (rising to falling)
    volatile int64_t width_low;   // Low pulse width (falling to rising)

    volatile int64_t period;  // Total pulse width (t_on + t_off)
    volatile int64_t laps;    // Number of complete rotations

    Direction direction = Direction::UNKNOWN;  // Current direction of rotation

    // Sector tracking
    static constexpr int64_t DEFAULT_MAX_T    = 4096LL;  // Default 4096 value
    static constexpr int64_t NUM_SECTORS      = 64LL;
    static constexpr int64_t STEPS_PER_SECTOR = DEFAULT_MAX_T / NUM_SECTORS;  // 64

    int64_t touched_sectors = 0;  // Bitmap of touched sectors
    int64_t current_sector  = 0;  // Current sector (0-63)
    int64_t last_sector     = 0;  // Last sector for direction detection
    int     touched_count   = 0;  // Count of touched sectors
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
    float getDegreesPerPulse() const
    {
        return 360.0f / static_cast<float>(EncoderState::DEFAULT_MAX_T);
    }

    // Millimeters per pulse
    float getMMPerPulse() const
    {
        return LEAD_SCREW_PITCH_MM / static_cast<float>(EncoderState::DEFAULT_MAX_T);
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
        return static_cast<float>(state.current_pulse) * getUMPerPulse();
    }

    // Total travel in μm
    float getTotalTravelPulse() const
    {
        float totalDistanceUM = (static_cast<float>(state.laps) * static_cast<float>(EncoderState::DEFAULT_MAX_T)) +
                                static_cast<float>(state.current_pulse);
        return totalDistanceUM;
    }

    // Total travel in mm
    float getTotalTravelMM() const
    {
        float totalDistance = (static_cast<float>(state.laps) * LEAD_SCREW_PITCH_MM) + getPositionMM();
        return totalDistance;
    }

    // Total travel in μm
    float getTotalTravelUM() const
    {
        float totalDistanceUM = (static_cast<float>(state.laps) * LEAD_SCREW_PITCH_UM) + getPositionUM();
        return totalDistanceUM;
    }

    void reset();

    // New method for processing PWM signal
    void processPWM();

    // --- Pulse width ring buffers ---
    static constexpr size_t                       PULSE_BUFFER_SIZE = 10;
    const std::array<int64_t, PULSE_BUFFER_SIZE>& getWidthHBuffer() const
    {
        return widthHBuffer;
    }

    const std::array<int64_t, PULSE_BUFFER_SIZE>& getWidthLBuffer() const
    {
        return widthLBuffer;
    }

    size_t getPulseBufferIndex() const
    {
        return pulseBufferIndex;
    }

    // Median filter for width_high
    int64_t getMedianWidthHigh() const;

    // Median filter for width_low
    int64_t getMedianWidthLow() const;

    // Sector tracking methods
    uint8_t getCurrentSector() const
    {
        return state.current_sector;
    }

    uint8_t getTouchedSectorCount() const
    {
        return state.touched_count;
    }

    uint64_t getTouchedSectors() const
    {
        return state.touched_sectors;
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
    std::array<int64_t, PULSE_BUFFER_SIZE> widthLBuffer{};
    std::array<int64_t, PULSE_BUFFER_SIZE> widthHBuffer{};
    size_t                                 pulseBufferIndex = 0;

    void updateSectorTracking();
    void printBinary64(uint64_t value);
};

#endif  // MAE3_ENCODER2_H