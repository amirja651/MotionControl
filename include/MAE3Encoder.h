#ifndef MAE3_ENCODER_H
#define MAE3_ENCODER_H

#include <Arduino.h>

class MAE3Encoder
{
public:
    // Constants
    static constexpr uint32_t MIN_PULSE_WIDTH    = 5;     // Minimum valid pulse width in microseconds
    static constexpr uint32_t MAX_PULSE_WIDTH    = 4121;  // Maximum valid pulse width in microseconds
    static constexpr float    POSITION_THRESHOLD = 0.5f;  // Minimum position change in degrees
    static constexpr uint32_t FILTER_SIZE        = 4;     // Size of the median filter
    static constexpr float    PULSE_TO_DEGREE    = 360.0f / MAX_PULSE_WIDTH;  // Conversion factor
    static constexpr float    DEGREE_TO_PULSE    = MAX_PULSE_WIDTH / 360.0f;  // Conversion factor

    /**
     * @brief Constructor for MAE3Encoder
     * @param signalPin GPIO pin connected to encoder PWM output
     */
    MAE3Encoder(uint8_t signalPin);

    /**
     * @brief Initialize the encoder
     */
    void begin();

    /**
     * @brief Read and update the current position
     * @return true if position changed, false otherwise
     */
    bool update();

    /**
     * @brief Get the current position in degrees (0-360)
     * @return Current position in degrees
     */
    float getPositionDegrees() const;

    /**
     * @brief Get the raw pulse width in microseconds
     * @return Pulse width in microseconds
     */
    uint32_t getPulseWidth() const;

    /**
     * @brief Get the current velocity in degrees per second
     * @return Current velocity in degrees per second
     */
    float getVelocityDPS() const;

    /**
     * @brief Convert degrees to pulse width
     * @param degree Angle in degrees
     * @return Corresponding pulse width in microseconds
     */
    uint32_t convertToPulseWidth(float degree);

private:
    // Static interrupt handler
    static void handleInterrupt();

    // Median filter implementation
    uint32_t medianFilter();

    // Pin assignments
    const uint8_t signalPin;

    // Interrupt-related members
    static MAE3Encoder*    instance;           // Static instance pointer for interrupt handler
    volatile uint32_t      currentPulseWidth;  // Current pulse width measurement
    volatile bool          newPulseAvailable;  // Flag indicating new pulse measurement
    volatile unsigned long pulseStartTime;     // Start time of current pulse

    // Filtering and position tracking
    uint32_t      pulseFilter[FILTER_SIZE];  // Array for median filtering
    uint8_t       filterIndex;               // Current index in filter array
    float         lastValidPosition;         // Last valid position in degrees
    float         currentVelocity;           // Current velocity in degrees per second
    unsigned long velocityUpdateTime;        // Last velocity update timestamp
    unsigned long lastUpdateTime;            // Last update timestamp
};

#endif  // MAE3_ENCODER_H