#ifndef MAE3_ENCODER_H
#define MAE3_ENCODER_H

#include <Arduino.h>

class MAE3Encoder
{
public:
    // Constants
    static constexpr uint32_t MIN_PULSE_WIDTH = 5;                         // Minimum valid pulse width in microseconds
    static constexpr uint32_t MAX_PULSE_WIDTH = 4121;                      // Maximum valid pulse width in microseconds
    static constexpr uint32_t FILTER_SIZE     = 4;                         // Size of the median filter
    static constexpr float    PULSE_TO_DEGREE = 360.0f / MAX_PULSE_WIDTH;  // Conversion factor
    static constexpr float    DEGREE_TO_PULSE = MAX_PULSE_WIDTH / 360.0f;  // Conversion factor

    MAE3Encoder(uint8_t signalPin, uint8_t interruptPin);

    void begin();

    bool update();

    float getPositionDegrees() const
    {
        return lastValidPosition;
    }

    uint32_t getPulseWidth() const
    {
        return pulseFilter[filterIndex];
    }

    float getVelocityDPS() const
    {
        return currentVelocity;
    }

    uint32_t convertToPulseWidth(float degree) const
    {
        return static_cast<uint32_t>(degree * DEGREE_TO_PULSE);
    }

    void handleInterrupt();

private:
    // Median filter implementation
    uint32_t medianFilter();

    // Pin assignments (packed to save memory)
    const uint8_t signalPin : 6;     // Using 6 bits for pin number (0-63)
    const uint8_t interruptPin : 6;  // Using 6 bits for pin number (0-63)

    // Interrupt-related members
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