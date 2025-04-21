#ifndef MAE3_ENCODER_H
#define MAE3_ENCODER_H

#include <Arduino.h>

class MAE3Encoder
{
public:
    static constexpr uint32_t MIN_PULSE_WIDTH    = 5;     // Minimum valid pulse width in microseconds
    static constexpr uint32_t MAX_PULSE_WIDTH    = 4121;  // Maximum valid pulse width in microseconds
    static constexpr double   POSITION_THRESHOLD = 0.5;   // Minimum position change in degrees
    static constexpr uint32_t FILTER_SIZE        = 4;     // Size of the median filter
    static constexpr double   PULSE_TO_DEGREE    = 360.0f / MAX_PULSE_WIDTH;  // Conversion factor
    static constexpr double   DEGREE_TO_PULSE    = MAX_PULSE_WIDTH / 360.0;   // Conversion factor

    MAE3Encoder(uint8_t signalPin, uint8_t interruptPin);

    void begin();
    bool update();

    double getPositionDegrees() const
    {
        return lastValidPosition;
    }

    uint32_t getPulseWidth() const
    {
        return pulseFilter[filterIndex];
    }

    double getVelocityDPS() const
    {
        return currentVelocity;
    }

    uint32_t convertToPulseWidth(double degree) const
    {
        return static_cast<uint32_t>(degree * DEGREE_TO_PULSE);
    }

    void     handleInterrupt();
    uint32_t medianFilter();

    const uint8_t signalPin : 6;     // Using 6 bits for pin number (0-63)
    const uint8_t interruptPin : 6;  // Using 6 bits for pin number (0-63)

    volatile uint32_t      currentPulseWidth;  // Current pulse width measurement
    volatile bool          newPulseAvailable;  // Flag indicating new pulse measurement
    volatile unsigned long pulseStartTime;     // Start time of current pulse

    uint32_t      pulseFilter[FILTER_SIZE];  // Array for median filtering
    uint8_t       filterIndex;               // Current index in filter array
    double        lastValidPosition;         // Last valid position in degrees
    double        currentVelocity;           // Current velocity in degrees per second
    unsigned long velocityUpdateTime;        // Last velocity update timestamp
    unsigned long lastUpdateTime;            // Last update timestamp
private:
};

#endif  // MAE3_ENCODER_H