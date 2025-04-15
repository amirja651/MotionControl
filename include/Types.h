#ifndef TYPES_H
#define TYPES_H

#include <Arduino.h>
#include <functional>

/**
 * Common types and definitions used across the motion control system
 */
namespace Types
{
    // Position types
    using EncoderPosition = int32_t;
    using MicronPosition  = float;
    using PixelPosition   = float;
    using StepPosition    = int32_t;

    // Speed and acceleration types
    using Speed        = float;
    using Acceleration = float;

    // Callback function types
    using StatusCallback   = std::function<void(bool)>;
    using CompleteCallback = std::function<void(void)>;

    // Error codes
    enum class ErrorCode
    {
        SUCCESS = 0,
        LIMIT_SWITCH_TRIGGERED,
        POSITION_OUT_OF_BOUNDS,
        CALIBRATION_FAILED,
        TIMEOUT
    };
}  // namespace Types

#endif  // TYPES_H