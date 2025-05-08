#ifndef CONSTANTS_H
#define CONSTANTS_H

constexpr double PIXELS_PER_MM = 5200.0;

constexpr double ROTATIONAL_POSITION_MIN = 0.1f;
constexpr double ROTATIONAL_POSITION_MAX = 359.9f;

constexpr double ROTATIONAL_OUTPUT_LIMIT_MIN = -180.0;
constexpr double ROTATIONAL_OUTPUT_LIMIT_MAX = 180.0;

constexpr double LINEAR_LOWER_LIMIT_PX = 550.0;
constexpr double LINEAR_UPPER_LIMIT_PX = 900.0;
constexpr double LINEAR_OFFSET_PX      = 680.0;

constexpr double ROTATIONAL_THRESHOLD = 0.5f;
constexpr double LINEAR_THRESHOLD     = 0.3f;

#endif  // CONSTANTS_H
