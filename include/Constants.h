#ifndef CONSTANTS_H
#define CONSTANTS_H

constexpr double ROTATIONAL_POSITION_MIN = 0.1f;
constexpr double ROTATIONAL_POSITION_MAX = 359.9f;

constexpr double ROTATIONAL_OUTPUT_LIMIT_MIN = -180.0;
constexpr double ROTATIONAL_OUTPUT_LIMIT_MAX = 180.0;

constexpr double LINEAR_LOWER_LIMIT_PX = 550.0;
constexpr double LINEAR_UPPER_LIMIT_PX = 880.0;
constexpr double LINEAR_OFFSET_PX      = 680.0;

constexpr float ROTATIONAL_THRESHOLD         = 0.1f;
constexpr float ROTATIONAL_DISPLAY_THRESHOLD = 0.1f;
constexpr float LINEAR_THRESHOLD             = 0.1f;
constexpr float LINEAR_DISPLAY_THRESHOLD     = 0.1f;

#endif  // CONSTANTS_H
