#ifndef TMC5160T_DRIVER_H
#define TMC5160T_DRIVER_H

#include "ESP32.h"

enum class MotorType
{
    LINEAR,
    ROTATIONAL
};

static const uint16_t DIR_A = 22;
static const uint16_t DIR_B = 15;
static const uint16_t DIR_C = 32;
static const uint16_t DIR_D = 27;

static const uint16_t STEP_A = 21;
static const uint16_t STEP_B = 2;
static const uint16_t STEP_C = 33;
static const uint16_t STEP_D = 14;

static const uint16_t CS_A = 5;
static const uint16_t CS_B = 4;
static const uint16_t CS_C = 25;
static const uint16_t CS_D = 12;

static const uint16_t EN_A = 17;
static const uint16_t EN_B = 16;
static const uint16_t EN_C = 26;
static const uint16_t EN_D = 13;

static const uint16_t ENC_A = 36;
static const uint16_t ENC_B = 39;
static const uint16_t ENC_C = 34;
static const uint16_t ENC_D = 35;

static constexpr double _KP = 2.0;
static constexpr double _KI = 0.5;
static constexpr double _KD = 0.1;

#endif  // TMC5160T_DRIVER_H