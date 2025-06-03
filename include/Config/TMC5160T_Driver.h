#ifndef TMC5160T_DRIVER_H
#define TMC5160T_DRIVER_H

#include "ESP32.h"

enum class MotorType
{
    LINEAR,
    ROTATIONAL
};

static const uint16_t pDIR_A = 22;
static const uint16_t pDIR_B = 15;
static const uint16_t pDIR_C = 32;
static const uint16_t pDIR_D = 27;

static const uint16_t pSTEP_A = 21;
static const uint16_t pSTEP_B = 2;
static const uint16_t pSTEP_C = 33;
static const uint16_t pSTEP_D = 14;

static const uint16_t pCS_A = 5;
static const uint16_t pCS_B = 4;
static const uint16_t pCS_C = 25;
static const uint16_t pCS_D = 12;

static const uint16_t pEN_A = 17;
static const uint16_t pEN_B = 16;
static const uint16_t pEN_C = 26;
static const uint16_t pEN_D = 13;

static const uint16_t ENC_A = 36;
static const uint16_t ENC_B = 39;
static const uint16_t ENC_C = 34;
static const uint16_t ENC_D = 35;

static constexpr double _KP = 2.0;
static constexpr double _KI = 0.5;
static constexpr double _KD = 0.1;

#endif  // TMC5160T_DRIVER_H