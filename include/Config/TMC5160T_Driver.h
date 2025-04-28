#ifndef TMC5160T_DRIVER_H
#define TMC5160T_DRIVER_H

enum class MotorType
{
    LINEAR,
    ROTATIONAL
};

static const uint8_t DIR_A  = 22;
static const uint8_t STEP_A = 21;
static const uint8_t EN_A   = 17;
static const uint8_t CS_A   = 5;

static const uint8_t DIR_B  = 15;
static const uint8_t STEP_B = 2;
static const uint8_t EN_B   = 16;
static const uint8_t CS_B   = 5;  // 4;

static const uint8_t DIR_C  = 32;
static const uint8_t STEP_C = 33;
static const uint8_t EN_C   = 26;
static const uint8_t CS_C   = 5;  // 25;

static const uint8_t DIR_D  = 27;
static const uint8_t STEP_D = 14;
static const uint8_t EN_D   = 13;
static const uint8_t CS_D   = 5;  // 12;

static const uint8_t ENC_A = 36;
static const uint8_t ENC_B = 39;
static const uint8_t ENC_C = 34;
static const uint8_t ENC_D = 35;

static constexpr double _KP = 2.0;
static constexpr double _KI = 0.5;
static constexpr double _KD = 0.1;

#endif  // TMC5160T_DRIVER_H