#ifndef TMC5160T_DRIVER_H
#define TMC5160T_DRIVER_H

#include "ESP32.h"

enum class MotorType
{
    LINEAR,
    ROTATIONAL
};

namespace CONFIG
{
    struct ENCODER
    {
        static const uint8_t ENC1 = ESP32W::LEFTPINS::D36_ENC1;
        static const uint8_t ENC2 = ESP32W::LEFTPINS::D39_ENC2;
        static const uint8_t ENC3 = ESP32W::LEFTPINS::D34_ENC3;
        static const uint8_t ENC4 = ESP32W::LEFTPINS::D35_ENC4;
    };

    struct MOTOR
    {
        static const uint8_t DIR_A  = ESP32W::RIGHTPINS::D22_A1;  // Blue
        static const uint8_t STEP_A = ESP32W::RIGHTPINS::D21_A2;  // Red
        static const uint8_t CS_A   = ESP32W::RIGHTPINS::D5_A3;   // Green
        static const uint8_t EN_A   = ESP32W::RIGHTPINS::D17_A4;  // Black

        static const uint8_t EN_B   = ESP32W::RIGHTPINS::D16_B1;  // Blue
        static const uint8_t CS_B   = ESP32W::RIGHTPINS::D4_B2;   // Red
        static const uint8_t STEP_B = ESP32W::RIGHTPINS::D2_B3;   // Green
        static const uint8_t DIR_B  = ESP32W::RIGHTPINS::D15_B4;  // Black

        static const uint8_t EN_C   = ESP32W::LEFTPINS::D26_C4;  // Blue
        static const uint8_t CS_C   = ESP32W::LEFTPINS::D25_C3;  // Red
        static const uint8_t STEP_C = ESP32W::LEFTPINS::D33_C2;  // Green
        static const uint8_t DIR_C  = ESP32W::LEFTPINS::D32_C1;  // Black

        static const uint8_t EN_D   = ESP32W::LEFTPINS::D13_D4;  // Blue
        static const uint8_t CS_D   = ESP32W::LEFTPINS::D12_D3;  // Red
        static const uint8_t STEP_D = ESP32W::LEFTPINS::D14_D2;  // Green
        static const uint8_t DIR_D  = ESP32W::LEFTPINS::D27_D1;  // Black
    };

    struct PID
    {
        static constexpr double KP = 2.0;
        static constexpr double KI = 0.5;
        static constexpr double KD = 0.1;
    };
}  // namespace CONFIG

#endif  // TMC5160T_DRIVER_H