#ifndef TMC5160T_DRIVER_H
#define TMC5160T_DRIVER_H

#include "ESP32.h"

namespace CONFIG
{
    struct MOTOR
    {
        static const uint8_t DIR1 = ESP32W::RIGHTPINS::GPIO17;
        static const uint8_t DIR2 = ESP32W::RIGHTPINS::GPIO16;
        static const uint8_t DIR3 = ESP32W::LEFTPINS::GPIO32;
        static const uint8_t DIR4 = ESP32W::LEFTPINS::GPIO13;

        static const uint8_t STEP1 = ESP32W::RIGHTPINS::GPIO5;
        static const uint8_t STEP2 = ESP32W::RIGHTPINS::GPIO4;
        static const uint8_t STEP3 = ESP32W::LEFTPINS::GPIO33;
        static const uint8_t STEP4 = ESP32W::LEFTPINS::GPIO12;

        static const uint8_t EN1 = ESP32W::RIGHTPINS::GPIO21;
        static const uint8_t EN2 = ESP32W::RIGHTPINS::GPIO2;
        static const uint8_t EN3 = ESP32W::LEFTPINS::GPIO25;
        static const uint8_t EN4 = ESP32W::LEFTPINS::GPIO14;

        static const uint8_t CS1 = ESP32W::RIGHTPINS::GPIO22;
        static const uint8_t CS2 = ESP32W::RIGHTPINS::GPIO15;
        static const uint8_t CS3 = ESP32W::LEFTPINS::GPIO26;
        static const uint8_t CS4 = ESP32W::LEFTPINS::GPIO27;
    };

    struct ENCODER
    {
        static const uint8_t ENC1 = ESP32W::LEFTPINS::GPIO36;
        static const uint8_t ENC2 = ESP32W::LEFTPINS::GPIO39;
        static const uint8_t ENC3 = ESP32W::LEFTPINS::GPIO34;
        static const uint8_t ENC4 = ESP32W::LEFTPINS::GPIO35;
    };

    struct PID
    {
        static constexpr double KP = 2.0;
        static constexpr double KI = 0.5;
        static constexpr double KD = 0.1;
    };
}  // namespace CONFIG

#endif  // TMC5160T_DRIVER_H