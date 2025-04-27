#ifndef SPI_H
#define SPI_H

#include "ESP32.h"

namespace CONFIG
{
    struct SPI
    {
        static const uint8_t MOSI = ESP32W::RIGHTPINS::D23_MOSI;
        static const uint8_t MISO = ESP32W::RIGHTPINS::D19_MISO;
        static const uint8_t SCK  = ESP32W::RIGHTPINS::D18_SCK;
    };
}  // namespace CONFIG

#endif  // SPI_H