#ifndef SPI_H
#define SPI_H

#include "ESP32.h"

namespace CONFIG
{
    struct SPI
    {
        static const uint8_t MOSI = ESP32W::RIGHTPINS::GPIO23;
        static const uint8_t MISO = ESP32W::RIGHTPINS::GPIO19;
        static const uint8_t SCK  = ESP32W::RIGHTPINS::GPIO18;
    };
}  // namespace CONFIG

#endif  // SPI_H