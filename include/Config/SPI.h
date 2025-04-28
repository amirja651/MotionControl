#ifndef SPI_H
#define SPI_H

#include "ESP32.h"

namespace CONFIG
{
    struct SPI
    {
        static const uint16_t MOSI = 23;
        static const uint16_t MISO = 19;
        static const uint16_t SCK  = 18;
    };
}  // namespace CONFIG

#endif  // SPI_H