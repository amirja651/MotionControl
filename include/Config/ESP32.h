#ifndef ESP32_H
#define ESP32_H

#include <stdint.h>

namespace CONFIG
{
    namespace ESP32W
    {
        struct LEFTPINS
        {
            static const uint16_t GPIO36 = 36;  // Input only, ADC1_0
            static const uint16_t GPIO39 = 39;  // Input only, ADC1_3
            static const uint16_t GPIO34 = 34;  // Input only, ADC1_6
            static const uint16_t GPIO35 = 35;  // Input only, ADC1_7
            static const uint16_t GPIO32 = 32;  // ADC1_4, Touch9, XTAL_32K_P
            static const uint16_t GPIO33 = 33;  // ADC1_5, Touch8, XTAL_32K_N
            static const uint16_t GPIO25 = 25;  // ADC2_8, DAC1
            static const uint16_t GPIO26 = 26;  // ADC2_9, DAC2
            static const uint16_t GPIO27 = 27;  // ADC2_7, Touch7
            static const uint16_t GPIO14 = 14;  // ADC2_6, Touch6
            static const uint16_t GPIO12 = 12;  // ADC2_5, Touch5
            static const uint16_t GPIO13 = 13;  // ADC2_4, Touch4
        };

        struct RIGHTPINS
        {
            static const uint16_t GPIO23 = 23;  // MOSI, V_SPI_D
            static const uint16_t GPIO22 = 22;  // V_SPI_WP
            static const uint16_t GPIO1  = 1;   // TX0
            static const uint16_t GPIO3  = 3;   // RX0
            static const uint16_t GPIO21 = 21;  // V_SPI_HD
            static const uint16_t GPIO19 = 19;  // MISO, V_SPI_Q
            static const uint16_t GPIO18 = 18;  // SCK, V_SPI_CLK
            static const uint16_t GPIO5  = 5;   // V_SPI_CS0
            static const uint16_t GPIO17 = 17;  // TX2
            static const uint16_t GPIO16 = 16;  // RX2
            static const uint16_t GPIO4  = 4;   // ADC2_0, Touch0
            static const uint16_t GPIO2  = 2;   // ADC2_2, Touch2
            static const uint16_t GPIO15 = 15;  // ADC2_3, Touch3
        };
    };  // namespace ESP32W
}  // namespace CONFIG

#endif  // ESP32_H