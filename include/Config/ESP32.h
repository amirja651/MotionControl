#ifndef ESP32_H
#define ESP32_H

#include <stdint.h>

namespace CONFIG
{
    namespace ESP32W
    {
        struct LEFTPINS
        {
            static const uint8_t D36_ENC1 = 36;  // Input only, ADC1_0
            static const uint8_t D39_ENC2 = 39;  // Input only, ADC1_3
            static const uint8_t D34_ENC3 = 34;  // Input only, ADC1_6
            static const uint8_t D35_ENC4 = 35;  // Input only, ADC1_7
            static const uint8_t D32_C1   = 32;  // ADC1_4, Touch9, XTAL_32K_P
            static const uint8_t D33_C2   = 33;  // ADC1_5, Touch8, XTAL_32K_N
            static const uint8_t D25_C3   = 25;  // ADC2_8, DAC1
            static const uint8_t D26_C4   = 26;  // ADC2_9, DAC2
            static const uint8_t D27_D1   = 27;  // ADC2_7, Touch7
            static const uint8_t D14_D2   = 14;  // ADC2_6, Touch6
            static const uint8_t D12_D3   = 12;  // ADC2_5, Touch5
            static const uint8_t D13_D4   = 13;  // ADC2_4, Touch4
        };

        struct RIGHTPINS
        {
            static const uint8_t D23_MOSI = 23;  // MOSI, V_SPI_D
            static const uint8_t D22_A1   = 22;  // V_SPI_WP
            static const uint8_t D1_TX0   = 1;   // TX0
            static const uint8_t D3_RX0   = 3;   // RX0
            static const uint8_t D21_A2   = 21;  // V_SPI_HD
            static const uint8_t D19_MISO = 19;  // MISO, V_SPI_Q
            static const uint8_t D18_SCK  = 18;  // SCK, V_SPI_CLK
            static const uint8_t D5_A3    = 5;   // V_SPI_CS0
            static const uint8_t D17_A4   = 17;  // TX2
            static const uint8_t D16_B1   = 16;  // RX2
            static const uint8_t D4_B2    = 4;   // ADC2_0, Touch0
            static const uint8_t D2_B3    = 2;   // ADC2_2, Touch2
            static const uint8_t D15_B4   = 15;  // ADC2_3, Touch3
        };
    };  // namespace ESP32W
}  // namespace CONFIG

#endif  // ESP32_H