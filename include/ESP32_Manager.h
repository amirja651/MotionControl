#ifndef ESP32_MANAGER_H
#define ESP32_MANAGER_H

#include <esp_system.h>  // Add this include for esp_reset_reason

const char* getResetReasonString(esp_reset_reason_t reason)
{
    switch (reason)
    {
        case ESP_RST_UNKNOWN:
            return "Unknown";
        case ESP_RST_POWERON:
            return "Power On";
        case ESP_RST_EXT:
            return "Reset by external pin ";
        case ESP_RST_SW:
            return "Software reset";
        case ESP_RST_PANIC:
            return "Software reset due to exception/panic";
        case ESP_RST_INT_WDT:
            return "Interrupt Watchdog";
        case ESP_RST_TASK_WDT:
            return "Task Watchdog";
        case ESP_RST_WDT:
            return "Other watchdogs";
        case ESP_RST_DEEPSLEEP:
            return "Deep Sleep";
        case ESP_RST_BROWNOUT:
            return "Brownout";
        case ESP_RST_SDIO:
            return "SDIO";
        default:
            return "Unknown";
    }
}

void printSystemInfo()
{
    Serial.println(F("\n\n=============== [System Initialization] ================="));
    Serial.println(F("CPU Frequency (MHz)  |  Free RAM (bytes)  |  Reset Reason"));

    // Format each value with fixed width
    char buffer[20];
    snprintf(buffer, sizeof(buffer), "%-19ld", F_CPU / 1000000);
    Serial.print(buffer);
    Serial.print(F("  |  "));
    snprintf(buffer, sizeof(buffer), "%-16d", ESP.getFreeHeap());
    Serial.print(buffer);
    Serial.print(F("  |  "));
    snprintf(buffer, sizeof(buffer), "%-12s", getResetReasonString(esp_reset_reason()));
    Serial.println(buffer);
    Serial.println();
}

#endif
