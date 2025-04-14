#if !defined(ESP32)
#error This code is intended to run on the ESP32 platform! Please check your Tools->Board setting.
#endif

#include <Arduino.h>
#include <ESP32TimerInterrupt.h>
#include "Logger.h"
#include "MotorInstances.h"

// These define's must be placed at the beginning before #include "ESP32_New_TimerInterrupt.h"
// _TIMERINTERRUPT_LOGLEVEL_ from 0 to 4
// #define _TIMERINTERRUPT_LOGLEVEL_ 4

#define TIMER0_INTERVAL_MS 1000
#define TIMER0_DURATION_MS 5000

#define TIMER1_INTERVAL_MS 3000
#define TIMER1_DURATION_MS 15000

// Don't use PIN_D1 in core v2.0.0 and v2.0.1. Check https://github.com/espressif/arduino-esp32/issues/5868
// Don't use PIN_D2 with ESP32_C3 (crash)
#define PIN_D19 19  // Pin D19 mapped to pin GPIO9 of ESP32
#define PIN_D3  3   // Pin D3 mapped to pin GPIO3/RX0 of ESP32

// With core v2.0.0+, you can't use Serial.print/println in ISR or crash.
// and you can't use float calculation inside ISR
// Only OK in core v1.0.6-
bool IRAM_ATTR TimerHandler0(void* timerNo)
{
    static bool toggle0 = false;

    // timer interrupt toggles pin PIN_D19
    // digitalWrite(PIN_D19, toggle0);
    motors[0].moveForward();
    toggle0 = !toggle0;

    return true;
}

bool IRAM_ATTR TimerHandler1(void* timerNo)
{
    static bool toggle1 = false;

    // timer interrupt toggles outputPin
    // digitalWrite(PIN_D3, toggle1);
    motors[1].moveForward();
    toggle1 = !toggle1;

    return true;
}

// Global logger instance
Logger logger;

// Init ESP32 timer 0 and 1
ESP32Timer ITimer0(0);
ESP32Timer ITimer1(1);

void setup()
{
    logger.initialize(115200);
    logger.info("Hello, World!", LogModule::SYSTEM);
    logger.info("CPU Frequency = " + String(F_CPU / 1000000) + " MHz");
    delay(500);

    // Using ESP32  => 80 / 160 / 240MHz CPU clock ,
    // For 64-bit timer counter
    // For 16-bit timer prescaler up to 1024

    // Interval in microsecs
    if (ITimer0.attachInterruptInterval(TIMER0_INTERVAL_MS * 1000, TimerHandler0))
    {
        logger.info("Starting  ITimer0 OK, millis() = " + String(millis()));
    }
    else
    {
        logger.error("Can't set ITimer0. Select another freq. or timer");
    }

    // Interval in microsecs
    if (ITimer1.attachInterruptInterval(TIMER1_INTERVAL_MS * 1000, TimerHandler1))
    {
        logger.info("Starting  ITimer1 OK, millis() = " + String(millis()));
    }
    else
    {
        logger.error("Can't set ITimer1. Select another freq. or timer");
    }

    initializeMotors();

    logger.flush();
}

void loop()
{
    static unsigned long lastTimer0 = 0;
    static unsigned long lastTimer1 = 0;

    static bool timer0Stopped = false;
    static bool timer1Stopped = false;

    if (millis() - lastTimer0 > TIMER0_DURATION_MS)
    {
        lastTimer0 = millis();

        if (timer0Stopped)
        {
            logger.info("Start ITimer0, millis() = " + String(millis()));
            ITimer0.restartTimer();
        }
        else
        {
            logger.info("Stop ITimer0, millis() = " + String(millis()));
            ITimer0.stopTimer();
            motors[0].stop();
        }

        timer0Stopped = !timer0Stopped;
    }

    if (millis() - lastTimer1 > TIMER1_DURATION_MS)
    {
        lastTimer1 = millis();

        if (timer1Stopped)
        {
            logger.info("Start ITimer1, millis() = " + String(millis()));
            ITimer1.restartTimer();
        }
        else
        {
            logger.info("Stop ITimer1, millis() = " + String(millis()));
            ITimer1.stopTimer();
            motors[1].stop();
        }

        timer1Stopped = !timer1Stopped;
    }
}