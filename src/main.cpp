#include <Arduino.h>
#include "Logger.h"

// Global logger instance
Logger logger;

void setup()
{
    logger.initialize(115200);
    logger.info("Hello, World!", LogModule::SYSTEM);
}

void loop()
{
    delay(10);
}