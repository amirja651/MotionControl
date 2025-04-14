#include "Logger.h"

Logger::Logger() : currentLevel(LogLevel::INFO), serialEnabled(true)
{
    // Initialize with smaller initial capacity
    logBuffer.reserve(10);
}

bool Logger::initialize(uint32_t serialBaudRate)
{
    if (serialBaudRate > 0)
    {
        Serial.begin(serialBaudRate);
        serialEnabled = true;

        // Wait for serial to be ready, but with timeout
        uint32_t startTime = millis();
        while (!Serial && (millis() - startTime) < 1000)
        {
            delay(10);
        }

        // Use direct Serial.println to avoid potential memory issues during initialization
        Serial.println(F("\n-------------------------> Logger initialized"));
        return true;
    }
    serialEnabled = false;
    return true;
}

void Logger::setLogLevel(LogLevel level)
{
    currentLevel = level;
    String msg   = "Log level set to: " + levelToString(level);
    info(msg, LogModule::SYSTEM);
}

void Logger::error(const String& message, LogModule module, const String& additionalInfo)
{
    log(LogLevel::ERROR, message, module, additionalInfo);
}

void Logger::warning(const String& message, LogModule module, const String& additionalInfo)
{
    log(LogLevel::WARNING, message, module, additionalInfo);
}

void Logger::info(const String& message, LogModule module, const String& additionalInfo)
{
    log(LogLevel::INFO, message, module, additionalInfo);
}

void Logger::debug(const String& message, LogModule module, const String& additionalInfo)
{
    log(LogLevel::DEBUGG, message, module, additionalInfo);
}

void Logger::log(LogLevel level, const String& message, LogModule module, const String& additionalInfo)
{
    // Check if we should log this level
    if (static_cast<int>(level) > static_cast<int>(currentLevel))
    {
        return;
    }

    // Create new log entry
    LogEntry entry;
    entry.timestamp      = millis();
    entry.level          = static_cast<uint8_t>(level);
    entry.module         = static_cast<uint8_t>(module);
    entry.additionalInfo = additionalInfo;
    entry.message        = message;

    // Add to buffer with size limit
    if (logBuffer.size() >= MAX_LOG_ENTRIES)
    {
        logBuffer.erase(logBuffer.begin());
    }

    // Use try-catch to prevent memory allocation failures
    try
    {
        logBuffer.push_back(entry);
    }
    catch (...)
    {
        // If push_back fails, clear buffer and try again
        logBuffer.clear();
        logBuffer.push_back(entry);
    }

    // Output to serial if enabled
    if (serialEnabled)
    {
        Serial.println(formatEntry(entry));
    }
}

void Logger::enableSerialOutput(bool enable)
{
    serialEnabled = enable;
    String msg    = enable ? F("Serial output enabled") : F("Serial output disabled");
    info(msg, LogModule::SYSTEM);
}

void Logger::clear()
{
    logBuffer.clear();
    info("Log buffer cleared", LogModule::SYSTEM);
}

size_t Logger::getEntryCount() const
{
    return logBuffer.size();
}

const LogEntry* Logger::getEntry(size_t index) const
{
    if (index >= logBuffer.size())
    {
        return nullptr;
    }
    return &logBuffer[index];
}

String Logger::formatEntry(const LogEntry& entry) const
{
    char buffer[32];
    snprintf(buffer, sizeof(buffer), "[%7u] ", entry.timestamp);

    String formattedMessage = buffer;
    formattedMessage += levelToString(static_cast<LogLevel>(entry.level));
    formattedMessage += " [";
    formattedMessage += moduleToString(static_cast<LogModule>(entry.module));
    formattedMessage += " - ";
    formattedMessage += entry.additionalInfo;
    formattedMessage += "] ";
    formattedMessage += addIcon(static_cast<LogLevel>(entry.level));
    formattedMessage += entry.message;

    return formattedMessage;
}

String Logger::addIcon(LogLevel level) const
{
    switch (level)
    {
        case LogLevel::ERROR:
            return "❌";
        case LogLevel::WARNING:
            return "⚠️";
        case LogLevel::INFO:
            return "ℹ️";
        case LogLevel::DEBUGG:
            return "🔍";
        default:
            return "";
    }
}

String Logger::levelToString(LogLevel level)
{
    switch (level)
    {
        case LogLevel::ERROR:
            return "ERROR  ";
        case LogLevel::WARNING:
            return "WARNING";
        case LogLevel::INFO:
            return "INFO   ";
        case LogLevel::DEBUGG:
            return "DEBUG  ";
        default:
            return "UNKNOWN";
    }
}

String Logger::moduleToString(LogModule module)
{
    switch (module)
    {
        case LogModule::SYSTEM:
            return "SYSTEM ";
        case LogModule::MOTOR:
            return "MOTOR  ";
        case LogModule::DRIVER:
            return "DRIVER ";
        case LogModule::COMMAND:
            return "COMMAND";
        case LogModule::CONFIG:
            return "CONFIG ";
        default:
            return "UNKNOWN";
    }
}

void Logger::flush()
{
    Serial.flush();
    logBuffer.clear();
}
