/*
 * Logger Module
 * Provides efficient logging functionality for the motor control system
 */

#ifndef LOGGER_H
#define LOGGER_H

#include <Arduino.h>
#include <vector>

/**
 * Log levels in order of priority
 */
enum class LogLevel
{
    ERROR   = 0,  // Critical errors
    WARNING = 1,  // Warnings
    INFO    = 2,  // Informational messages
    DEBUGG  = 3   // Debug messages
};

/**
 * Module identifiers for targeted logging
 */
enum class LogModule
{
    SYSTEM  = 0,  // System-wide logs
    MOTOR   = 1,  // Motor control logs
    DRIVER  = 2,  // Driver logs
    COMMAND = 3,  // Command processing logs
    CONFIG  = 4   // Configuration logs
};

/**
 * Log entry structure optimized for memory usage
 */
struct LogEntry
{
    uint32_t timestamp;  // Timestamp in milliseconds
    uint8_t  level;      // Log level (packed)
    uint8_t  module;     // Module identifier (packed)
    String   message;    // Log message
};

/**
 * Logger class for system logging
 */
class Logger
{
public:
    /**
     * Constructor
     */
    Logger();

    /**
     * Initialize the logger
     * @param serialBaudRate Serial baud rate (0 to disable serial output)
     * @return true if initialization successful
     */
    bool initialize(uint32_t serialBaudRate = 115200);

    /**
     * Set global log level
     * @param level Minimum log level to output
     */
    void setLogLevel(LogLevel level);

    /**
     * Log an error message
     * @param message Message to log
     * @param module Source module
     */
    void error(const String& message, LogModule module = LogModule::SYSTEM);

    /**
     * Log a warning message
     * @param message Message to log
     * @param module Source module
     */
    void warning(const String& message, LogModule module = LogModule::SYSTEM);

    /**
     * Log an info message
     * @param message Message to log
     * @param module Source module
     */
    void info(const String& message, LogModule module = LogModule::SYSTEM);

    /**
     * Log a debug message
     * @param message Message to log
     * @param module Source module
     */
    void debug(const String& message, LogModule module = LogModule::SYSTEM);

    /**
     * Enable/disable serial output
     * @param enable true to enable serial output
     */
    void enableSerialOutput(bool enable);

    /**
     * Clear the log buffer
     */
    void clear();

    /**
     * Get number of log entries
     * @return Number of entries in log buffer
     */
    size_t getEntryCount() const;

    /**
     * Get a log entry
     * @param index Entry index
     * @return Pointer to log entry or nullptr if invalid
     */
    const LogEntry* getEntry(size_t index) const;

    // Internal log function - moved from private to public
    void log(LogLevel level, const String& message, LogModule module);

private:
    static const size_t MAX_LOG_ENTRIES = 100;  // Maximum number of log entries to store

    std::vector<LogEntry> logBuffer;      // Circular buffer for log entries
    LogLevel              currentLevel;   // Current log level
    bool                  serialEnabled;  // Serial output enabled flag

    /**
     * Format log entry for output
     * @param entry Log entry to format
     * @return Formatted string
     */
    String formatEntry(const LogEntry& entry) const;

    /**
     * Convert log level to string
     * @param level Log level
     * @return String representation
     */
    static String levelToString(LogLevel level);

    /**
     * Convert module to string
     * @param module Module identifier
     * @return String representation
     */
    static String moduleToString(LogModule module);
};

#endif  // LOGGER_H
