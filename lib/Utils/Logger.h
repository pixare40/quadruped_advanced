/**
 * @file Logger.h
 * @brief Advanced logging system for debugging and telemetry
 * @author Advanced Quadruped Team
 * @date 2024
 */

#pragma once
#include <Arduino.h>

namespace Utils {

enum class LogLevel {
    TRACE = 0,
    DEBUG = 1,
    INFO = 2,
    WARN = 3,
    ERROR = 4,
    FATAL = 5
};

class Logger {
public:
    static void initialize(uint32_t baud_rate = 115200);
    static void setLevel(LogLevel level);
    
    static void trace(const char* format, ...);
    static void debug(const char* format, ...);
    static void info(const char* format, ...);
    static void warn(const char* format, ...);
    static void error(const char* format, ...);
    static void fatal(const char* format, ...);
    
private:
    static LogLevel current_level;
    static bool initialized;
    static void log(LogLevel level, const char* format, va_list args);
    static const char* levelToString(LogLevel level);
};

// Convenience macros
#ifdef DEBUG_BUILD
    #define LOG_TRACE(...) Utils::Logger::trace(__VA_ARGS__)
    #define LOG_DEBUG(...) Utils::Logger::debug(__VA_ARGS__)
    #define LOG_INFO(...) Utils::Logger::info(__VA_ARGS__)
    #define LOG_WARN(...) Utils::Logger::warn(__VA_ARGS__)
    #define LOG_ERROR(...) Utils::Logger::error(__VA_ARGS__)
    #define LOG_FATAL(...) Utils::Logger::fatal(__VA_ARGS__)
#else
    #define LOG_TRACE(...)
    #define LOG_DEBUG(...)
    #define LOG_INFO(...) Utils::Logger::info(__VA_ARGS__)
    #define LOG_WARN(...) Utils::Logger::warn(__VA_ARGS__)
    #define LOG_ERROR(...) Utils::Logger::error(__VA_ARGS__)
    #define LOG_FATAL(...) Utils::Logger::fatal(__VA_ARGS__)
#endif

} // namespace Utils