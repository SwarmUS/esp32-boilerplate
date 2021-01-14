#include "ROSLogger.h"
#include "ros/ros.h"

void print(const char* format, va_list a);

ROSLogger::ROSLogger(LogLevel logLevel) : m_logLevel(logLevel) {}

void ROSLogger::log(LogLevel level, const char* format, va_list args) const {
    if (level >= m_logLevel) {
        const int bufferSize = 512;
        char buffer[bufferSize];
        vsnprintf(buffer, bufferSize, format, args);
        switch (level) {
        case ERROR:
            error(buffer);
            break;
        case WARNING:
            warn(buffer);
            break;
        case DEBUG:
            debug(buffer);
            break;
        case INFO:
        default:
            info(buffer);
            break;
        }
    }
}