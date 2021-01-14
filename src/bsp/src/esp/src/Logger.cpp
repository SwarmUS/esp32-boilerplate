#include "Logger.h"
#include <cstdio>
#

Logger::Logger(LogLevel logLevel) : m_logLevel(logLevel) {}

void Logger::log(LogLevel level, const char* format, va_list args) const {
    if (level >= m_logLevel) {
        const int bufferSize = 512;
        char buffer[bufferSize];
        vsnprintf(buffer, bufferSize, format, args);
        printf("%s\n\r", buffer);
    }
}