#include "Logger.h"
#include <cstdio>
#

Logger::Logger(LogLevel logLevel) : m_logLevel(logLevel) {}

void Logger::log(LogLevel level, const char* format, ...) const {
    if (level >= m_logLevel) {
        const int bufferSize = 512;
        char buffer[bufferSize];
        va_list args;
        va_start(args, format);
        vsnprintf(buffer, bufferSize, format, args);
        va_end(args);
        printf("%s\n\r", buffer);
    }
}