#include "ROSLogger.h"
#include "ros/ros.h"

void print(const char* format, va_list a);

ROSLogger::ROSLogger(LogLevel logLevel) : m_logLevel(logLevel) {}

void ROSLogger::log(LogLevel level, const char* format, ...) const {
    if (level >= m_logLevel) {
        const int bufferSize = 512;
        char buffer[bufferSize];
        va_list args;
        va_start(args, format);
        vsnprintf(buffer, bufferSize, format, args);
        va_end(args);
        switch (level) {
        case ERROR:
            ROS_ERROR("%s", buffer);
            break;
        case WARNING:
            ROS_WARN("%s", buffer);
            break;
        case DEBUG:
            ROS_DEBUG("%s", buffer);
            break;
        case INFO:
        default:
            ROS_INFO("%s", buffer);
            break;
        }
    }
}