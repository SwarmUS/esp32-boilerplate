#include "UserInterface.h"
#include "ros/ros.h"
#include <cstdarg>

int UserInterface::print(const char* format, va_list args) const {
    int ret = vprintf(format, args);
    printf("\n\r");
    return ret;
}

int UserInterface::print(const char* format, ...) const {
    va_list args;
    va_start(args, format);
    int retValue = print(format, args);
    va_end(args);

    return retValue;
}

int UserInterface::logInfo(const char* format, va_list args) const {
    const int bufferSize = 512;
    char buffer[bufferSize];
    int ret = vsnprintf(buffer, bufferSize, format, args);
    ROS_INFO("%s", buffer);
    return ret;
}

int UserInterface::logDebug(const char* format, va_list args) const {
    const int bufferSize = 512;
    char buffer[bufferSize];
    int ret = vsnprintf(buffer, bufferSize, format, args);
    ROS_DEBUG("%s", buffer);
    return ret;
};

int UserInterface::logWarn(const char* format, va_list args) const {
    const int bufferSize = 512;
    char buffer[bufferSize];
    int ret = vsnprintf(buffer, bufferSize, format, args);
    ROS_WARN("%s", buffer);
    return ret;
};

int UserInterface::logError(const char* format, va_list args) const {
    const int bufferSize = 512;
    char buffer[bufferSize];
    int ret = vsnprintf(buffer, bufferSize, format, args);
    ROS_ERROR("%s", buffer);
    return ret;
}