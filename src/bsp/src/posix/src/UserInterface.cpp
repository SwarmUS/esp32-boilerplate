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

int UserInterface::printInfo(const char* format, va_list args) const {
    const int bufferSize = 512;
    char buffer[bufferSize];
    int ret = vsnprintf(buffer, bufferSize, format, args);
    ROS_INFO("%s", buffer);
    return ret;
}

int UserInterface::printDebug(const char* format, va_list args) const {
    std::pair<char*, int> statement = UserInterface::generateBuffer(format, args);
    ROS_DEBUG("%s", statement.first);
    return statement.second;
};

int UserInterface::printWarning(const char* format, va_list args) const {
    std::pair<char*, int> statement = UserInterface::generateBuffer(format, args);
    ROS_WARN("%s", statement.first);
    return statement.second;
};

int UserInterface::printError(const char* format, va_list args) const {
    std::pair<char*, int> statement = UserInterface::generateBuffer(format, args);
    ROS_ERROR("%s", statement.first);
    return statement.second;
}

std::pair<char*, int> UserInterface::generateBuffer(const char* format, va_list args) {
    std::pair<char*, int> retVal;
    const int bufferSize = 512;
    char buffer[bufferSize];
    retVal.second = vsnprintf(buffer, bufferSize, format, args);
    retVal.first = buffer;
    return retVal;
}