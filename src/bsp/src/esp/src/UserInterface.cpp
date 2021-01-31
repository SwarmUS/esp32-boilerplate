#include "UserInterface.h"
#include <cstdio>

int UserInterface::print(const char* format, va_list args) const {
    int ret = vprintf(format, args);
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
    int retValue = vprintf(format, args);
    UserInterface::print("\r\n");
    return retValue;
}

int UserInterface::printDebug(const char* format, va_list args) const {
    int retValue = vprintf(format, args);
    UserInterface::print("\r\n");
    return retValue;
};

int UserInterface::printWarning(const char* format, va_list args) const {
    int retValue = vprintf(format, args);
    UserInterface::print("\r\n");
    return retValue;
};

int UserInterface::printError(const char* format, va_list args) const {
    int retValue = vprintf(format, args);
    UserInterface::print("\r\n");
    return retValue;
}