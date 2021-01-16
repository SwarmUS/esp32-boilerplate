#ifndef USERINTERFACE_H
#define USERINTERFACE_H

#include "bsp/IUserInterface.h"

class UserInterface : public IUserInterface {
  public:
    UserInterface() = default;
    ~UserInterface() override = default;

    int print(const char* format, ...) const override;

    int print(const char* format, va_list args) const override;

    int logInfo(const char* format, va_list args) const override;

    int logDebug(const char* format, va_list args) const override;

    int logWarn(const char* format, va_list args) const override;

    int logError(const char* format, va_list args) const override;
};

#endif // HIVE_CONNECT_USERINTERFACE_H
