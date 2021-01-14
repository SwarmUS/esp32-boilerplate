#ifndef LOGGER_H
#define LOGGER_H

#include "bsp/ILogger.h"

class Logger : public ILogger {
  public:
    Logger(LogLevel logLevel);
    ~Logger() = default;

    void log(LogLevel level, const char* format, va_list args) const override;

  private:
    LogLevel m_logLevel;
};
#endif // LOGGER_H
