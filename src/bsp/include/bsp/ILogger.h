#ifndef ILOGGER_H
#define ILOGGER_H

#include <cstdarg>

enum LogLevel { INFO = 0, DEBUG, WARNING, ERROR };

class ILogger {
  public:
    virtual ~ILogger() = default;

    virtual void log(LogLevel level, const char* format, va_list arg) const = 0;
};

#endif // ILOGGER_H
