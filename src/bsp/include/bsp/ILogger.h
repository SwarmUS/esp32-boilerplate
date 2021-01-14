#ifndef ILOGGER_H
#define ILOGGER_H

#include <cstdarg>

enum LogLevel {
    INFO,
    DEBUG,
    WARNING,
    ERROR
};

class ILogger {
  public:
    virtual ~ILogger() = default;

    virtual void log(LogLevel level, const char * format, ...) const = 0 ;
};

#endif // ILOGGER_H
