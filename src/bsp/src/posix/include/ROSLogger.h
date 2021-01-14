#ifndef ROSLOGGER_H
#define ROSLOGGER_H

#include "bsp/ILogger.h"

#define LOGGER_BUFFER_SIZE 512u

class ROSLogger : public ILogger {
  public:
    ROSLogger(LogLevel logLevel);
    ~ROSLogger() = default;

    void log(LogLevel level, const char * format, ... ) const override;

  private:
    LogLevel m_logLevel;

};

#endif // ROSLOGGER_H
