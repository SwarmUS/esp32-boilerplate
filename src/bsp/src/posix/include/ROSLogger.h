#ifndef ROSLOGGER_H
#define ROSLOGGER_H

#include "bsp/ILogger.h"
#include "ros/ros.h"

#define LOGGER_BUFFER_SIZE 512u

class ROSLogger : public ILogger {
  public:
    ROSLogger(LogLevel logLevel);
    ~ROSLogger() = default;

    void log(LogLevel level, const char* format, va_list arg) const override;

  private:
    LogLevel m_logLevel;
    static void info(char* buffer) { ROS_INFO("%s", buffer); }
    static void debug(char* buffer) { ROS_DEBUG("%s", buffer); }
    static void warn(char* buffer) { ROS_WARN("%s", buffer); }
    static void error(char* buffer) { ROS_ERROR("%s", buffer); }
};

#endif // ROSLOGGER_H
