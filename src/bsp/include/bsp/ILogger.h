#ifndef ILOGGER_H
#define ILOGGER_H

#include <cstdarg>
/**
 * @brief Log levels available, from lowest to highest criticality
 */
enum LogLevel { INFO = 0, DEBUG, WARNING, ERROR };

class ILogger {
  public:
    virtual ~ILogger() = default;

    /**
     * @brief Logging function to log a statement with a corresponding LogLevel.
     *        Follows the same syntax as printf after supplying LogLevel.
     * @param level The corresponding level of the statement to be logged
     * @param format Formatting string for logging statement
     * @param arg List of arguments for formatting string
     */
    virtual void log(LogLevel level, const char* format, va_list arg) const = 0;
};

#endif // ILOGGER_H
