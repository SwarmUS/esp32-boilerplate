#include "logger/Logger.h"
#include <LockGuard.h>
#include <bsp/IUserInterface.h>
#include <cstdarg>

Logger::Logger(LogLevel level, const IUserInterface& ui) : m_ui(ui), m_mutex(10) {
    m_logLevel = level;
}

LogRet Logger::log(LogLevel level, const char* format, ...) {
    if (level >= m_logLevel) {
        LockGuard lock = LockGuard(m_mutex);
        va_list args;
        va_start(args, format);
        int retValue = -1;
        switch (level) {

        case LogLevel::Debug:
            retValue = m_ui.printDebug(format, args);
            break;
        case LogLevel::Info:
            retValue = m_ui.printInfo(format, args);
            break;
        case LogLevel::Warn:
            retValue = m_ui.printWarning(format, args);
            break;
        case LogLevel::Error:
            retValue = m_ui.printError(format, args);
            break;
        }
        va_end(args);
        if (retValue >= 0) {
            return LogRet::Ok;
        }
        return LogRet::Error;
    }
    return LogRet::LowLevel;
}
