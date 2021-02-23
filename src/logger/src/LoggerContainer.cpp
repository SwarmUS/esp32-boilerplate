#include "logger/LoggerContainer.h"
#include "bsp/Container.h"

Logger& LoggerContainer::getLogger() {
    static Logger s_logger =
        Logger(LogLevel::Info, BspContainer::getUserInterface());
    return s_logger;
}
