#include "logger/LoggerContainer.h"
#include <bsp/Container.h>

Logger& LoggerContainer::getLogger() {
    // TODO: Inject log level, from bsp maybe? TBD
    static Logger s_logger = Logger(LogLevel::Info, BspContainer::getUserInterface());
    return s_logger;
}
