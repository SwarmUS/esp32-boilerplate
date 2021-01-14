#include "BSP.h"
#include "esp_system.h"

BSP::BSP(ILogger& logger) { m_logger = &logger; }
BSP::~BSP() = default;

void BSP::initChip() {}

ChipInfo BSP::getChipInfo() {
    esp_chip_info_t chipInfo;
    esp_chip_info(&chipInfo);

    return (ChipInfo){.m_cores = chipInfo.cores, .m_osType = ChipInfo::ESP};
}

void BSP::log(LogLevel level, const char* format, ...) {
    va_list args;
    va_start(args, format);
    m_logger->log(level, format, args);
    va_end(args);
}
