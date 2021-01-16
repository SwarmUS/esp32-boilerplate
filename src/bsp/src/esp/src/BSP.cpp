#include "BSP.h"
#include "esp_system.h"

BSP::BSP() = default;
BSP::~BSP() = default;

void BSP::initChip() {}

ChipInfo BSP::getChipInfo() {
    esp_chip_info_t chipInfo;
    esp_chip_info(&chipInfo);

    return (ChipInfo){.m_cores = chipInfo.cores, .m_osType = ChipInfo::ESP};
}

