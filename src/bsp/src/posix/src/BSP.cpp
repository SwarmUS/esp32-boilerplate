#include "BSP.h"
#include "BaseTask.h"
#include <thread>

constexpr uint8_t gc_spinnerCores = 1;

BSP::~BSP() = default;


BSP::BSP(const ros::NodeHandle& nodeHandle, const int loopRate) : m_loopRate(loopRate), m_spinner(gc_spinnerCores)  {
    m_rosNodeHandle = std::make_shared<ros::NodeHandle>(nodeHandle);
}

void BSP::initChip() {
    m_spinner.start();
}

ChipInfo BSP::getChipInfo() {
    return ChipInfo{.m_cores = (uint8_t)std::thread::hardware_concurrency(),
                    .m_osType = ChipInfo::SIMULATION};
}

std::shared_ptr<ros::NodeHandle> BSP::getNodeHandle() { return m_rosNodeHandle; }
