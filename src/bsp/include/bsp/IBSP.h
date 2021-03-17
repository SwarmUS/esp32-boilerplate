#ifndef IBSP_H
#define IBSP_H

#include <cstdint>

/**
 * @brief Structure containing basic system information
 */
struct ChipInfo {
    uint8_t m_cores;
    enum OS_TYPE { ESP, SIMULATION } m_osType;
};
class IBSP {
  public:
    virtual ~IBSP() = default;

    /**
     * @brief Initialise chip for usage. Should be called within the main.
     */
    virtual void initChip() = 0;

    /**
     * @brief Get the chip from the BSP class
     * @return The chip info for the BSP initiated
     */
    virtual ChipInfo getChipInfo() = 0;

    /**
     * @brief Get the uuid of the system
     * @return The uuid value
     */
    virtual uint16_t getUUID() = 0;

    /**
     * @brief Set the uuid of the system
     */
    virtual void setUUID(uint16_t uuid) = 0;
};

#endif // IBSP_H
