#ifndef IBSP_H
#define IBSP_H

#include "ILogger.h"
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
     * @brief Looging function. Follows same syntax as printf after the LogLevel
     * @param level The corresponding level of the statement to be logged
     * @param format Formatting string for logging statement
     * @param ... printf-style input arguments for formatting string
     */
    virtual void log(LogLevel level, const char* format, ...) = 0;
};

#endif // IBSP_H
