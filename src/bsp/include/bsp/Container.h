#ifndef FACTORY_H
#define FACTORY_H

#include "IBSP.h"
#include <memory>

namespace BspContainer {

/**
 * @brief Returns the BSP instance for the application
 */
IBSP& getBSP();
} // namespace BspContainer

#endif // FACTORY_H
