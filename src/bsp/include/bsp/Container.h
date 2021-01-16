#ifndef FACTORY_H
#define FACTORY_H

#include "IBSP.h"
#include "IUserInterface.h"
#include <memory>

namespace BspContainer {

/**
 * @brief Returns the BSP instance for the application
 */
IBSP& getBSP();

/**
 * @brief Returns the logger instance for the application
 */
IUserInterface& getUserInterface();
} // namespace BspContainer

#endif // FACTORY_H
