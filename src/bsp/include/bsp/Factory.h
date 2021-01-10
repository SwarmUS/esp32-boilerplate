#ifndef FACTORY_H
#define FACTORY_H

#include "IBSP.h"
#include <memory>

namespace BspFactory {

/**
 * @brief Returns the BSP instance for the application
 */
std::shared_ptr<IBSP> getBSP();
} // namespace BspFactory

#endif // FACTORY_H
