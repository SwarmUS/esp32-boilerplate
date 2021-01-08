#ifndef FACTORY_H
#define FACTORY_H

#include <memory>
#include "IBSP.h"

namespace BspFactory {

    /**
     * @brief Returns the BSP instance for the application
     */
        std::shared_ptr<IBSP> getBSP();
}

#endif //FACTORY_H
