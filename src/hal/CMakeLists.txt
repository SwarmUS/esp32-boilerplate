set(THIS hal)

set(SOURCES
    hal.cpp
)

add_library(${THIS} STATIC ${SOURCES})

#including headers
target_include_directories(${THIS} PUBLIC ${PROJECT_SOURCE_DIR}/include)
#allows to include "hal.hpp" instead of "hal/hal.hpp"
target_include_directories(${THIS} PUBLIC ${PROJECT_SOURCE_DIR}/include/${THIS})

# The following line is very practical:
# it will allow you to automatically add the correct include directories with "target_link_libraries"
target_include_directories(${THIS} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})