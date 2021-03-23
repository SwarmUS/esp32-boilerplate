set(LIB_NAME "swarmus_hive_connect_bsp_esp")
set(LIB_ALIAS "SwarmUS::HiveConnect::BSP::Platform")

set(LIB_SOURCES
        src/BSP.cpp
        src/Container.cpp
        src/UserInterface.cpp
        src/SpiStmMock.cpp
        hal/spi_callbacks.c)

set(LIB_PRIVATE_HEADERS
        include/BSP.h
        include/UserInterface.h
        hal/pin_map.h
        hal/spi_callbacks.h)

add_library(${LIB_NAME} ${LIB_SOURCES} ${LIB_PRIVATE_HEADERS})
add_library(${LIB_ALIAS} ALIAS ${LIB_NAME})

target_include_directories(${LIB_NAME}
        PRIVATE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        ${BSP_CMAKE_DIR}/include
        include
        )

target_link_libraries(${LIB_NAME}
        PUBLIC
        # Defined libraries
        SwarmUS::HiveConnect::OS
        SwarmUS::HiveConnect::Network
        PRIVATE
        idf::esp32
        idf::driver
        idf::nvs_flash
        SwarmUS::Propolis::C::Common
)