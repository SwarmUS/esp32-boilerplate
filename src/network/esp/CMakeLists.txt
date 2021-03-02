set(LIB_NAME "swarmus-hiveconnect-network")
set(LIB_ALIAS "SwarmUS::HiveConnect::Network")

set(LIB_SOURCES
        src/NetworkConfig.cpp
        )

set(LIB_HEADERS
        include/NetworkConfig.h
        )

add_library(${LIB_NAME} STATIC ${LIB_HEADERS} ${LIB_SOURCES})
add_library(${LIB_ALIAS} ALIAS ${LIB_NAME})

target_include_directories(${LIB_NAME}
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        )

target_link_libraries(${LIB_NAME}
        PUBLIC
        SwarmUS::HiveConnect::BSP
        SwarmUS::HiveConnect::OS
        idf::nvs_flash
        )

