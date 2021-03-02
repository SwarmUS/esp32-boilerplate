set(LIB_NAME "swarmus-hiveconnect-network")
set(LIB_ALIAS "SwarmUS::HiveConnect::Network")

set(LIB_SOURCES
        src/NetworkConfig.cpp
        src/NetworkManager.cpp
        )

set(LIB_HEADERS
        include/NetworkConfig.h
        include/NetworkContainer.h
        include/NetworkManager.h
        )

include(DefaultNetworkConfig.cmake)
configure_file(include/DefaultNetworkConfig.in.h include/DefaultNetworkConfig.h)

add_library(${LIB_NAME} STATIC ${LIB_HEADERS} ${LIB_SOURCES})
add_library(${LIB_ALIAS} ALIAS ${LIB_NAME})

target_include_directories(${LIB_NAME}
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>

        #Include the generated DefaultNetworkConfig.h
        ${CMAKE_CURRENT_BINARY_DIR}/include
        )

target_link_libraries(${LIB_NAME}
        PUBLIC
        SwarmUS::HiveConnect::OS
        SwarmUS::HiveConnect::Logger
        )

