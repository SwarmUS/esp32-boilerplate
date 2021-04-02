set(LIB_NAME "swarmus-hiveconnect-network-common")
set(LIB_ALIAS "SwarmUS::HiveConnect::Network::Common")

set(LIB_HEADERS
    include/AbstractNetworkManager.h
)

set(LIB_SOURCES
    src/AbstractNetworkManager.cpp
)


add_library(${LIB_NAME} STATIC ${LIB_HEADERS} ${LIB_SOURCES})
add_library(${LIB_ALIAS} ALIAS ${LIB_NAME})

target_include_directories(${LIB_NAME}
        PUBLIC
        include
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        ${NETWORK_CMAKE_DIR}/include
        PRIVATE
        )

target_link_libraries(${LIB_NAME}
        PUBLIC
        SwarmUS::HiveConnect::Logger
        SwarmUS::Propolis::Cpp::Common
)