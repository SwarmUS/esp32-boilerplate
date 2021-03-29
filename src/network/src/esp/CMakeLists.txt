set(LIB_NAME "swarmus-hiveconnect-network-esp")
set(LIB_ALIAS "SwarmUS::HiveConnect::Network::Platform")

set(LIB_SOURCES
        src/NetworkConfig.cpp
        src/NetworkManager.cpp
        src/NetworkContainer.cpp
        src/SocketFactory.cpp
        src/TCPServer.cpp
        src/TCPClient.cpp
        )

set(LIB_HEADERS

        include/NetworkConfig.h
        include/NetworkManager.h
        include/SocketFactory.h
        include/TCPClient.h
        include/TCPServer.h
        )

include(DefaultNetworkConfig.cmake)
configure_file(include/DefaultNetworkConfig.in.h include/DefaultNetworkConfig.h)

add_library(${LIB_NAME} STATIC ${LIB_HEADERS} ${LIB_SOURCES})
add_library(${LIB_ALIAS} ALIAS ${LIB_NAME})

target_include_directories(${LIB_NAME}
        PUBLIC
        include
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        ${NETWORK_CMAKE_DIR}/include

        #Include the generated DefaultNetworkConfig.h
        ${CMAKE_CURRENT_BINARY_DIR}/include

        )

target_link_libraries(${LIB_NAME}
        PUBLIC
        SwarmUS::HiveConnect::OS
        SwarmUS::Propolis::OS::Common
        SwarmUS::HiveConnect::Logger
        SwarmUS::Propolis::C::Common
        SwarmUS::Propolis::Pheromones::Common
        PRIVATE
        idf::lwip
        )

