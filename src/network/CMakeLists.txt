set(LIB_NAME "swarmus_hive_connect_network")
set(LIB_ALIAS "SwarmUS::HiveConnect::Network")

get_filename_component(NETWORK_CMAKE_DIR ${CMAKE_CURRENT_LIST_FILE} DIRECTORY)

add_subdirectory(src)

set(LIB_HEADERS
    include/IAbstractNetworkManager.h
    include/INetworkOutputStream.h
    include/INetworkInputStream.h
    include/INetworkBroadcast.h
    include/NetworkContainer.h
)

add_library(${LIB_NAME} STATIC ${LIB_HEADERS})
add_library(${LIB_ALIAS} ALIAS ${LIB_NAME})

target_include_directories(${LIB_NAME}
        PUBLIC
        include
        ${CMAKE_CURRENT_SOURCE_DIR}/include>
        PRIVATE
        src
        )

target_link_libraries(${LIB_NAME}
        PUBLIC
        SwarmUS::HiveConnect::Network::Platform
        protobuf-nanopb-static
        SwarmUS::Propolis::Pheromones::Common
        SwarmUS::HiveConnect::Logger
        )

set_target_properties(${LIB_NAME} PROPERTIES LINKER_LANGUAGE CXX)


