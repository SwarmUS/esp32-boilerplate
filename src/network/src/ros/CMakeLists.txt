set(LIB_NAME "swarmus-hiveconnect-network-ros")
set(LIB_ALIAS "SwarmUS::HiveConnect::Network::Platform")

set(LIB_HEADERS
        include/NetworkManager.h
        include/NetworkOutputStream.h
        include/NetworkInputStream.h
        include/NetworkBroadcast.h
        )

set(LIB_SOURCES
    src/NetworkContainer.cpp
    src/NetworkInputStream.cpp
    src/NetworkOutputStream.cpp
    src/NetworkBroadcast.cpp
    src/NetworkManager.cpp
)


add_library(${LIB_NAME} STATIC ${LIB_HEADERS} ${LIB_SOURCES})
add_library(${LIB_ALIAS} ALIAS ${LIB_NAME})

# Make ROS messages/services available in headers
add_dependencies(${LIB_NAME} ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})


target_include_directories(${LIB_NAME}
        PUBLIC
        include
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        ${NETWORK_CMAKE_DIR}/include
        PRIVATE
        ${catkin_INCLUDE_DIRS}
        src/broker/config
)

target_link_libraries(${LIB_NAME}
        PUBLIC
        SwarmUS::HiveConnect::OS
        SwarmUS::HiveConnect::Logger
        SwarmUS::Propolis::C::Common
        SwarmUS::Propolis::Cpp::Common
        SwarmUS::Propolis::Pheromones
        SwarmUS::HiveConnect::Network::Common
        PRIVATE
        ${catkin_LIBRARIES}

        )

add_subdirectory(src/broker)

if(ENABLE_TESTS)
    add_subdirectory(tests)
endif()

set_target_properties(${LIB_NAME} PROPERTIES LINKER_LANGUAGE CXX)