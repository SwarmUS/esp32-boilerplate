set(LIB_NAME "swarmus-hiveconnect-message-handler")
set(LIB_ALIAS "SwarmUS::HiveConnect::MessageHandler")

set(LIB_SOURCES
    src/MessageSender.cpp
    src/NetworkAPIHandler.cpp
    src/MessageDispatcher.cpp
        )

set(LIB_HEADERS
    include/message-handler/IMessageDispatcher.h
    include/message-handler/IMessageSender.h
        include/message-handler/INetworkAPIHandler.h

    include/message-handler/MessageDispatcher.h
    include/message-handler/MessageSender.h
    include/message-handler/NetworkAPIHandler.h
        )

add_library(${LIB_NAME} STATIC ${LIB_HEADERS} ${LIB_SOURCES})
add_library(${LIB_ALIAS} ALIAS ${LIB_NAME})

target_include_directories(${LIB_NAME}
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>

        PRIVATE
        include/message-handler
        )

target_link_libraries(${LIB_NAME}
        PUBLIC
        SwarmUS::HiveConnect::Logger
        SwarmUS::Propolis::OS::Common
        SwarmUS::Propolis::Cpp::Common
        SwarmUS::Propolis::Pheromones::HiveMind::Host

        PRIVATE
        SwarmUS::HiveConnect::BSP
        SwarmUS::HiveConnect::OS
        )

if(ENABLE_TESTS)
    add_subdirectory(tests)
endif()
