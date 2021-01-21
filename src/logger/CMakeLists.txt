set(LIB_NAME "swarmus-connect-logger")
set(LIB_ALIAS "SwarmUs::HiveConnect::Logger")

set(LIB_SOURCES
    src/Logger.cpp
)

set(LIB_HEADERS
    include/logger/ILogger.h
    include/logger/Logger.h
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
        SwarmUs::HiveConnect::BSP
        SwarmUs::HiveConnect::FreeRTOS
        SwarmUs::HiveConnect::TaskManager
)

if(ENABLE_TESTS)
    add_subdirectory(tests)
endif()

