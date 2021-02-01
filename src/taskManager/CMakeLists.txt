set(LIB_NAME "swarmus-hive_connect-task_manager")
set(LIB_ALIAS "SwarmUS::HiveConnect::TaskManager")

set(LIB_SOURCES
        src/AbstractTask.cpp
)

set(LIB_HEADERS
        include/taskManager/AbstractTask.h
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
        SwarmUS::HiveConnect::FreeRTOS
)

