set(LIB_NAME "swarmus_hive_connect_freertos_posix")
set(LIB_CONFIG_NAME "swarmus_hive_connect_freertos_posix_config")
set(LIB_ALIAS "SwarmUs::HiveConnect::freertos::posix")
set(LIB_CONFIG_ALIAS "SwarmUs::HiveConnect::freertos::posix::config")

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
find_package(FreeRTOS COMPONENTS POSIX REQUIRED)

target_compile_options(FreeRTOS::POSIX INTERFACE -fPIC)

set(LIB_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/platform_freertos_hooks.c
)

add_library(${LIB_NAME} INTERFACE)
add_library(${LIB_ALIAS} ALIAS ${LIB_NAME})
target_sources(${LIB_NAME} INTERFACE ${LIB_SOURCES})

target_link_libraries(${LIB_NAME}
    INTERFACE
        Threads::Threads
        FreeRTOS::POSIX
)

add_library(${LIB_CONFIG_NAME} INTERFACE)
add_library(${LIB_CONFIG_ALIAS} ALIAS ${LIB_CONFIG_NAME})
target_include_directories(${LIB_CONFIG_NAME}
    INTERFACE
        include
        ${FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/Posix
)