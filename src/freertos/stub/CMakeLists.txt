set(LIB_NAME "stub_freertos")

include(freertos/common)
freertos_fetch_kernel()

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
find_package(FreeRTOS COMPONENTS POSIX REQUIRED)

target_compile_options(FreeRTOS::POSIX INTERFACE -fPIC)


set(LIB_HEADERS
    include/freertos/
)

set(LIB_SOURCES
    src/platform_freertos_hooks.c
)

add_library(${LIB_NAME} STATIC
        ${LIB_SOURCES}
)

target_link_libraries(${LIB_NAME}
    PUBLIC
        Threads::Threads
        FreeRTOS::POSIX
        FreeRTOS::Coroutine
        FreeRTOS::EventGroups
        FreeRTOS::StreamBuffer
        FreeRTOS::Timers
        FreeRTOS::Heap::3
)


target_include_directories(${LIB_NAME}
        PUBLIC
        ${LIB_HEADERS}

        # Allows freertos to build properly
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/freertos>
        $<INSTALL_INTERFACE:include>
        ${FREERTOS_KERNEL_PATH}/include
        ${FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/Posix)


add_library(SwarmUs::freertos ALIAS ${LIB_NAME})

# Removing warnings from freertos compilation on executable target
if (DISABLE_EXTERNAL_WARNINGS)
    target_compile_options(${LIB_NAME} PRIVATE -w)
    set_target_properties(${LIB_NAME} PROPERTIES CXX_CLANG_TIDY "" )
    set_target_properties(${LIB_NAME} PROPERTIES C_CLANG_TIDY "" )
endif()