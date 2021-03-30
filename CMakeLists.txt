cmake_minimum_required(VERSION 3.13)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

#Options
option(DISABLE_EXTERNAL_WARNINGS "Disables warnings from external libs (gtest, freertos, etc)" ON)

set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 17)
project(hive_connect C CXX ASM)


# Set appropriate files and flags to link libraries
set(SUPPORTED_TARGETS esp32)
if(COMPILE_ESP)
    # Include for ESP-IDF build system functions
    include($ENV{IDF_PATH}/tools/cmake/idf.cmake)
    # Create idf::esp32 and idf::freertos static libraries
    idf_build_process(esp32
                    # try and trim the build; additional components
                    # will be included as needed based on dependency tree
                    # epstool_py always required
                    COMPONENTS esp32 freertos esptool_py driver nvs_flash
                    SDKCONFIG ${CMAKE_CURRENT_SOURCE_DIR}/sdkconfig
                    BUILD_DIR ${CMAKE_BINARY_DIR})
    set(EXECUTABLE_NAME "${CMAKE_PROJECT_NAME}.elf")
else()
    set(EXECUTABLE_NAME ${CMAKE_PROJECT_NAME})
    message("Compiling locally")
    set(ENABLE_TESTS ON)
    #ROS dependencies
    include(ros/generators.cmake)
    include(catkin/common)
endif()


add_executable(${EXECUTABLE_NAME} src/main.cpp) # Needs to be in the root of the project as it is expected by esp-idf

# Link the libraries to the executable, depending on alias
target_link_libraries(${EXECUTABLE_NAME}
        PUBLIC
        SwarmUS::HiveConnect::OS
        SwarmUS::HiveConnect::BSP
        SwarmUS::HiveConnect::Logger
        SwarmUS::HiveConnect::Network
        SwarmUS::HiveConnect::MessageHandler
)


if(COMPILE_ESP)
    # Register the executable for idf, needs to be done after linking
    idf_build_executable(${EXECUTABLE_NAME})
    # Create target to flash via openocd with JTAG interface
    add_custom_target(
            hive_connect_flash
            COMMAND openocd -f ${CMAKE_CURRENT_SOURCE_DIR}/tools/openocd/adafruit-esp.cfg -c "program_esp ${CMAKE_BINARY_DIR}/${CMAKE_PROJECT_NAME}.bin 0x10000 verify exit"
    )
else()
    #Target to launch local executable
    add_custom_target(
            hive_connect_run
            COMMAND $<TARGET_FILE:${EXECUTABLE_NAME}>
    )


endif()

# Testing
if(ENABLE_TESTS)
    include(googletest/common)
    enable_testing()

    if(NOT TARGET gtest)
        googletest_get_populate()
    endif()

    if(DISABLE_EXTERNAL_WARNINGS)
        googletest_disable_warnings()
    endif()
endif()

# Include sources
include(propolis/common)
add_subdirectory(src)

#Additional tooling include
include(doxygen/doxygen)
include(clang-tools/clang-tidy)
include(clang-tools/clang-format)