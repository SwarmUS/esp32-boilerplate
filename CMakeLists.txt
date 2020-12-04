cmake_minimum_required(VERSION 3.5)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

#Options
option(DISABLE_EXTERNAL_WARNINGS "Disables warnings from exernal libs (gtest, freertos, etc)" ON)

project(esp32-boilerplate C CXX ASM)



set(SUPPORTED_TARGETS esp32)
if("${TARGET}" STREQUAL "esp32")
    # Include for ESP-IDF build system functions
    include($ENV{IDF_PATH}/tools/cmake/idf.cmake)
    # Create idf::esp32 and idf::freertos static libraries
    idf_build_process(esp32
                    # try and trim the build; additional components
                    # will be included as needed based on dependency tree
                    # epstool_py always required
                    COMPONENTS esp32 freertos esptool_py
                    SDKCONFIG ${CMAKE_BINARY_DIR}/sdkconfig
                    BUILD_DIR ${CMAKE_BINARY_DIR})
    set(ALIAS_PREFIX "idf")
else()

    set(ALIAS_PREFIX "SwarmUs")
    set(ENABLE_TESTS ON)
endif()
add_subdirectory(src)
#set executable name
set(elf_file ${CMAKE_PROJECT_NAME}.elf)

add_executable(${elf_file} src/main.c) # Needs to be in the root of the project as it is expected by esp-idf

# Link the libraries to the executable, depending on alias
target_link_libraries(${elf_file} ${ALIAS_PREFIX}::esp32 ${ALIAS_PREFIX}::freertos ${ALIAS_PREFIX}::spi_flash)

if("${TARGET}" STREQUAL "esp32")
    # Register the executable for idf, needs to be done after linking
    idf_build_executable(${elf_file})
endif()

# Testing
if(ENABLE_TESTS)
    include(googletest/common)
    enable_testing()
    googletest_get_populate()
    add_subdirectory(tests)
endif()