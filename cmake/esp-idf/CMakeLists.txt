cmake_minimum_required(VERSION 3.2)

# Download and unpack esp-idf at configure time
configure_file(CMakeLists.txt.in esp-idf-download/CMakeLists.txt)
set(patate ${CMAKE_CURRENT_BINARY_DIR}/esp-idf-src/)
message("idf-path set to: ${patate}")

find_package(PythonInterp 2.7 REQUIRED)


#include(${IDF_PATH}/tools/cmake/project.cmake)
execute_process(COMMAND ${CMAKE_COMMAND} -DPYTHON_LIBRARIES="$ENV{IDF_PYTHON_ENV_PATH}" .
        RESULT_VARIABLE result
       WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/esp-idf-download )
if(result)
    message(FATAL_ERROR "CMake step for esp-idf failed: ${result}")
endif()
execute_process(COMMAND ${CMAKE_COMMAND} --build .
        RESULT_VARIABLE result
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/esp-idf-download )
if(result)
    message(FATAL_ERROR "Build step for esp-idf failed: ${result}")
endif()

