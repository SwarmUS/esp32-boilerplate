cmake_minimum_required(VERSION 3.2)
set(THIS spi_tests)

set(SOURCES
    spi_tests.cpp
)

add_executable(${THIS} ${SOURCES})

#including headers
target_include_directories(${THIS} PUBLIC ${PROJECT_SOURCE_DIR}/include)

#add dependencies here
target_link_libraries(${THIS} PUBLIC
#external
    gtest
    gmock
    gtest_main

#internal code
    spi
    hal
)

#For those who uses ctest
add_test(NAME ${THIS} COMMAND ${THIS})