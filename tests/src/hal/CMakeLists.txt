cmake_minimum_required(VERSION 3.2)
set(THIS hal_tests)

set(SOURCES
    hal_tests.cpp
)

add_executable(${THIS} ${SOURCES})

#including headers
target_include_directories(${THIS} PUBLIC ${PROJECT_SOURCE_DIR}/include)

#add dependencies here
target_link_libraries(${THIS} PUBLIC
#external
    gtest
#    gmock
    gtest_main

#internal code
    hal
)


#For those who uses ctest
add_test(NAME ${THIS} COMMAND ${THIS})