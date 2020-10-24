cmake_minimum_required(VERSION 3.2)

set(THIS Template_CXX)

project(${THIS} C CXX)

set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 11)

enable_testing()

add_subdirectory(src)
add_subdirectory(tests)
