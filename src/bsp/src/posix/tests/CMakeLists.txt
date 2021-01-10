set(TEST_NAME tests_bsp_posix)

set(TEST_SOURCES
        BSPTests.cpp
)

set(TEST_HEADERS

)

add_executable(${TEST_NAME} ${TEST_SOURCES} ${TEST_HEADERS})
target_include_directories(${TEST_NAME} SYSTEM PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(${TEST_NAME}
    PUBLIC
    gtest
    gtest_main
    gmock
    SwarmUs::HiveConnect::BSP
    SwarmUs::BSP::Platform
    ${catkin_LIBRARIES}
)

add_test(NAME ${TEST_NAME}
        COMMAND ${TEST_NAME})