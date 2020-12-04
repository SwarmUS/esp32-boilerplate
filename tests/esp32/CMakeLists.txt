set(TEST_NAME test_stub_esp32)

set(TEST_SOURCES
        system_api.cpp
)

add_executable(${TEST_NAME} ${TEST_SOURCES})

target_link_libraries(${TEST_NAME}
    gtest
    gtest_main
    gmock
    stub_esp32
)

add_test(NAME ${TEST_NAME}
        COMMAND ${TEST_NAME})