set(TEST_NAME test-network)

set(TEST_SOURCES
    UnicastCommunicationUnitTests.cpp
)

set(TEST_HEADERS
    mocks/NetworkManagerMock.h
    mocks/LoggerInterfaceMock.h
        )

add_executable(${TEST_NAME} ${TEST_SOURCES} ${TEST_HEADERS})

target_link_libraries(${TEST_NAME}
        PUBLIC
        gtest
        gtest_main
        gmock

        SwarmUS::HiveConnect::Network::Platform
        )

add_test(NAME ${TEST_NAME}
        COMMAND ${TEST_NAME})
