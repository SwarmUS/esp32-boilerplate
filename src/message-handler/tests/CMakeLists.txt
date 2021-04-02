set(TEST_NAME test-message-handler)

set(TEST_SOURCES
        MessageSenderUnitTests.cpp
        NetworkApiHandlerUnitTests.cpp
        MessageDispatcherUnitTests.cpp
        )

set(TEST_HEADERS
        mocks/BSPMock.h
        mocks/CircularQueueInterfaceMock.h
        mocks/HiveMindHostDeserializerInterfaceMock.h
        mocks/HiveMindHostSerializerInterfaceMock.h
        mocks/LoggerInterfaceMock.h
        mocks/NetworkAPIHandlerMock.h
        mocks/NetworkManagerMock.h

        DTOMatchers.h
        )

add_executable(${TEST_NAME} ${TEST_SOURCES} ${TEST_HEADERS})

target_link_libraries(${TEST_NAME}
        PUBLIC
        gtest
        gtest_main
        gmock

        SwarmUS::HiveConnect::MessageHandler
        )

add_test(NAME ${TEST_NAME}
        COMMAND ${TEST_NAME})
