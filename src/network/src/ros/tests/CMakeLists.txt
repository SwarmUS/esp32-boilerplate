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

if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)

    ## Broadcast Test ##
    add_rostest_gtest(broadcast_test_node launch/broadcast_tests.launch BroadcastTests.cpp)
    target_link_libraries(broadcast_test_node
            SwarmUS::HiveConnect::Network::Platform
            ${catkin_LIBRARIES}
            gmock
            gtest)
    target_include_directories(broadcast_test_node
            PRIVATE
            mocks/
            ${catkin_INCLUDE_DIRS}
            )
    add_dependencies(broadcast_test_node
            communication_broker
            ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS}
            )

    ## add other tests here
endif()
