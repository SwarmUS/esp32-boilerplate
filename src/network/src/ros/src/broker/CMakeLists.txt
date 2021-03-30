set(BROKER_EXECUTABLE_NAME "communication_broker")

set(BROKER_HEADERS
    CommunicationBroker.h
        )

set(BROKER_SOURCES
    CommunicationBroker.cpp
    main.cpp
        )


add_executable(${BROKER_EXECUTABLE_NAME} ${BROKER_SOURCES})

# Make ROS messages/services available in headers
add_dependencies(${BROKER_EXECUTABLE_NAME} ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})


target_include_directories(${BROKER_EXECUTABLE_NAME}
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        PRIVATE
        ${catkin_INCLUDE_DIRS}
        )

target_link_libraries(${BROKER_EXECUTABLE_NAME}
        PUBLIC
        ${catkin_LIBRARIES}
        )


set_target_properties(${LIB_NAME} PROPERTIES LINKER_LANGUAGE CXX)