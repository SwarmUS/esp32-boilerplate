set(LIB_NAME "stub_esp32")

set(LIB_SOURCES
        src/system_api.c
        src/cpu_start.cpp
)

add_library(${LIB_NAME} SHARED ${LIB_SOURCES})

# Make ROS messages/services available in headers
#add_dependencies(${LIB_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS})

target_include_directories(${LIB_NAME} SYSTEM PRIVATE ${catkin_INCLUDE_DIRS})
target_include_directories(${LIB_NAME}
        PUBLIC
            include

)

target_link_libraries(${LIB_NAME}
    PUBLIC
        ${catkin_LIBRARIES}
)

add_library(SwarmUs::HiveConnect::esp32 ALIAS ${LIB_NAME})