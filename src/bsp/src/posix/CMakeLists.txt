set(LIB_NAME "swarmus_hive_connect_bsp_posix")
set(LIB_ALIAS "SwarmUs::HiveConnect::BSP::Platform")

set(LIB_SOURCES
        src/BSP.cpp
        src/Factory.cpp
)

set(LIB_PRIVATE_HEADERS
        include/BSP.h
)

add_library(${LIB_NAME} ${LIB_SOURCES} ${LIB_PRIVATE_HEADERS})
add_library(${LIB_ALIAS} ALIAS ${LIB_NAME})

# Make ROS messages/services available in headers
#add_dependencies(${LIB_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS})

target_include_directories(${LIB_NAME} SYSTEM PRIVATE ${catkin_INCLUDE_DIRS})
target_include_directories(${LIB_NAME}
        PRIVATE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        ${BSP_CMAKE_DIR}/include
        PUBLIC
        include
        )

target_link_libraries(${LIB_NAME}
        PUBLIC
        # Defined libraries
        SwarmUs::HiveConnect::FreeRTOS
        ${catkin_LIBRARIES}
)

set_target_properties(${LIB_NAME} PROPERTIES LINKER_LANGUAGE CXX)

if(ENABLE_TESTS)
    add_subdirectory(tests)
endif()
