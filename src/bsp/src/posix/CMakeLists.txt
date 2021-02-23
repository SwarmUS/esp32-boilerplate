set(LIB_NAME "swarmus_hive_connect_bsp_posix")
set(LIB_ALIAS "SwarmUS::HiveConnect::BSP::Platform")

set(LIB_SOURCES
        src/BSP.cpp
        src/Container.cpp
        src/System.cpp
        src/UserInterface.cpp
)

set(LIB_PRIVATE_HEADERS
        include/BSP.h
        include/System.h
        include/UserInterface.h
        include/SpiStm.h
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
        SwarmUS::HiveConnect::OS
        PRIVATE
        ${catkin_LIBRARIES}
)

set_target_properties(${LIB_NAME} PROPERTIES LINKER_LANGUAGE CXX)
