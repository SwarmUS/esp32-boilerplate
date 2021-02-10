set(LIB_NAME "swarmus-hive_connect-bsp-common")
set(LIB_ALIAS "SwarmUS::HiveConnect::BSP::Common")

set(LIB_SOURCES
    src/CRC.cpp
        )

set(LIB_HEADERS
        include/common/CRC.h
        )

add_library(${LIB_NAME} STATIC ${LIB_HEADERS} ${LIB_SOURCES})
add_library(${LIB_ALIAS} ALIAS ${LIB_NAME})

target_include_directories(${LIB_NAME}
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        include
        )

target_link_libraries(${LIB_NAME}
        PUBLIC
        )

if(ENABLE_TESTS)
    add_subdirectory(tests)
endif()

