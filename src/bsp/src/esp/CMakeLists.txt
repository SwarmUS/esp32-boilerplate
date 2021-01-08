set(LIB_NAME "swarmus-bsp-esp")
set(LIB_ALIAS "SwarmUs::BSP::Platform")

set(LIB_SOURCES
        src/BSP.cpp
        src/Factory.cpp
        )

set(LIB_PRIVATE_HEADERS
        include/BSP.h
        )

add_library(${LIB_NAME} ${LIB_SOURCES} ${LIB_PRIVATE_HEADERS})
add_library(${LIB_ALIAS} ALIAS ${LIB_NAME})

target_include_directories(${LIB_NAME}
        PRIVATE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        ${BSP_CMAKE_DIR}/include
        include
        )

target_link_libraries(${LIB_NAME}
        PUBLIC
        # Defined libraries
        idf::freertos
)