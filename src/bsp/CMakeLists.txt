set(LIB_NAME "swarmus-bsp")
set(LIB_ALIAS "SwarmUs::HiveConnect::BSP")

get_filename_component(BSP_CMAKE_DIR ${CMAKE_CURRENT_LIST_FILE} DIRECTORY)

add_subdirectory(src)

set(LIB_HEADERS
        include/bsp/IBSP.h
        include/bsp/Factory.h
)

add_library(${LIB_NAME} STATIC ${LIB_HEADERS})
add_library(${LIB_ALIAS} ALIAS ${LIB_NAME})

target_include_directories(${LIB_NAME}
        PUBLIC
        include
        ${CMAKE_CURRENT_SOURCE_DIR}/include>
        PRIVATE
        src
        )

target_link_libraries(${LIB_NAME}
        PRIVATE
        SwarmUs::BSP::Platform
)

set_target_properties(${LIB_NAME} PROPERTIES LINKER_LANGUAGE CXX)


