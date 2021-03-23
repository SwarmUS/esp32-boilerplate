set(LIB_NAME "swarmus_hive_connect_bsp")
set(LIB_ALIAS "SwarmUS::HiveConnect::BSP")

get_filename_component(BSP_CMAKE_DIR ${CMAKE_CURRENT_LIST_FILE} DIRECTORY)


add_subdirectory(src)

set(LIB_HEADERS
        include/bsp/IBSP.h
        include/bsp/Container.h
        include/bsp/IUserInterface.h
        include/bsp/ISpiStm.h)

add_library(${LIB_NAME} STATIC ${LIB_HEADERS})
add_library(${LIB_ALIAS} ALIAS ${LIB_NAME})

target_include_directories(${LIB_NAME}
        PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include>
        PRIVATE
            src
        )

target_link_libraries(${LIB_NAME}
        PUBLIC
        SwarmUS::Propolis::Pheromones::Common
        PRIVATE
        SwarmUS::HiveConnect::BSP::Platform
)

set_target_properties(${LIB_NAME} PROPERTIES LINKER_LANGUAGE CXX)


