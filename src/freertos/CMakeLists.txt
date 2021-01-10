if(NOT COMPILE_ESP)
    add_subdirectory(posix)
else()
    add_library(swarmus_freertos_esp INTERFACE)

    #Hack to match the same include path as posix
    get_target_property(FREERTOS_SOURCE_DIR idf::freertos SOURCE_DIR)
    target_include_directories(swarmus_freertos_esp
            INTERFACE
            ${FREERTOS_SOURCE_DIR}/include/freertos
            )
    target_link_libraries(swarmus_freertos_esp INTERFACE idf::freertos)
    add_library(SwarmUs::HiveConnect::FreeRTOS ALIAS swarmus_freertos_esp)
endif()