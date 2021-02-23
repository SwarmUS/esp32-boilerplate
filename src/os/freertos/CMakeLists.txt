set(LIB_NAME "swarmus-hiveconnect-os")
set(LIB_ALIAS "SwarmUS::HiveConnect::OS")

add_library(${LIB_NAME} STATIC)
add_library(${LIB_ALIAS} ALIAS ${LIB_NAME})

#Hack to match the same include path as posix
get_target_property(FREERTOS_SOURCE_DIR idf::freertos SOURCE_DIR)
target_include_directories(${LIB_NAME}
        PUBLIC
        ${FREERTOS_SOURCE_DIR}/include/freertos
        ${FREERTOS_SOURCE_DIR}/xtensa/include/freertos
        ${PROPOLIS_OS_INCLUDES}
        )

target_link_libraries(${LIB_NAME}
        INTERFACE
        idf::freertos
        PRIVATE
        idf::esp_common
        PUBLIC
        SwarmUS::Propolis::OS::FreeRTOS
        )