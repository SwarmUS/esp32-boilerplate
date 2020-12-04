set(LIB_NAME "stub_esp32")

set(LIB_SOURCES
        src/system_api.c
        src/cpu_start.c
)

add_library(${LIB_NAME} SHARED ${LIB_SOURCES})

target_include_directories(stub_esp32 PUBLIC include)
add_library(SwarmUs::esp32 ALIAS ${LIB_NAME})