set(LIB_NAME "stub_freertos")

add_library(${LIB_NAME} STATIC
        src/task.c
)


target_include_directories(${LIB_NAME} PUBLIC include)
add_library(SwarmUs::freertos ALIAS ${LIB_NAME})