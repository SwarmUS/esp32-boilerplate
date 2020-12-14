set(LIB_NAME stub_spi_flash)

add_library(${LIB_NAME} STATIC
    src/flash_ops.c
)

target_include_directories(${LIB_NAME} PUBLIC include)
add_library(SwarmUs::spi_flash ALIAS ${LIB_NAME})