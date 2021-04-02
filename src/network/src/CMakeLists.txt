add_subdirectory(common)

if (COMPILE_ESP)
    add_subdirectory(esp)
else()
    add_subdirectory(ros)
endif()
