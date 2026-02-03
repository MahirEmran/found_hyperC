cmake_minimum_required(VERSION 3.21)

project(hc-compress VERSION 0.1.0 LANGUAGES C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

add_library(hc-compress-lib STATIC
    src/ccsds123_core.c
    src/ccsds123_utils.c
    src/ccsds123_io.c
    src/hc_compress_api.c
)
target_compile_definitions(hc-compress-lib PRIVATE UNIT_TEST)
target_include_directories(hc-compress-lib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_executable(hc-compress
    src/ccsds123_cli.c
)
target_link_libraries(hc-compress PRIVATE hc-compress-lib)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/hybrid_encoder_tables")
    add_custom_command(TARGET hc-compress POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
                "${CMAKE_CURRENT_SOURCE_DIR}/hybrid_encoder_tables"
                "$<TARGET_FILE_DIR:hc-compress>/hybrid_encoder_tables"
    )
    install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/hybrid_encoder_tables"
            DESTINATION .)
endif()

find_library(MATH_LIBRARY m)
if(MATH_LIBRARY)
    target_link_libraries(hc-compress-lib PUBLIC ${MATH_LIBRARY})
endif()
    
    
