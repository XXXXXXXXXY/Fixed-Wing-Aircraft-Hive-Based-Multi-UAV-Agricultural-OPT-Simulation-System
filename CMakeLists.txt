cmake_minimum_required(VERSION 3.16)
project(scout_opt_c C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

add_executable(scout_opt
    c_src/main.c
    c_src/scout_opt.c
    c_src/config_loader.c
    c_src/sitl_bridge.c
    c_src/diagnostics.c
    c_src/visual_export.c
)

target_include_directories(scout_opt PRIVATE c_include)

if (MSVC)
    target_compile_options(scout_opt PRIVATE /W4)
else()
    target_compile_options(scout_opt PRIVATE -Wall -Wextra -pedantic)
    target_link_libraries(scout_opt PRIVATE m)
endif()
