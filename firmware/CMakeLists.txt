cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(intelliVibe VERSION 1.0.0)

target_include_directories(app PRIVATE src/drivers/)
target_include_directories(app PRIVATE src/)
target_include_directories(app PRIVATE src/application/includes/)
target_include_directories(app PRIVATE src/DataProcessing/)

add_subdirectory(src/drivers)
add_subdirectory(src/DataProcessing)

target_sources(app PRIVATE
    src/main.c
    src/application/src/UserConfig.c
)
