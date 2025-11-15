cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(spms_ble)

# So we can do #include "sen0546.h" and #include "log_store.h"
zephyr_include_directories(include)

target_sources(app PRIVATE
    src/main.c
    src/sen0114.c
    src/sen0390.c
    src/sen0546.c
    src/log_store.c
)
