cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(spms_ble)

zephyr_include_directories(include)

target_sources(app PRIVATE
    src/main.c
    src/log_store.c
    src/sen0114.c
    src/sen0390.c
    src/sen0546.c
    src/spms_ble.c
    src/spms_sensors.c
)
