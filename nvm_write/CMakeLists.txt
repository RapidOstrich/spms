cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
zephyr_include_directories(include)

project(nvm_write)

target_sources(app PRIVATE
    src/main.c
    src/sen0546.c
    src/log_store.c
    src/button_dump.c
)
