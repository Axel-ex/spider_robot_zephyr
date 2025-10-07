cmake_minimum_required(VERSION 3.22)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(BOARD esp32)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(SpiderBot)

set(SRCS
    src/main.c
    src/servos.c
    src/gait.c
    src/robot_state.c
    src/threads/tcp_server_thread.c
    src/threads/motors_thread.c
    src/threads/gait_thread.c)

include_directories(app PRIVATE include/)

zephyr_include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# set(KCONFIG_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/Kconfig)

target_sources(app PRIVATE ${SRCS})
