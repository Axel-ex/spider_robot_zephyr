cmake_minimum_required(VERSION 3.22)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(BOARD esp32)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project (hello)

include_directories(app PRIVATE include/)

zephyr_include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Tell Zephyr about your custom Kconfig file
set(KCONFIG_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/Kconfig.secrets)

target_sources(app PRIVATE src/main.c src/servos.c src/robot_state.c src/tcp_server_thread.c src/motors_thread.c)
