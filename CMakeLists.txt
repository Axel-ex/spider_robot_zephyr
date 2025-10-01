cmake_minimum_required(VERSION 3.22)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(BOARD esp32)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project (hello)

include_directories(app PRIVATE include/)

target_sources(app PRIVATE src/main.c src/servos.c src/kinematics.c src/tcp_server.c)
