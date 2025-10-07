cmake_minimum_required(VERSION 3.22)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(kinematics_test)

target_sources(app PRIVATE src/test_kinematics.c
                           ../../src/robot_state.c
                           ../../src/gait_thread.c
						   ../../src/servos.c
					       ../../src/motors_thread.c)
target_include_directories(app PRIVATE ../../include)

