# Copyright (c) 2026 Space Cubics Inc.
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

if(NOT DEFINED BOARD)
    set(BOARD scobc_v1/miv)
endif()

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(scobc-v1-safety-processor)

add_subdirectory(drivers/i2c)

target_sources(app PRIVATE
    src/main.c
    src/system.c
    src/mppc.c
    src/sensor.c
    src/ipc.c
    )
