# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(chinese_fingerprint_sensor)

# add_subdirectory(inc)
include_directories(../m080r_driver_module/drivers/sensor/m080r)
target_sources(app PRIVATE src/main.c)
