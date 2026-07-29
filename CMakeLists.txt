# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(nPZero_nRF52)

# NOTE: previously used add_subdirectory(nPZero_Driver/Src) with two bugs:
#   1) wrong case ("nPZero_Driver" vs actual folder "nPZero_driver")
#   2) nPZero_driver/Src has no CMakeLists.txt of its own (it is a plain vendor
#      driver submodule, not a Zephyr module), so add_subdirectory cannot work
#      here regardless of case. Sources are added directly instead.
target_sources(app PRIVATE
    src/main.c
    src/npz_temp_sensor.c
    src/npz_hal.c
    nPZero_driver/Src/npz.c
    nPZero_driver/Src/npz_device_control.c
    nPZero_driver/Src/npz_logs.c
)

target_include_directories(app PRIVATE nPZero_driver/Inc)
