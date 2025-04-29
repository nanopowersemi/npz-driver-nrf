# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(nPZero_nRF52)

add_subdirectory(nPZero_Driver/Src)
add_subdirectory(nPZero_internal)

target_sources(app PRIVATE src/main.c)
target_sources(app PRIVATE src/np0_ble.c)

target_include_directories(app PRIVATE nPZero_Driver/Inc)