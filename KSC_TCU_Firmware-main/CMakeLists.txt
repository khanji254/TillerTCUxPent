# Root CMake for the KSC TCU firmware test workspace (ESP-IDF).
# PlatformIO builds this directly.
cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(tcu_tests)
