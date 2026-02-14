# NexSense M5-DoorSensor - Phase 1 Core Firmware
# Target: M5StickC Plus 2 (ESP32-PICO-V3-02)

cmake_minimum_required(VERSION 3.16)

set(EXTRA_COMPONENT_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/components
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(nexsense-doorsensor)
