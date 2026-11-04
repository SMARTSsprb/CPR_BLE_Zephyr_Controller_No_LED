# CMakeLists.txt - Zephyr RTOS Build Configuration
# CPR BLE RTOS Controller (No LED Build) for ESP32-C3

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(cpr_ble_zephyr_controller_no_led)

target_include_directories(app PRIVATE src)

target_sources(app PRIVATE
    src/main.c
    src/cpr_protocol.c
    src/ble_service.c
    src/ir_sensor.c
    src/haptic.c
    src/aed_sensor.c
    src/air_sensor.c
    src/battery.c
)

