cmake_minimum_required(VERSION 3.12)

# Pull in SDK (must be before project)
include(pico_sdk_import.cmake)

project(PicoFC C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)


# Set flags and directory variables
set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -DTSDEBUG")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -DTSDEBUG")
set(FREERTOS_CFG_DIRECTORY "${CMAKE_SOURCE_DIR}/config")
set(FREERTOS_SRC_DIRECTORY "${CMAKE_SOURCE_DIR}/FreeRTOS-Kernel")

# Initialize the SDK
pico_sdk_init()

add_compile_options(-Wall
        -Wno-format          # int != int32_t as far as the compiler is concerned because gcc has int32_t as long int
        -Wno-unused-function # we have some for the docs that aren't called
        -Wno-maybe-uninitialized
        )

# Add FreeRTOS as a library
add_library(FreeRTOS STATIC
    ${FREERTOS_SRC_DIRECTORY}/event_groups.c
    ${FREERTOS_SRC_DIRECTORY}/list.c
    ${FREERTOS_SRC_DIRECTORY}/queue.c
    ${FREERTOS_SRC_DIRECTORY}/stream_buffer.c
    ${FREERTOS_SRC_DIRECTORY}/tasks.c
    ${FREERTOS_SRC_DIRECTORY}/timers.c
    ${FREERTOS_SRC_DIRECTORY}/portable/MemMang/heap_3.c
    ${FREERTOS_SRC_DIRECTORY}/portable/GCC/ARM_CM0/port.c
)

# Build FreeRTOS
target_include_directories(FreeRTOS PUBLIC
    ${FREERTOS_CFG_DIRECTORY}/
    ${FREERTOS_SRC_DIRECTORY}/include
    ${FREERTOS_SRC_DIRECTORY}/portable/GCC/ARM_CM0
)

# Add peripherals drivers
add_subdirectory(drivers)

add_executable(picoFC
        src/main.c
        src/sensors/driver_mpu6050_interface.c
        src/radio/crsf.c
        src/misc/crc8.c
        src/tasks/task_crsf.c
        src/tasks/task_mpu6050.c
        src/tasks/task_leds.c
        )

target_include_directories(picoFC PUBLIC
        "${PROJECT_SOURCE_DIR}/drivers"
        )

# pull in common dependencies and additional i2c hardware support
target_link_libraries(picoFC pico_stdlib hardware_i2c hardware_pwm mpu6050 FreeRTOS)

# create map/bin/hex file etc.
pico_add_extra_outputs(picoFC)

pico_enable_stdio_usb(picoFC 1)
pico_enable_stdio_uart(picoFC 0)