# Oscilloscope RP2040
# Copyright (C) 2024-2026 Daniel Gorbea <danielgorbea@hotmail.com>
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

add_executable(${PROJECT_NAME}
    main.c
    common.c
    oscilloscope.c
    protocol.c
    usb_config.c
)

target_include_directories(${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(${PROJECT_NAME}
    pico_stdlib
    pico_multicore
    hardware_irq
    hardware_uart
    hardware_clocks
    hardware_dma
    hardware_adc
    hardware_pwm
    usb_library_rp2040
)

pico_add_extra_outputs(${PROJECT_NAME})

pico_set_binary_type(${PROJECT_NAME} copy_to_ram)

execute_process(
    COMMAND git describe --tags --abbrev=0 --match "v[0-9]*"
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    OUTPUT_VARIABLE PROJECT_VERSION
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

message(STATUS "Oscilloscope ${PROJECT_VERSION}")
target_compile_definitions(${PROJECT_NAME} PRIVATE
    PROJECT_VERSION="${PROJECT_VERSION}"
)