cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME HM_Nightfall-mini_v1)

# Include toolchain file
include("cmake/gcc-arm-none-eabi.cmake")

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    Core/Src/auxiliary.c
    Core/Src/control.c
    Core/Src/dijkstra.c
    Core/Src/drive.c
    Core/Src/eeprom.c
    Core/Src/interrupt.c
    Core/Src/logging.c
    Core/Src/mode1.c
    Core/Src/mode2.c
    Core/Src/mode3.c
    Core/Src/mode4.c
    Core/Src/mode5.c
    Core/Src/mode6.c
    Core/Src/mode7.c
    Core/Src/path.c
    Core/Src/run.c
    Core/Src/search.c
    Core/Src/sensor.c
    Core/Src/shortest_run_params.c
    Core/Src/test_mode.c
    Core/Src/turn_time_calculator.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)

# Enable printf float support
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
    -u _printf_float    # floatのprintf対応を有効化
)

# Generate .bin file for UART flashing with CubeProgrammer
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.bin
    COMMENT "Generating binary file for UART flashing: ${CMAKE_PROJECT_NAME}.bin"
)

# Generate .hex file (optional)
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.hex
    COMMENT "Generating hex file: ${CMAKE_PROJECT_NAME}.hex"
)
