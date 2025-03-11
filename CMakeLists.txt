cmake_minimum_required(VERSION 3.22)

add_compile_options(-Wno-unused-parameter)

option(DEV_C_TEST_BUILD "Enable development C test build" OFF)

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
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/build)

# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

if(${DEV_C_TEST_BUILD})
    add_compile_definitions(${DEV_C_TEST_BUILD})
    message("Test build enabled.")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME DevC)

# Include toolchain file
include("cmake/mcu_stm32f4.cmake")

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})

# Enable CMake support for ASM and C languages
enable_language(C CXX ASM)


message("Build type: " ${CMAKE_BUILD_TYPE})

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME}.elf)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME}.elf PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME}.elf PRIVATE
    # Add user sources here
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME}.elf PRIVATE
    # Add user defined symbols
)


# LibXR
set(LIBXR_SYSTEM FreeRTOS)
set(LIBXR_DRIVER st)
add_subdirectory(libxr)
target_link_libraries(
    xr
    stm32cubemx
)

target_include_directories(xr
    PUBLIC $<TARGET_PROPERTY:stm32cubemx,INTERFACE_INCLUDE_DIRECTORIES>
    PUBLIC Core/Inc
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME}.elf PRIVATE
    # Add user defined include paths
    PUBLIC $<TARGET_PROPERTY:xr,INTERFACE_INCLUDE_DIRECTORIES>
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}.elf
    stm32cubemx

    # Add user defined libraries
    xr
)


if(${DEV_C_TEST_BUILD})
target_sources(${CMAKE_PROJECT_NAME}.elf
    PRIVATE Test/app_main.cpp
)

target_include_directories(stm32cubemx INTERFACE
    Test
)
else()
target_sources(${CMAKE_PROJECT_NAME}.elf
    PRIVATE User/app_main.cpp
)

target_include_directories(stm32cubemx INTERFACE
    User
)
endif()

# Prints the section sizes
function(print_section_sizes TARGET)
  # Build Events command
  add_custom_command(
    TARGET ${TARGET}
    POST_BUILD
    COMMAND ${CMAKE_SIZE} ${CMAKE_CURRENT_SOURCE_DIR}/build/${TARGET})
endfunction()

# Creates output in hex format
function(create_hex_output TARGET)
  add_custom_target(
    ${TARGET}.hex ALL
    DEPENDS ${TARGET}.elf
    COMMAND ${CMAKE_OBJCOPY} --output-target ihex ${CMAKE_CURRENT_SOURCE_DIR}/build/${TARGET}.elf ${CMAKE_CURRENT_SOURCE_DIR}/build/${TARGET}.hex)
endfunction()

# Creates output in binary format
function(create_bin_output TARGET)
  add_custom_target(
    ${TARGET}.bin ALL
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/build/${TARGET}.elf
    COMMAND ${CMAKE_OBJCOPY} --output-target binary --strip-all ${CMAKE_CURRENT_SOURCE_DIR}/build/${TARGET}.elf
            ${TARGET}.bin)
endfunction()

create_hex_output(${PROJECT_NAME})
create_bin_output(${PROJECT_NAME})
print_section_sizes(${PROJECT_NAME}.elf)
