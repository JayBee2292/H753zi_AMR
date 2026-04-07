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
set(CMAKE_PROJECT_NAME h753_ros_humble)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME}
        Core/Inc/app_microros.h
        Core/Inc/app_robot.h
        Core/Inc/calc_input_data.h
        Core/Src/app_microros.c
        Core/Src/app_robot.c
        Core/Src/calc_input_data.c)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    Drivers/CMSIS/DSP/Include
    Drivers/CMSIS/DSP/PrivateInclude
    micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros/include
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
    ARM_MATH_CM7
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    ${CMAKE_CURRENT_SOURCE_DIR}/micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros/libmicroros.a
    # Add user defined libraries
)

# Add extra source files for micro-ROS
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    micro_ros_stm32cubemx_utils/extra_sources/custom_memory_manager.c
    micro_ros_stm32cubemx_utils/extra_sources/microros_allocators.c
    micro_ros_stm32cubemx_utils/extra_sources/microros_time.c
    micro_ros_stm32cubemx_utils/extra_sources/microros_transports/poll_transport.c
)
