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
set(CMAKE_PROJECT_NAME 3_CAN)

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
)

# Select the CAN node role at configure time.
set(CAN_NODE_ROLE "MASTER" CACHE STRING "CAN node role")
set_property(CACHE CAN_NODE_ROLE PROPERTY STRINGS MASTER SLAVE)

if(CAN_NODE_ROLE STREQUAL "MASTER")
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE CAN_NODE_MASTER)
elseif(CAN_NODE_ROLE STREQUAL "SLAVE")
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE CAN_NODE_SLAVE)
else()
    message(FATAL_ERROR "CAN_NODE_ROLE must be MASTER or SLAVE")
endif()

if(CAN_LOOPBACK_TEST)
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE CAN_LOOPBACK_TEST)
endif()

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
