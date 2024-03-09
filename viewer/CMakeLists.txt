cmake_minimum_required(VERSION 3.13...3.28)

# Define the project.
project(
    CageModeler-App
    VERSION 1.0
    LANGUAGES CXX C)

# Set the C++ standard and other settings.
set(CMAKE_C_STANDARD 17)
set(CMAKE_VERBOSE_MAKEFILE ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

# strongly encouraged to enable this globally to avoid conflicts between
# -Wpedantic being enabled and -std=c++20 and -std=gnu++20 for example
# when compiling with PCH enabled
set(CMAKE_CXX_EXTENSIONS ON)

# Enable Vulkan validation layers for debug builds.
IF (MCD_ENABLE_VULKAN_VALIDATION)
    set(ENABLE_VULKAN_VALIDATION true)
ELSE()
    set(ENABLE_VULKAN_VALIDATION false)
ENDIF()

# Add vulkan dependencies using environment variable from LunarG SDK Setup.
IF (APPLE)
    include_directories("$ENV{VULKAN_SDK}/include")
    link_directories("$ENV{VULKAN_SDK}/lib")
    link_directories("$ENV{VULKAN_SDK}/share/vulkan/explicit_layer.d")
ELSE()
    include_directories("$ENV{VULKAN_SDK}/include")
    link_directories("$ENV{VULKAN_SDK}/lib")
    link_directories("$ENV{VULKAN_SDK}/etc/explicit_layer.d")
ENDIF()

# On multi-config setups we want to have all configurations.
get_property(BUILDING_MULTI_CONFIG GLOBAL PROPERTY GENERATOR_IS_MULTI_CONFIG)
if (BUILDING_MULTI_CONFIG)
    if (NOT CMAKE_BUILD_TYPE)
        # Make sure that all supported configuration types have their
        # associated conan packages available. You can reduce this
        # list to only the configuration types you use, but only if one
        # is not forced-set on the command line for VS
        message(TRACE "Setting up multi-config build types")
        set(CMAKE_CONFIGURATION_TYPES
                Debug
                Release
                RelWithDebInfo
                MinSizeRel
                CACHE STRING "Enabled build types" FORCE)
    else()
        message(TRACE "User chose a specific build type, so we are using that")
        set(CMAKE_CONFIGURATION_TYPES
                ${CMAKE_BUILD_TYPE}
                CACHE STRING "Enabled build types" FORCE)
    endif()
endif()

if (APPLE)
    # CMake versions 3 or later need CMAKE_MACOSX_RPATH defined.
    # This avoids the CMP0042 policy message.
    set(CMAKE_MACOSX_RPATH 1)
    # The "install" target for MacOS fixes up bundles in place.
    set(CMAKE_INSTALL_PREFIX ${CMAKE_BINARY_DIR})
endif()

include(cmake/Utilities.cmake)

# Required to compile SDL successfully.
if (APPLE)
    enable_language(OBJC)
endif()

add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/external/SDL3" EXCLUDE_FROM_ALL)
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/external/nativefiledialog-extended" nfd)

find_package(Python3 REQUIRED COMPONENTS Interpreter)

find_package(spdlog CONFIG REQUIRED)
find_package(glm CONFIG REQUIRED)
find_package(OpenMesh CONFIG REQUIRED)
find_package(volk CONFIG REQUIRED)
find_package(imgui CONFIG REQUIRED)
find_package(Vulkan REQUIRED)

# Configure files based on CMake configuration options.
add_subdirectory(configured_files)

# Add all of the app.
add_subdirectory(app)

# If MSVC is being used, and ASAN is enabled, we need to set the debugger environment
# so that it behaves well with MSVC's debugger, and we can run the target from visual studio
if (MSVC)
    get_all_installable_targets(all_targets)
    message("all_targets=${all_targets}")
    set_target_properties(${all_targets} PROPERTIES VS_DEBUGGER_ENVIRONMENT "PATH=$(VC_ExecutablePath_x64);%PATH%")
endif()
