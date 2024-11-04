cmake_minimum_required(VERSION 3.18 FATAL_ERROR)

project(cageDeformation3D CXX)

# Only do these if this is the main project, and not if it is included through add_subdirectory.
if (CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
    set(CMAKE_CXX_STANDARD 14)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
    set(CMAKE_CXX_EXTENSIONS OFF)

    # Let's ensure -std=c++xx instead of -std=g++xx
    set(CMAKE_CXX_EXTENSIONS OFF)

    # Let's nicely support folders in IDEs.
    set_property(GLOBAL PROPERTY USE_FOLDERS ON)

    # On Windows with ClangCL and Linux we want to ignore the warning.
    set(Boost_NO_WARN_NEW_VERSIONS 1)

    # Don't bother us with the CGAL warning.
    set(CGAL_DO_NOT_WARN_ABOUT_CMAKE_BUILD_TYPE TRUE)
endif()

# Compilation using MSVC
if (MSVC)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /bigobj")
        
    # Force MSVC to use utf-8
    add_compile_options("$<$<C_COMPILER_ID:MSVC>:/utf-8>")
    add_compile_options("$<$<CXX_COMPILER_ID:MSVC>:/utf-8>")
endif()

# When compiling with gcc we need an extra flag to avoid an issue with intrisics "error: third argument to ‘__builtin_prefetch’ must be a constant".
if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    add_compile_definitions(EIGEN_DONT_VECTORIZE)
endif()

# Eigen3
find_package(Eigen3 REQUIRED)

# Additional dependencies required for the library.
add_subdirectory("${CMAKE_SOURCE_DIR}/external/LBC" LBCLib)
add_subdirectory("${CMAKE_SOURCE_DIR}/external/libigl" libIGL)
add_subdirectory("${CMAKE_SOURCE_DIR}/external/OpenFBX" OpenFBXLib)

option(SOMIG_WITH_CUDA "" OFF)

# On Linux we want to enable Somigliana only if we have CUDA.
if (UNIX)
    include(CheckLanguage)
    check_language(CUDA)

    if (CMAKE_CUDA_COMPILER)
        message("Found CUDA compiler and adding Somgliana.")

        if (NOT DEFINED CMAKE_CUDA_STANDARD)
            set(CMAKE_CUDA_STANDARD 11)
            set(CMAKE_CUDA_STANDARD_REQUIRED ON)
        endif()

        enable_language(CUDA)
        option(SOMIG_WITH_CUDA "" ON)
    else()
        message("No CUDA compiler found.")

    endif()
endif()

add_compile_definitions(WITH_SOMIGLIANA)
set(ENABLE_SOMIGLIANA true)
add_subdirectory("${CMAKE_SOURCE_DIR}/external/somigliana" SomiglianaLib)

# Include directories of the external dependencies.
set(IGL_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/external/libigl/include")
set(LBC_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/external/LBC/include")
set(CAGEDEFORMATIONS_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/cagedeformations/include")

if (DEFINED ENABLE_SOMIGLIANA)
    set(SOMIGLIANA_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/external/somigliana/include")
endif()

# OpenMP on Apple
if (APPLE)
    set(OPENMP_INCLUDE_DIR "/usr/local/opt/libomp/include")
endif()

# Add the library itself.
add_subdirectory(cagedeformations)

# Add the command-line application.
add_subdirectory(app)

# Add the viewer directory to set that up.
add_subdirectory(viewer)
