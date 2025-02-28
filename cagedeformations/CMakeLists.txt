cmake_minimum_required(VERSION 3.18 FATAL_ERROR)

set(LIBRARY_NAME cagedeformations)
set(ALIAS_NAME CageDeformations)

project(lib${LIBRARY_NAME} VERSION 1.0.0 LANGUAGES CXX)

list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_SOURCE_DIR}/cmake)

# Compilation using MSVC
if (MSVC)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /bigobj")
    set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
endif()

set(${LIBRARY_NAME}_HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/include/${LIBRARY_NAME}/globals.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/${LIBRARY_NAME}/GreenCoordinates.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/${LIBRARY_NAME}/MaximumLikelihoodCoordinates.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/${LIBRARY_NAME}/MaximumEntropyCoordinates.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/${LIBRARY_NAME}/LoadMesh.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/${LIBRARY_NAME}/LoadFBX.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/${LIBRARY_NAME}/InfluenceMap.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/${LIBRARY_NAME}/Parametrization.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/${LIBRARY_NAME}/WeightInterpolation.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/${LIBRARY_NAME}/somig.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/${LIBRARY_NAME}/BHC.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/${LIBRARY_NAME}/point3.h)

set(${LIBRARY_NAME}_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/globals.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/GreenCoordinates.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/MaximumLikelihoodCoordinates.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/MaximumEntropyCoordinates.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/LoadMesh.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/LoadFBX.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/InfluenceMap.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Parametrization.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/WeightInterpolation.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/somig.cpp)

# Add the source files to the library.
add_library(${LIBRARY_NAME} ${${LIBRARY_NAME}_SOURCES} ${${LIBRARY_NAME}_HEADERS})
target_sources(${LIBRARY_NAME} PRIVATE ${${LIBRARY_NAME}_SOURCES})

# Put the headers in a folder in the IDE.
source_group(
    TREE "${CMAKE_CURRENT_SOURCE_DIR}/include"
    PREFIX "Header Files"
    FILES ${${LIBRARY_NAME}_HEADERS})

# Add an alias so that library can be used inside the build tree.
add_library(${ALIAS_NAME}::${LIBRARY_NAME} ALIAS ${LIBRARY_NAME})

# Include the OpenMP library on macOS, requires installation through Homebrew as well.
if (APPLE)
    target_include_directories(${LIBRARY_NAME} PUBLIC "/opt/homebrew/opt/libomp/include")
    target_link_directories(${LIBRARY_NAME} PUBLIC "/opt/homebrew/opt/libomp/lib")
endif()

# Set include directories of the project.
target_include_directories(${LIBRARY_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
        $<BUILD_INTERFACE:${IGL_INCLUDE_DIR}>
)

target_compile_features(${LIBRARY_NAME} PRIVATE cxx_auto_type)
target_compile_options(${LIBRARY_NAME} PRIVATE
    $<$<OR:$<CXX_COMPILER_ID:Clang>,$<CXX_COMPILER_ID:AppleClang>,$<CXX_COMPILER_ID:GNU>>:
    -Wall -Wextra -Wpedantic>)

# Compile definitions for boost
# target_compile_definitions(${LIBRARY_NAME} PUBLIC BOOST_PROGRAM_OPTIONS_DYN_LINK)
target_compile_definitions(${LIBRARY_NAME} PUBLIC BOOST_ALL_DYN_LINK)

# Add a 'd' suffix to debug targets.
set_target_properties(${LIBRARY_NAME} PROPERTIES DEBUG_POSTFIX "d")

target_link_libraries(${LIBRARY_NAME}
    PUBLIC
        Eigen3::Eigen
    PRIVATE
        LBC
        OpenFBX
)

# On macOS we still link OpenMP.
if (APPLE)
    target_link_libraries(
        ${LIBRARY_NAME}
        PRIVATE
        omp
    )
endif()

if (DEFINED ENABLE_SOMIGLIANA)
    target_link_libraries(
        ${LIBRARY_NAME}
        PRIVATE
        green
    )
    target_compile_definitions(${LIBRARY_NAME} PRIVATE WITH_SOMIGLIANA)
endif()

# Required for CMAKE_INSTALL_* variables.
include(GNUInstallDirs)
set(INSTALL_CONFIGDIR ${CMAKE_INSTALL_LIBDIR}/cmake/${ALIAS_NAME})

set(INSTALL_EXTERNAL_LIBS LBC OpenFBX)

if (DEFINED ENABLE_SOMIGLIANA)
    list(APPEND INSTALL_EXTERNAL_LIBS green green_core igl_core)
endif()

install(TARGETS ${LIBRARY_NAME} ${INSTALL_EXTERNAL_LIBS}
    EXPORT ${LIBRARY_NAME}-targets
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${LIBRARY_NAME}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

set_target_properties(${LIBRARY_NAME} PROPERTIES EXPORT_NAME ${ALIAS_NAME})

install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# Export the targets.
install(EXPORT ${LIBRARY_NAME}-targets
    FILE ${ALIAS_NAME}Targets.cmake
    NAMESPACE ${ALIAS_NAME}::
    DESTINATION ${INSTALL_CONFIGDIR}
)

# Create a ConfigVersion.cmake file.
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/${ALIAS_NAME}ConfigVersion.cmake
    VERSION 1.0
    COMPATIBILITY AnyNewerVersion
)

configure_package_config_file(${CMAKE_SOURCE_DIR}/cmake/${ALIAS_NAME}Config.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/${ALIAS_NAME}Config.cmake
    INSTALL_DESTINATION ${INSTALL_CONFIGDIR}
)

# Export the targets.
install(FILES
    ${CMAKE_SOURCE_DIR}/cmake/FindOpenMP.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/${ALIAS_NAME}Config.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/${ALIAS_NAME}ConfigVersion.cmake
    DESTINATION ${INSTALL_CONFIGDIR}
)

configure_file(${CMAKE_SOURCE_DIR}/cmake/FindOpenMP.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/FindOpenMP.cmake
    COPYONLY)

# Register package in the User Package Registry.
export(EXPORT ${LIBRARY_NAME}-targets
    FILE ${CMAKE_CURRENT_BINARY_DIR}/${ALIAS_NAME}Targets.cmake
    NAMESPACE ${ALIAS_NAME}::)

export(PACKAGE ${ALIAS_NAME})
