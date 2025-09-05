# Define the project.
project(
    CageModeler
    VERSION 1.0
    LANGUAGES CXX C)

# Define the output path for ImGUI bindings (default: <build>/bin).
set(MCD_BINARY_PATH "${CMAKE_BINARY_DIR}/bin" CACHE PATH "Binary output path for ImGUI bindings")

list(APPEND CMAKE_MODULE_PATH ${CMAKE_BINARY_DIR})

# ImGUI bindings to Vulkan and SDL2.
file(GLOB_RECURSE IMGUI_BINDINGS "${CMAKE_SOURCE_DIR}/viewer/external/ImGUI/Bindings/src/*.cpp")
include_directories("${CMAKE_SOURCE_DIR}/viewer/external/ImGUI/Bindings/include")

# On Apple we have MoltenVK, but we want to also be able to include the auto-generated header file
# vk_enum_string_helper.h to access result-to-string conversion for Vulkan result types.
IF (APPLE)
    include_directories("/usr/local/include")
endif ()

# Set the startup project for the "play" button in MSVC.
set_property(DIRECTORY PROPERTY VS_STARTUP_PROJECT ${PROJECT_NAME})

# Add the source files to the executable.
add_executable(${PROJECT_NAME}
    Core/Subsystem.cpp
    Logging/LoggingSubsystem.cpp
    Editor/Editor.cpp
    Editor/Scene.cpp
    Editor/Gizmo.cpp
    UI/WindowSubsystem.cpp
    UI/StatusBar.cpp
    UI/ProjectSettingsPanel.cpp
    UI/ProjectOptionsPanel.cpp
    UI/NewProjectPanel.cpp
    UI/ToolBar.cpp
    UI/UserInterfaceBackend.cpp
    Tools/ToolSystem.cpp
    Thread/ThreadPool.cpp
    Input/InputSubsystem.cpp
    Navigation/CameraSubsystem.cpp
    Navigation/Camera.cpp
    Mesh/BVH.cpp
    Mesh/PolygonMesh.cpp
    Mesh/PolygonMeshProxy.cpp
    Mesh/MeshTransformation.cpp
    Mesh/ScreenPass.cpp
    Mesh/MeshLibrary.cpp
    Mesh/Operations/MeshOperationSystem.cpp
    Mesh/Operations/MeshComputeWeightsOperation.cpp
    Mesh/Operations/MeshComputeInfluenceMapOperation.cpp
    Mesh/Operations/MeshComputeDeformationOperation.cpp
    Mesh/Operations/MeshExportInfluenceMapOperation.cpp
    Mesh/Operations/MeshLoadOperation.cpp
    Mesh/Operations/MeshExportOperation.cpp
    Rendering/Commands/BufferCommand.h
    Rendering/Commands/RenderCommandScheduler.cpp
    Rendering/Core/RenderProxy.cpp
    Rendering/Core/RenderProxyCollector.cpp
    Rendering/Core/Instance.cpp
    Rendering/Core/Device.cpp
    Rendering/Core/DescriptorPool.cpp
    Rendering/Core/DescriptorSetLayout.cpp
    Rendering/Core/RenderSurface.cpp
    Rendering/Core/Swapchain.cpp
    Rendering/Core/RenderResourceManager.cpp
    Rendering/Scene/SceneRenderer.cpp
    Rendering/RenderPipelineManager.cpp
    Rendering/RenderSubsystem.cpp
    App.cpp
    main.cpp
    ${IMGUI_BINDINGS})

target_compile_definitions(
    ${PROJECT_NAME}
    PRIVATE
        GLM_ENABLE_EXPERIMENTAL
        GLM_FORCE_DEPTH_ZERO_TO_ONE
        GLM_FORCE_RADIANS
        GLM_FORCE_DEFAULT_ALIGNED_GENTYPES
        IMGUI_IMPL_VULKAN_NO_PROTOTYPES
        SPDLOG_NO_EXCEPTIONS
        SDL_MAIN_HANDLED)

# Move the ImGUI files to the binaries folder.
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory "${CMAKE_SOURCE_DIR}/viewer/external/ImGUI/Bindings/include" "${MCD_BINARY_PATH}"
    COMMENT "Copying ImGUI files to build directory.")

# There are issues on Apple platforms with exceptions and -fno-rtti, so keep it
# enabled there.
# See https://stackoverflow.com/questions/21737201/problems-throwing-and-catching-exceptions-on-os-x-with-fno-rtti
# NOTE: Cannot use with OpenMesh, because it requires type information.
#if (MSVC)
#    target_compile_options(${PROJECT_NAME} PRIVATE /GR-)
#elseif(APPLE)
#    target_compile_options(${PROJECT_NAME} PRIVATE -fno-rtti)
#endif()

# Allow disabling exceptions, but warn the user about the consequences
if (MSVC)
    target_compile_options(${PROJECT_NAME} PUBLIC /EHsc)
else()
    if (CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
        target_compile_options(${PROJECT_NAME} PUBLIC -fno-exceptions)
    endif()
endif()

# if (WIN32)
    # set(ASSETS_OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_BUILD_TYPE}/assets")
# else()
    set(ASSETS_OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}/assets")
# endif()

# Copy all assets to the binary folder.
execute_process(
    COMMAND
    ${Python3_EXECUTABLE}
    "${CMAKE_SOURCE_DIR}/viewer/scripts/PackageAssets.py" "${CMAKE_SOURCE_DIR}/viewer/assets/packaged" "${ASSETS_OUTPUT_DIR}")

# Copy all models from the original project into the binary assets.
execute_process(
    COMMAND
    ${Python3_EXECUTABLE}
    "${CMAKE_SOURCE_DIR}/viewer/scripts/PackageAssets.py" "${CMAKE_SOURCE_DIR}/models" "${ASSETS_OUTPUT_DIR}/meshes")

# Compile the shaders.
execute_process(
    COMMAND
    ${Python3_EXECUTABLE}
    "${CMAKE_SOURCE_DIR}/viewer/scripts/CompileShaders.py" "${CMAKE_SOURCE_DIR}/viewer/assets/raw/shaders" "${ASSETS_OUTPUT_DIR}/shaders")

if (WIN32)
    add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy -t $<TARGET_FILE_DIR:${PROJECT_NAME}> $<TARGET_RUNTIME_DLLS:${PROJECT_NAME}>
        COMMAND_EXPAND_LISTS)
endif()

# Link the libraries against the project.
target_link_libraries(
    ${PROJECT_NAME}
    PRIVATE
        cagedeformations
        spdlog::spdlog
        glm
        volk::volk volk::volk_headers
        imgui::imgui
        Vulkan::Vulkan
        nfd
        SDL3::SDL3
        OpenMeshCore
)

# On Windows will enable SDL2 to recognize high DPI monitors.
if (WIN32)
    set_target_properties(${PROJECT_NAME} PROPERTIES
        VS_DPI_AWARE "PerMonitor"
    )
endif()

# Add location of all library files to include search path
target_include_directories(
    ${PROJECT_NAME}
    PRIVATE
        ${VulkanHeaders_INCLUDE_DIRS}
        ${VKBIND_INCLUDE_DIRS}
        ${OPENMESH_INCLUDE_DIRS}
        ${CAGEDEFORMATIONS_INCLUDE_DIR}
        ${IGL_INCLUDE_DIR}
        ${LBC_INCLUDE_DIR}
)

if (DEFINED ENABLE_SOMIGLIANA)
    target_include_directories(${PROJECT_NAME}
        PRIVATE
            ${SOMIGLIANA_INCLUDE_DIR})
endif()

# Include the ImGui font library.
target_include_directories(${PROJECT_NAME}
    PRIVATE
        "${CMAKE_SOURCE_DIR}/viewer/external/ImGUI/IconFontCppHeaders")

# Include SDL3 headers.
target_include_directories(${PROJECT_NAME}
    PRIVATE
        "${CMAKE_SOURCE_DIR}/viewer/external/SDL3/include")

target_include_directories(${PROJECT_NAME} BEFORE PRIVATE "${CMAKE_CURRENT_LIST_DIR}")

# Precompiled header file includes.
target_precompile_headers(
    ${PROJECT_NAME}
    PRIVATE
        <Core/Platform.h>
        <vector>)
