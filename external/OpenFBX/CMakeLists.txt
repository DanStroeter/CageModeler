project(OpenFBX LANGUAGES CXX C)

#find_package(miniz REQUIRED)

set(${PROJECT_NAME}_HEADERS
    include/OpenFBX/libdeflate.h
    include/OpenFBX/ofbx.h
)

set(${PROJECT_NAME}_SOURCES
    src/libdeflate.c
    src/ofbx.cpp
)

add_library(${PROJECT_NAME} STATIC ${${PROJECT_NAME}_SOURCES} ${${PROJECT_NAME}_HEADERS})
target_sources(${PROJECT_NAME} PRIVATE ${${PROJECT_NAME}_SOURCES})

#target_link_libraries(${PROJECT_NAME} PRIVATE miniz::miniz)

target_include_directories(
    ${PROJECT_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
)

target_compile_definitions(${PROJECT_NAME} PRIVATE _LARGEFILE64_SOURCE)
set_target_properties(${PROJECT_NAME} PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS ON)
