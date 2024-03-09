cmake_minimum_required(VERSION 3.18 FATAL_ERROR)

project(LBC CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Eigen3
find_package(Eigen3 REQUIRED)

if (NOT APPLE)
	find_package(OpenMP REQUIRED)
endif()

set(${PROJECT_NAME}_HEADERS
	include/LBC/LBCSolver.h
	include/LBC/DataSetup.h
	include/LBC/common.h
)

set(${PROJECT_NAME}_SOURCES
	src/LBCSolver.cpp
	src/DataSetup.cpp
)

add_library(${PROJECT_NAME} STATIC ${${PROJECT_NAME}_SOURCES} ${${PROJECT_NAME}_HEADERS})
target_sources(${PROJECT_NAME} PRIVATE ${${PROJECT_NAME}_SOURCES})

target_link_libraries(${PROJECT_NAME}
	PUBLIC
	Eigen3::Eigen
)

if (APPLE)
	target_include_directories(${PROJECT_NAME}
		PUBLIC
		"/opt/homebrew/opt/libomp/include")
	target_link_directories(${PROJECT_NAME}
		PUBLIC
		"/opt/homebrew/opt/libomp/lib")

	target_link_libraries(
		${PROJECT_NAME}
		PRIVATE
		omp
	)
elseif (UNIX)
	if (CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
		set_target_properties(${PROJECT_NAME}
			PROPERTIES
			COMPILE_FLAGS "-fopenmp=libomp")
	else()
		set_target_properties(${PROJECT_NAME}
			PROPERTIES
			COMPILE_FLAGS "-fopenmp")
	endif()
endif()

if (OpenMP_FOUND)
	target_compile_options(
		${PROJECT_NAME}
		PUBLIC
		$<$<COMPILE_LANGUAGE:CXX>:${OpenMP_CXX_FLAGS}>
	)
	target_link_libraries(
		${PROJECT_NAME}
		PUBLIC
		${OpenMP_CXX_LIBRARIES}
	)
endif()

# Using clang with clang-cl front end on Windows.
if (CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
	if (CMAKE_CXX_COMPILER_FRONTEND_VARIANT STREQUAL "MSVC")
		set_target_properties(${PROJECT_NAME}
			PROPERTIES
			COMPILE_FLAGS "-fopenmp -Xclang"
			LINK_FLAGS "-openmp")
	endif()
endif()

# Set include directories of the project.
target_include_directories(
	${PROJECT_NAME}
	PUBLIC
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
		$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
	PRIVATE
		${CMAKE_CURRENT_SOURCE_DIR}
)
