cmake_minimum_required(VERSION 3.24 FATAL_ERROR)

project(green CXX CUDA Fortran)

find_package(CGAL REQUIRED)
find_package(spdlog REQUIRED)
find_package(LAPACK REQUIRED)

add_library(cuda_green STATIC src/cuda_impl.cu)

set_target_properties(cuda_green PROPERTIES
	CUDA_SEPARABLE_COMPILATION ON
	CUDA_RESOLVE_DEVICE_SYMBOLS ON)

find_package(CUDAToolkit REQUIRED)

set_target_properties(cuda_green PROPERTIES CUDA_ARCHITECTURES "50;72")

target_link_libraries(cuda_green
	CUDA::cudart
	Eigen3::Eigen
	CGAL::CGAL
)

set(${PROJECT_NAME}_HEADERS
	include/somigliana/catch.hpp
	include/somigliana/helper_string.h
	include/somigliana/kelvin_state.h
	include/somigliana/io.h
	include/somigliana/macro.h
	include/somigliana/nanoflann.hpp
	include/somigliana/quadrule.hpp
	include/somigliana/util.h
	include/somigliana/vtk.h
	include/somigliana/types.h
	include/somigliana/trig_quad_rule.h
	include/somigliana/helper_cuda.h
	include/somigliana/cuda_impl.h
	include/somigliana/somigliana_2d.h
	include/somigliana/somigliana_3d.h
)

set(${PROJECT_NAME}_SOURCES
	src/io.cc
	src/quadrule.cpp
	src/somigliana_2d.cc
	src/somigliana_3d.cc
	src/tet_fem.f90
	src/trig_quad_rule.cc
)

add_library(${PROJECT_NAME} STATIC ${${PROJECT_NAME}_SOURCES} ${${PROJECT_NAME}_HEADERS})
target_sources(${PROJECT_NAME} PRIVATE ${${PROJECT_NAME}_SOURCES})

target_link_libraries(${PROJECT_NAME}
	igl::core
	${Boost_LIBRARIES}
	${LAPACK_LIBRARIES}
	cuda_green
	spdlog::spdlog_header_only
)

target_include_directories(${PROJECT_NAME}
	PUBLIC
		${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES})

target_include_directories(
	${PROJECT_NAME}
	PUBLIC
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
		$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
	PRIVATE
		${CMAKE_CURRENT_SOURCE_DIR}
		${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES}
)
