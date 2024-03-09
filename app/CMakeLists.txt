include(CMakeFindDependencyMacro)

add_executable(${PROJECT_NAME} main.cpp)

set_directory_properties(PROPERTIES VS_STARTUP_PROJECT ${PROJECT_NAME})

# Boost
find_package(Boost REQUIRED COMPONENTS program_options)

target_include_directories(${PROJECT_NAME}
	PRIVATE
	${IGL_INCLUDE_DIR}
	${LBC_INCLUDE_DIR}
	${CAGEDEFORMATIONS_INCLUDE_DIR})

if (DEFINED ENABLE_SOMIGLIANA)
	target_include_directories(${PROJECT_NAME}
		PRIVATE
		${SOMIGLIANA_INCLUDE_DIR})
endif()

target_link_libraries(
	${PROJECT_NAME}
	PRIVATE
	cagedeformations
	Eigen3::Eigen
	Boost::program_options
)

install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION bin)
