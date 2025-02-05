cmake_minimum_required(VERSION 3.18 FATAL_ERROR)

project(CageGenerator CXX)

set(BOOST_ROOT "/opt/homebrew/Cellar/boost/1.87.0")
set(BOOST_INCLUDEDIR "/opt/homebrew/include")
set(BOOST_LIBRARYDIR "/opt/homebrew/lib")
find_package(CGAL REQUIRED COMPONENTS Core )

find_package(Eigen3 REQUIRED)

##set(CMAKE_PREFIX_PATH "/opt/homebrew/opt/cgal")
find_package(CGAL REQUIRED)
find_package(boost_headers REQUIRED)
find_package(boost_system REQUIRED)
find_package(boost_filesystem REQUIRED)

include_directories(${Boost_INCLUDE_DIRS})


include_directories(/opt/homebrew/opt/libomp/include)

add_executable(${PROJECT_NAME} main.cpp)

target_include_directories(${PROJECT_NAME} PUBLIC ${CGAL_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PRIVATE Boost::filesystem Boost::system)


include_directories(${EIGEN3_INCLUDE_DIR})

target_link_libraries(${PROJECT_NAME} PUBLIC CGAL::CGAL )




target_compile_options(${PROJECT_NAME} PRIVATE -Xpreprocessor -fopenmp)
target_link_libraries(${PROJECT_NAME} PRIVATE /opt/homebrew/opt/libomp/lib/libomp.dylib) 

 
  
target_link_libraries(${PROJECT_NAME} PRIVATE Boost::headers Boost::system Boost::filesystem)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "Choose the type of build." FORCE)
endif()