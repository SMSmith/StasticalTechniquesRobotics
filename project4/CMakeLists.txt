cmake_minimum_required(VERSION 2.8)
project(monteCarlo)
find_package(OpenCV 3.0.0 EXACT REQUIRED)
find_package(Eigen3 REQUIRED)
set( NAME_SRC
	monteCarlo.cpp
)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
endif()

INCLUDE_DIRECTORIES( ${CMAKE_CURRENT_SOURCE_DIR}/include Eigen/)
link_directories( ${CMAKE_BINARY_DIR}/bin )
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
add_executable( monteCarlo ${NAME_SRC} ${NAME_HEADERS} )

target_link_libraries( monteCarlo ${OpenCV_LIBS} )