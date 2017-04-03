cmake_minimum_required(VERSION 2.4.6)
project(libelas)

set(CMAKE_BUILD_TYPE RelWithDebInfo)
add_definitions(-O3 -march=native)

#include(../../libks/common.cmake)

find_package(catkin REQUIRED COMPONENTS cmake_modules roscpp rosconsole)

find_package(OpenCV REQUIRED)

# directories
set (LIBELAS_SRC_DIR "${PROJECT_SOURCE_DIR}")

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

catkin_package(
    INCLUDE_DIRS "${PROJECT_SOURCE_DIR}/src" # exported include paths for the package
    LIBRARIES elas#ks # exported libraries
    #CATKIN_DEPENDS # catkin projects that this project depends on
    #DEPENDS # non-catkin cmake projects that this project depends on
)

include_directories(${OpenCV_INCLUDE_DIRS})


# include directory
include_directories("${LIBELAS_SRC_DIR}")

# build library
add_library(elas
	src/descriptor.h
	src/descriptor.cpp
	src/elas.h
	src/elas.cpp
	src/filter.h
	src/filter.cpp
	src/matrix.h
	src/matrix.cpp
	src/timer.h
	src/triangle.h
	src/triangle.cpp
)

# build demo program

add_executable(elas_demo src/main.cpp)
target_link_libraries(elas_demo elas ${OpenCV_LIBS})

