cmake_minimum_required(VERSION 2.4.6)
project(sparsestereo)

set(CMAKE_BUILD_TYPE RelWithDebInfo)
add_definitions(-O3 -march=native)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


#include(../../libks/common.cmake)

find_package(catkin REQUIRED COMPONENTS cmake_modules nodelet roscpp libks libks_msgs tf)

#find_package(Boost REQUIRED COMPONENTS system thread)
#find_package(Boost 1.40 COMPONENTS filesystem system serialization thread REQUIRED)
find_package(Boost COMPONENTS filesystem system serialization thread REQUIRED)
# boost 1.57
find_package(OpenCV REQUIRED)
find_package(SDL REQUIRED)
find_package(Eigen REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${SDL_INCLUDE_DIR})
##include_directories(${CVD_INCLUDE_DIR})
include_directories(${Eigen_INCLUDE_DIRS})
include_directories(
    ${catkin_INCLUDE_DIRS}
)

include_directories(${Boost_INCLUDE_DIRS})

catkin_package(
    INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/src # exported include paths for the package
    LIBRARIES capture_nodelet
    #LIBRARIES ptam ptam_nodebase ptam_stereo # exported libraries
    CATKIN_DEPENDS libks libks_msgs #rosbag # catkin projects that this project depends on
)

set(EXTERNAL_LIBRARIES
    ${catkin_LIBRARIES}
    #${OCTOMAP_LIBRARIES}
    ${Boost_LIBRARIES}
    ${SDL_LIBRARY}
)

set(SDL_LIBRARY ${SDL_LIBRARY} PARENT_SCOPE)

add_subdirectory(src)
