cmake_minimum_required(VERSION 2.4.6)
project(occupancymap)

set(CMAKE_BUILD_TYPE RelWithDebInfo)
add_definitions(-O3 -march=native)

#include(../../libks/common.cmake)

find_package(catkin REQUIRED COMPONENTS cmake_modules roscpp rosbag libks libks_msgs octomap_ros octomap_msgs geometry_msgs tf pcl_ros pcl_conversions )

#find_package(Boost REQUIRED COMPONENTS system thread)
find_package(Boost 1.40 COMPONENTS filesystem system serialization thread REQUIRED)

#find_package(catkin REQUIRED COMPONENTS  pcl_ros pcl_conversions)

find_package(octomap REQUIRED)
find_package(OpenCV REQUIRED)
find_package(SDL REQUIRED)
find_package(Eigen REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${SDL_INCLUDE_DIR})
##include_directories(${CVD_INCLUDE_DIR})
include_directories(${Eigen_INCLUDE_DIRS})
include_directories(
    ${catkin_INCLUDE_DIRS}
    ${OCTOMAP_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)
include_directories(${OCTOMAP_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})

link_libraries(${OCTOMAP_LIBRARIES})
link_directories(${OCTOMAP_LIBRARY_DIRS})

catkin_package(
    INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/src # exported include paths for the package
    LIBRARIES occupancy_map_nodelet
    #LIBRARIES ptam ptam_nodebase ptam_stereo # exported libraries
    CATKIN_DEPENDS libks libks_msgs rosbag # catkin projects that this project depends on
)

set(EXTERNAL_LIBRARIES
    ${catkin_LIBRARIES}
    ${OCTOMAP_LIBRARIES}
    ${Boost_LIBRARIES}
    ${SDL_LIBRARY}
)


#set(octomap_ros_DIR /opt/ros/hydro/share/octomap)
#ks_common_dependencies()


##ks_find_library(flycapture flycapture/FlyCapture2.h FlyCapture WARNING)
##ks_find_library(cvd cvd/esm.h CVD ERROR)


# Export
##set(FoundFlyCapture ${FoundFlyCapture} PARENT_SCOPE)
##set(FlyCapture_LIBRARY ${FlyCapture_LIBRARY} PARENT_SCOPE)
set(SDL_LIBRARY ${SDL_LIBRARY} PARENT_SCOPE)

add_subdirectory(src)
