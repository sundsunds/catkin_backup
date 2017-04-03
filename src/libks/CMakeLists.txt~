cmake_minimum_required(VERSION 2.4.6)
project(libks)

include(common.cmake)
set(ROS_BUILD_TYPE RelWithDebInfo)
add_definitions(-O3 -march=native)

find_package(cmake_modules REQUIRED)

#include(FindFLYCAP.cmake)

find_package(catkin REQUIRED COMPONENTS cv_bridge libelas libks_msgs rosconsole roscpp sensor_msgs)

#raj find_package(Boost 1.40 COMPONENTS filesystem system serialization thread REQUIRED)
find_package(Boost COMPONENTS filesystem system serialization thread REQUIRED)
# Boost 1.57

find_package(SDL REQUIRED)
find_package(Eigen REQUIRED)
#raj find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
#find_package(cvd REQUIRED)

#add_definitions("-DHAS_FLYCAPTURE")
flycapture_dependencies()

#message(${libelas_INCLUDE_DIRS} " ++ ++ +  libelas_inc_dir ! ks ++ RAJ  ")

#find_package(flycap REQUIRED)

catkin_package(
    INCLUDE_DIRS ${PROJECT_SOURCE_DIR} ${SDL_INCLUDE_DIR}# exported include paths for the package
    LIBRARIES ks # exported libraries
    CATKIN_DEPENDS # catkin projects that this project depends on
    DEPENDS # non-catkin cmake projects that this project depends on
)

include_directories(
  ${PROJECT_SOURCE_DIR}  
  ${catkin_INCLUDE_DIRS}
  ${SDL_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
  #${PCL_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${libelas_INCLUDE_DIRS}
  #/usr/include/flycapture
  ${FlyCapture_INCLUDE_DIRS}
)

#include_directories()

#include_directories(src)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
#set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

set(EXTERNAL_LIBRARIES
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  #${PCL_LIBRARIES}
  ${SDL_LIBRARY}
  ${OpenCV_LIBRARIES}
  ${FlyCapture_LIBRARY}
  #flycapture
  #raj cvd
  elas
)

add_subdirectory(libks)
#add_subdirectory(apps)


