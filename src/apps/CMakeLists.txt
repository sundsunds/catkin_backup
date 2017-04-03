cmake_minimum_required(VERSION 2.4.6)
project(apps)

set(CMAKE_BUILD_TYPE RelWithDebInfo)
add_definitions(-O3 -march=native)

#include(../../libks/common.cmake)

#find_package(catkin REQUIRED COMPONENTS cmake_modules roscpp rospy rosbag libks libks_msgs octomap_ros octomap_msgs geometry_msgs tf sensor_msgs rosconsole)
find_package(catkin REQUIRED COMPONENTS cmake_modules roscpp rospy rosbag libks libks_msgs octomap_msgs geometry_msgs tf sensor_msgs rosconsole)

#find_package(Boost REQUIRED COMPONENTS system thread)
#find_package(Boost 1.40 COMPONENTS filesystem system serialization thread REQUIRED)
find_package(Boost COMPONENTS filesystem system serialization thread REQUIRED)
#1.57 
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
include_directories(${LIBKS_MSGS_INCLUDE_DIRS})

link_libraries(${OCTOMAP_LIBRARIES})
link_directories(${OCTOMAP_LIBRARY_DIRS})


#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#ks_common_dependencies()
#rosbuild_add_boost_directories()

add_executable(stereo stereo.cpp)
#rosbuild_link_boost(stereo system filesystem thread)
target_link_libraries(stereo ${SDL_LIBRARY} ${Boost_LIBRARIES} ${libks_LIBRARIES})
#ks_create_link(stereo ${CMAKE_CURRENT_BINARY_DIR}/bin/stereo ../bin/stereo)



add_executable(feature feature.cpp)
target_link_libraries(feature ${SDL_LIBRARY} cvd ${Boost_LIBRARIES} ${libks_LIBRARIES})
#ks_create_link(feature ${CMAKE_CURRENT_BINARY_DIR}/bin/feature ../bin/feature)

add_executable(censusfeature censusfeature.cpp)
target_link_libraries(censusfeature ${SDL_LIBRARY} ${Boost_LIBRARIES} ${libks_LIBRARIES})
#ks_create_link(censusfeature ${CMAKE_CURRENT_BINARY_DIR}/bin/censusfeature ../bin/censusfeature)

add_executable(calibrate calibrate.cpp)
target_link_libraries(calibrate ${SDL_LIBRARY}  ${Boost_LIBRARIES} ${libks_LIBRARIES})
#ks_create_link(calibrate ${CMAKE_CURRENT_BINARY_DIR}/bin/calibrate ../bin/calibrate)

add_executable(rectify rectify.cpp)
target_link_libraries(rectify ${SDL_LIBRARY}  ${Boost_LIBRARIES} ${libks_LIBRARIES})
#ks_create_link(rectify ${CMAKE_CURRENT_BINARY_DIR}/bin/rectify ../bin/rectify)

add_executable(rawconv rawconv.cpp)
target_link_libraries(rawconv ${SDL_LIBRARY}  ${Boost_LIBRARIES} ${libks_LIBRARIES})
#ks_create_link(rawconv ${CMAKE_CURRENT_BINARY_DIR}/bin/rawconv ../bin/rawconv)

add_executable(evalfeature evalfeature.cpp)
target_link_libraries(evalfeature ${SDL_LIBRARY}  ${Boost_LIBRARIES} ${libks_LIBRARIES})
#ks_create_link(evalfeature ${CMAKE_CURRENT_BINARY_DIR}/bin/evalfeature ../bin/evalfeature)

add_executable(evalstereo evalstereo.cpp)
target_link_libraries(evalstereo ${SDL_LIBRARY} ${Boost_LIBRARIES} ${libks_LIBRARIES})
#ks_create_link(evalstereo ${CMAKE_CURRENT_BINARY_DIR}/bin/evalstereo ../bin/evalstereo)

add_executable(convertcapture convertcapture.cpp)
target_link_libraries(convertcapture ${Boost_LIBRARIES} ${libks_LIBRARIES}  ${rosbag_LIBRARIES})
#ks_create_link(convertcapture ${CMAKE_CURRENT_BINARY_DIR}/bin/convertcapture ../bin/convertcapture)

add_executable(pointstf pointstf.cpp)
target_link_libraries(pointstf ${Boost_LIBRARIES} ${libks_LIBRARIES}  ${tf_LIBRARIES})
#ks_create_link(pointstf ${CMAKE_CURRENT_BINARY_DIR}/bin/pointstf ../bin/pointstf)

add_executable(evaloctomap evaloctomap.cpp)
target_link_libraries(evaloctomap ${OCTOMAP_LIBRARIES}  ${Boost_LIBRARIES} ${libks_LIBRARIES})
#ks_create_link(evaloctomap ${CMAKE_CURRENT_BINARY_DIR}/bin/evaloctomap ../bin/evaloctomap)

add_custom_target(conv_rawseeds ALL)
#ks_create_link(conv_rawseeds ${PROJECT_SOURCE_DIR}/conv_rawseeds.py ../bin/conv_rawseeds.py)

add_custom_target(conv_kitti ALL)
#ks_create_link(conv_kitti ${PROJECT_SOURCE_DIR}/conv_kitti.py ../bin/conv_kitti.py)

add_custom_target(conv_newcollege ALL)
#ks_create_link(conv_newcollege ${PROJECT_SOURCE_DIR}/conv_newcollege.py ../bin/conv_newcollege.py)

add_executable(convertcolor convertcolor.cpp)
target_link_libraries(convertcolor ${Boost_LIBRARIES} ${libks_LIBRARIES})
#ks_create_link(convertcolor ${CMAKE_CURRENT_BINARY_DIR}/bin/convertcolor ../bin/convertcolor)

add_executable(extractframes extractframes.cpp)
target_link_libraries(extractframes ${Boost_LIBRARIES} ${libks_LIBRARIES} ${rosbag_LIBRARIES})
#ks_create_link(extractframes ${CMAKE_CURRENT_BINARY_DIR}/bin/extractframes ../bin/extractframes)
