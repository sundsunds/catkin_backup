# Install script for directory: /home/sun/catkin_ws/src/eigen3.3.2/unsupported/Eigen

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Devel")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/eigen3/unsupported/Eigen" TYPE FILE FILES
    "/home/sun/catkin_ws/src/eigen3.3.2/unsupported/Eigen/AdolcForward"
    "/home/sun/catkin_ws/src/eigen3.3.2/unsupported/Eigen/AlignedVector3"
    "/home/sun/catkin_ws/src/eigen3.3.2/unsupported/Eigen/ArpackSupport"
    "/home/sun/catkin_ws/src/eigen3.3.2/unsupported/Eigen/AutoDiff"
    "/home/sun/catkin_ws/src/eigen3.3.2/unsupported/Eigen/BVH"
    "/home/sun/catkin_ws/src/eigen3.3.2/unsupported/Eigen/EulerAngles"
    "/home/sun/catkin_ws/src/eigen3.3.2/unsupported/Eigen/FFT"
    "/home/sun/catkin_ws/src/eigen3.3.2/unsupported/Eigen/IterativeSolvers"
    "/home/sun/catkin_ws/src/eigen3.3.2/unsupported/Eigen/KroneckerProduct"
    "/home/sun/catkin_ws/src/eigen3.3.2/unsupported/Eigen/LevenbergMarquardt"
    "/home/sun/catkin_ws/src/eigen3.3.2/unsupported/Eigen/MatrixFunctions"
    "/home/sun/catkin_ws/src/eigen3.3.2/unsupported/Eigen/MoreVectorization"
    "/home/sun/catkin_ws/src/eigen3.3.2/unsupported/Eigen/MPRealSupport"
    "/home/sun/catkin_ws/src/eigen3.3.2/unsupported/Eigen/NonLinearOptimization"
    "/home/sun/catkin_ws/src/eigen3.3.2/unsupported/Eigen/NumericalDiff"
    "/home/sun/catkin_ws/src/eigen3.3.2/unsupported/Eigen/OpenGLSupport"
    "/home/sun/catkin_ws/src/eigen3.3.2/unsupported/Eigen/Polynomials"
    "/home/sun/catkin_ws/src/eigen3.3.2/unsupported/Eigen/Skyline"
    "/home/sun/catkin_ws/src/eigen3.3.2/unsupported/Eigen/SparseExtra"
    "/home/sun/catkin_ws/src/eigen3.3.2/unsupported/Eigen/SpecialFunctions"
    "/home/sun/catkin_ws/src/eigen3.3.2/unsupported/Eigen/Splines"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Devel")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/eigen3/unsupported/Eigen" TYPE DIRECTORY FILES "/home/sun/catkin_ws/src/eigen3.3.2/unsupported/Eigen/src" FILES_MATCHING REGEX "/[^/]*\\.h$")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/sun/catkin_ws/src/eigen3.3.2/build/unsupported/Eigen/CXX11/cmake_install.cmake")

endif()

