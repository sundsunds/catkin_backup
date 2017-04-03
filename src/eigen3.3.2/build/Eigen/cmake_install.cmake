# Install script for directory: /home/sun/catkin_ws/src/eigen3.3.2/Eigen

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/eigen3/Eigen" TYPE FILE FILES
    "/home/sun/catkin_ws/src/eigen3.3.2/Eigen/Sparse"
    "/home/sun/catkin_ws/src/eigen3.3.2/Eigen/LU"
    "/home/sun/catkin_ws/src/eigen3.3.2/Eigen/Cholesky"
    "/home/sun/catkin_ws/src/eigen3.3.2/Eigen/CholmodSupport"
    "/home/sun/catkin_ws/src/eigen3.3.2/Eigen/SPQRSupport"
    "/home/sun/catkin_ws/src/eigen3.3.2/Eigen/SVD"
    "/home/sun/catkin_ws/src/eigen3.3.2/Eigen/Geometry"
    "/home/sun/catkin_ws/src/eigen3.3.2/Eigen/OrderingMethods"
    "/home/sun/catkin_ws/src/eigen3.3.2/Eigen/SparseQR"
    "/home/sun/catkin_ws/src/eigen3.3.2/Eigen/StdDeque"
    "/home/sun/catkin_ws/src/eigen3.3.2/Eigen/PardisoSupport"
    "/home/sun/catkin_ws/src/eigen3.3.2/Eigen/Eigen"
    "/home/sun/catkin_ws/src/eigen3.3.2/Eigen/IterativeLinearSolvers"
    "/home/sun/catkin_ws/src/eigen3.3.2/Eigen/SparseCore"
    "/home/sun/catkin_ws/src/eigen3.3.2/Eigen/Eigenvalues"
    "/home/sun/catkin_ws/src/eigen3.3.2/Eigen/QR"
    "/home/sun/catkin_ws/src/eigen3.3.2/Eigen/Householder"
    "/home/sun/catkin_ws/src/eigen3.3.2/Eigen/QtAlignedMalloc"
    "/home/sun/catkin_ws/src/eigen3.3.2/Eigen/SparseLU"
    "/home/sun/catkin_ws/src/eigen3.3.2/Eigen/Core"
    "/home/sun/catkin_ws/src/eigen3.3.2/Eigen/Dense"
    "/home/sun/catkin_ws/src/eigen3.3.2/Eigen/StdVector"
    "/home/sun/catkin_ws/src/eigen3.3.2/Eigen/SparseCholesky"
    "/home/sun/catkin_ws/src/eigen3.3.2/Eigen/SuperLUSupport"
    "/home/sun/catkin_ws/src/eigen3.3.2/Eigen/MetisSupport"
    "/home/sun/catkin_ws/src/eigen3.3.2/Eigen/StdList"
    "/home/sun/catkin_ws/src/eigen3.3.2/Eigen/Jacobi"
    "/home/sun/catkin_ws/src/eigen3.3.2/Eigen/PaStiXSupport"
    "/home/sun/catkin_ws/src/eigen3.3.2/Eigen/UmfPackSupport"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Devel")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/eigen3/Eigen" TYPE DIRECTORY FILES "/home/sun/catkin_ws/src/eigen3.3.2/Eigen/src" FILES_MATCHING REGEX "/[^/]*\\.h$")
endif()

