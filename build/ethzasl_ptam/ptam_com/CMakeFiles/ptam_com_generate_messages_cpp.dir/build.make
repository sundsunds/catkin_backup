# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.2

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list

# Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/sun/catkin_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/sun/catkin_ws/build

# Utility rule file for ptam_com_generate_messages_cpp.

# Include the progress variables for this target.
include ethzasl_ptam/ptam_com/CMakeFiles/ptam_com_generate_messages_cpp.dir/progress.make

ethzasl_ptam/ptam_com/CMakeFiles/ptam_com_generate_messages_cpp: /home/sun/catkin_ws/devel/include/ptam_com/ptam_info.h
ethzasl_ptam/ptam_com/CMakeFiles/ptam_com_generate_messages_cpp: /home/sun/catkin_ws/devel/include/ptam_com/OctoMapScan.h
ethzasl_ptam/ptam_com/CMakeFiles/ptam_com_generate_messages_cpp: /home/sun/catkin_ws/devel/include/ptam_com/OctoMapPointStamped.h
ethzasl_ptam/ptam_com/CMakeFiles/ptam_com_generate_messages_cpp: /home/sun/catkin_ws/devel/include/ptam_com/Vector3Array.h
ethzasl_ptam/ptam_com/CMakeFiles/ptam_com_generate_messages_cpp: /home/sun/catkin_ws/devel/include/ptam_com/KeyFrame_msg.h
ethzasl_ptam/ptam_com/CMakeFiles/ptam_com_generate_messages_cpp: /home/sun/catkin_ws/devel/include/ptam_com/OctoMapPointArray.h
ethzasl_ptam/ptam_com/CMakeFiles/ptam_com_generate_messages_cpp: /home/sun/catkin_ws/devel/include/ptam_com/KeyFrame_srv.h
ethzasl_ptam/ptam_com/CMakeFiles/ptam_com_generate_messages_cpp: /home/sun/catkin_ws/devel/include/ptam_com/PointCloud.h

/home/sun/catkin_ws/devel/include/ptam_com/ptam_info.h: /opt/ros/indigo/lib/gencpp/gen_cpp.py
/home/sun/catkin_ws/devel/include/ptam_com/ptam_info.h: /home/sun/catkin_ws/src/ethzasl_ptam/ptam_com/msg/ptam_info.msg
/home/sun/catkin_ws/devel/include/ptam_com/ptam_info.h: /opt/ros/indigo/share/std_msgs/msg/Header.msg
/home/sun/catkin_ws/devel/include/ptam_com/ptam_info.h: /opt/ros/indigo/share/gencpp/msg.h.template
	$(CMAKE_COMMAND) -E cmake_progress_report /home/sun/catkin_ws/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Generating C++ code from ptam_com/ptam_info.msg"
	cd /home/sun/catkin_ws/build/ethzasl_ptam/ptam_com && ../../catkin_generated/env_cached.sh /usr/bin/python /opt/ros/indigo/share/gencpp/cmake/../../../lib/gencpp/gen_cpp.py /home/sun/catkin_ws/src/ethzasl_ptam/ptam_com/msg/ptam_info.msg -Iptam_com:/home/sun/catkin_ws/src/ethzasl_ptam/ptam_com/msg -Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg -Igeometry_msgs:/opt/ros/indigo/share/geometry_msgs/cmake/../msg -Isensor_msgs:/opt/ros/indigo/share/sensor_msgs/cmake/../msg -p ptam_com -o /home/sun/catkin_ws/devel/include/ptam_com -e /opt/ros/indigo/share/gencpp/cmake/..

/home/sun/catkin_ws/devel/include/ptam_com/OctoMapScan.h: /opt/ros/indigo/lib/gencpp/gen_cpp.py
/home/sun/catkin_ws/devel/include/ptam_com/OctoMapScan.h: /home/sun/catkin_ws/src/ethzasl_ptam/ptam_com/msg/OctoMapScan.msg
/home/sun/catkin_ws/devel/include/ptam_com/OctoMapScan.h: /home/sun/catkin_ws/src/ethzasl_ptam/ptam_com/msg/OctoMapPointStamped.msg
/home/sun/catkin_ws/devel/include/ptam_com/OctoMapScan.h: /opt/ros/indigo/share/geometry_msgs/msg/Point.msg
/home/sun/catkin_ws/devel/include/ptam_com/OctoMapScan.h: /opt/ros/indigo/share/geometry_msgs/msg/Quaternion.msg
/home/sun/catkin_ws/devel/include/ptam_com/OctoMapScan.h: /opt/ros/indigo/share/geometry_msgs/msg/PoseWithCovarianceStamped.msg
/home/sun/catkin_ws/devel/include/ptam_com/OctoMapScan.h: /opt/ros/indigo/share/geometry_msgs/msg/PoseWithCovariance.msg
/home/sun/catkin_ws/devel/include/ptam_com/OctoMapScan.h: /opt/ros/indigo/share/geometry_msgs/msg/Vector3.msg
/home/sun/catkin_ws/devel/include/ptam_com/OctoMapScan.h: /opt/ros/indigo/share/std_msgs/msg/Header.msg
/home/sun/catkin_ws/devel/include/ptam_com/OctoMapScan.h: /home/sun/catkin_ws/src/ethzasl_ptam/ptam_com/msg/OctoMapPointArray.msg
/home/sun/catkin_ws/devel/include/ptam_com/OctoMapScan.h: /opt/ros/indigo/share/geometry_msgs/msg/Pose.msg
/home/sun/catkin_ws/devel/include/ptam_com/OctoMapScan.h: /opt/ros/indigo/share/gencpp/msg.h.template
	$(CMAKE_COMMAND) -E cmake_progress_report /home/sun/catkin_ws/build/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Generating C++ code from ptam_com/OctoMapScan.msg"
	cd /home/sun/catkin_ws/build/ethzasl_ptam/ptam_com && ../../catkin_generated/env_cached.sh /usr/bin/python /opt/ros/indigo/share/gencpp/cmake/../../../lib/gencpp/gen_cpp.py /home/sun/catkin_ws/src/ethzasl_ptam/ptam_com/msg/OctoMapScan.msg -Iptam_com:/home/sun/catkin_ws/src/ethzasl_ptam/ptam_com/msg -Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg -Igeometry_msgs:/opt/ros/indigo/share/geometry_msgs/cmake/../msg -Isensor_msgs:/opt/ros/indigo/share/sensor_msgs/cmake/../msg -p ptam_com -o /home/sun/catkin_ws/devel/include/ptam_com -e /opt/ros/indigo/share/gencpp/cmake/..

/home/sun/catkin_ws/devel/include/ptam_com/OctoMapPointStamped.h: /opt/ros/indigo/lib/gencpp/gen_cpp.py
/home/sun/catkin_ws/devel/include/ptam_com/OctoMapPointStamped.h: /home/sun/catkin_ws/src/ethzasl_ptam/ptam_com/msg/OctoMapPointStamped.msg
/home/sun/catkin_ws/devel/include/ptam_com/OctoMapPointStamped.h: /opt/ros/indigo/share/geometry_msgs/msg/Vector3.msg
/home/sun/catkin_ws/devel/include/ptam_com/OctoMapPointStamped.h: /opt/ros/indigo/share/std_msgs/msg/Header.msg
/home/sun/catkin_ws/devel/include/ptam_com/OctoMapPointStamped.h: /opt/ros/indigo/share/gencpp/msg.h.template
	$(CMAKE_COMMAND) -E cmake_progress_report /home/sun/catkin_ws/build/CMakeFiles $(CMAKE_PROGRESS_3)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Generating C++ code from ptam_com/OctoMapPointStamped.msg"
	cd /home/sun/catkin_ws/build/ethzasl_ptam/ptam_com && ../../catkin_generated/env_cached.sh /usr/bin/python /opt/ros/indigo/share/gencpp/cmake/../../../lib/gencpp/gen_cpp.py /home/sun/catkin_ws/src/ethzasl_ptam/ptam_com/msg/OctoMapPointStamped.msg -Iptam_com:/home/sun/catkin_ws/src/ethzasl_ptam/ptam_com/msg -Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg -Igeometry_msgs:/opt/ros/indigo/share/geometry_msgs/cmake/../msg -Isensor_msgs:/opt/ros/indigo/share/sensor_msgs/cmake/../msg -p ptam_com -o /home/sun/catkin_ws/devel/include/ptam_com -e /opt/ros/indigo/share/gencpp/cmake/..

/home/sun/catkin_ws/devel/include/ptam_com/Vector3Array.h: /opt/ros/indigo/lib/gencpp/gen_cpp.py
/home/sun/catkin_ws/devel/include/ptam_com/Vector3Array.h: /home/sun/catkin_ws/src/ethzasl_ptam/ptam_com/msg/Vector3Array.msg
/home/sun/catkin_ws/devel/include/ptam_com/Vector3Array.h: /opt/ros/indigo/share/geometry_msgs/msg/Vector3.msg
/home/sun/catkin_ws/devel/include/ptam_com/Vector3Array.h: /opt/ros/indigo/share/std_msgs/msg/Header.msg
/home/sun/catkin_ws/devel/include/ptam_com/Vector3Array.h: /opt/ros/indigo/share/gencpp/msg.h.template
	$(CMAKE_COMMAND) -E cmake_progress_report /home/sun/catkin_ws/build/CMakeFiles $(CMAKE_PROGRESS_4)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Generating C++ code from ptam_com/Vector3Array.msg"
	cd /home/sun/catkin_ws/build/ethzasl_ptam/ptam_com && ../../catkin_generated/env_cached.sh /usr/bin/python /opt/ros/indigo/share/gencpp/cmake/../../../lib/gencpp/gen_cpp.py /home/sun/catkin_ws/src/ethzasl_ptam/ptam_com/msg/Vector3Array.msg -Iptam_com:/home/sun/catkin_ws/src/ethzasl_ptam/ptam_com/msg -Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg -Igeometry_msgs:/opt/ros/indigo/share/geometry_msgs/cmake/../msg -Isensor_msgs:/opt/ros/indigo/share/sensor_msgs/cmake/../msg -p ptam_com -o /home/sun/catkin_ws/devel/include/ptam_com -e /opt/ros/indigo/share/gencpp/cmake/..

/home/sun/catkin_ws/devel/include/ptam_com/KeyFrame_msg.h: /opt/ros/indigo/lib/gencpp/gen_cpp.py
/home/sun/catkin_ws/devel/include/ptam_com/KeyFrame_msg.h: /home/sun/catkin_ws/src/ethzasl_ptam/ptam_com/msg/KeyFrame_msg.msg
/home/sun/catkin_ws/devel/include/ptam_com/KeyFrame_msg.h: /opt/ros/indigo/share/geometry_msgs/msg/Point.msg
/home/sun/catkin_ws/devel/include/ptam_com/KeyFrame_msg.h: /opt/ros/indigo/share/geometry_msgs/msg/PoseWithCovariance.msg
/home/sun/catkin_ws/devel/include/ptam_com/KeyFrame_msg.h: /opt/ros/indigo/share/geometry_msgs/msg/PoseWithCovarianceStamped.msg
/home/sun/catkin_ws/devel/include/ptam_com/KeyFrame_msg.h: /opt/ros/indigo/share/std_msgs/msg/Header.msg
/home/sun/catkin_ws/devel/include/ptam_com/KeyFrame_msg.h: /opt/ros/indigo/share/geometry_msgs/msg/Quaternion.msg
/home/sun/catkin_ws/devel/include/ptam_com/KeyFrame_msg.h: /opt/ros/indigo/share/geometry_msgs/msg/Pose.msg
/home/sun/catkin_ws/devel/include/ptam_com/KeyFrame_msg.h: /opt/ros/indigo/share/gencpp/msg.h.template
	$(CMAKE_COMMAND) -E cmake_progress_report /home/sun/catkin_ws/build/CMakeFiles $(CMAKE_PROGRESS_5)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Generating C++ code from ptam_com/KeyFrame_msg.msg"
	cd /home/sun/catkin_ws/build/ethzasl_ptam/ptam_com && ../../catkin_generated/env_cached.sh /usr/bin/python /opt/ros/indigo/share/gencpp/cmake/../../../lib/gencpp/gen_cpp.py /home/sun/catkin_ws/src/ethzasl_ptam/ptam_com/msg/KeyFrame_msg.msg -Iptam_com:/home/sun/catkin_ws/src/ethzasl_ptam/ptam_com/msg -Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg -Igeometry_msgs:/opt/ros/indigo/share/geometry_msgs/cmake/../msg -Isensor_msgs:/opt/ros/indigo/share/sensor_msgs/cmake/../msg -p ptam_com -o /home/sun/catkin_ws/devel/include/ptam_com -e /opt/ros/indigo/share/gencpp/cmake/..

/home/sun/catkin_ws/devel/include/ptam_com/OctoMapPointArray.h: /opt/ros/indigo/lib/gencpp/gen_cpp.py
/home/sun/catkin_ws/devel/include/ptam_com/OctoMapPointArray.h: /home/sun/catkin_ws/src/ethzasl_ptam/ptam_com/msg/OctoMapPointArray.msg
/home/sun/catkin_ws/devel/include/ptam_com/OctoMapPointArray.h: /opt/ros/indigo/share/geometry_msgs/msg/Vector3.msg
/home/sun/catkin_ws/devel/include/ptam_com/OctoMapPointArray.h: /home/sun/catkin_ws/src/ethzasl_ptam/ptam_com/msg/OctoMapPointStamped.msg
/home/sun/catkin_ws/devel/include/ptam_com/OctoMapPointArray.h: /opt/ros/indigo/share/std_msgs/msg/Header.msg
/home/sun/catkin_ws/devel/include/ptam_com/OctoMapPointArray.h: /opt/ros/indigo/share/gencpp/msg.h.template
	$(CMAKE_COMMAND) -E cmake_progress_report /home/sun/catkin_ws/build/CMakeFiles $(CMAKE_PROGRESS_6)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Generating C++ code from ptam_com/OctoMapPointArray.msg"
	cd /home/sun/catkin_ws/build/ethzasl_ptam/ptam_com && ../../catkin_generated/env_cached.sh /usr/bin/python /opt/ros/indigo/share/gencpp/cmake/../../../lib/gencpp/gen_cpp.py /home/sun/catkin_ws/src/ethzasl_ptam/ptam_com/msg/OctoMapPointArray.msg -Iptam_com:/home/sun/catkin_ws/src/ethzasl_ptam/ptam_com/msg -Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg -Igeometry_msgs:/opt/ros/indigo/share/geometry_msgs/cmake/../msg -Isensor_msgs:/opt/ros/indigo/share/sensor_msgs/cmake/../msg -p ptam_com -o /home/sun/catkin_ws/devel/include/ptam_com -e /opt/ros/indigo/share/gencpp/cmake/..

/home/sun/catkin_ws/devel/include/ptam_com/KeyFrame_srv.h: /opt/ros/indigo/lib/gencpp/gen_cpp.py
/home/sun/catkin_ws/devel/include/ptam_com/KeyFrame_srv.h: /home/sun/catkin_ws/src/ethzasl_ptam/ptam_com/srv/KeyFrame_srv.srv
/home/sun/catkin_ws/devel/include/ptam_com/KeyFrame_srv.h: /opt/ros/indigo/share/geometry_msgs/msg/Point.msg
/home/sun/catkin_ws/devel/include/ptam_com/KeyFrame_srv.h: /opt/ros/indigo/share/geometry_msgs/msg/PoseWithCovariance.msg
/home/sun/catkin_ws/devel/include/ptam_com/KeyFrame_srv.h: /opt/ros/indigo/share/geometry_msgs/msg/PoseWithCovarianceStamped.msg
/home/sun/catkin_ws/devel/include/ptam_com/KeyFrame_srv.h: /opt/ros/indigo/share/std_msgs/msg/Header.msg
/home/sun/catkin_ws/devel/include/ptam_com/KeyFrame_srv.h: /opt/ros/indigo/share/geometry_msgs/msg/Quaternion.msg
/home/sun/catkin_ws/devel/include/ptam_com/KeyFrame_srv.h: /opt/ros/indigo/share/geometry_msgs/msg/Pose.msg
/home/sun/catkin_ws/devel/include/ptam_com/KeyFrame_srv.h: /opt/ros/indigo/share/gencpp/msg.h.template
/home/sun/catkin_ws/devel/include/ptam_com/KeyFrame_srv.h: /opt/ros/indigo/share/gencpp/srv.h.template
	$(CMAKE_COMMAND) -E cmake_progress_report /home/sun/catkin_ws/build/CMakeFiles $(CMAKE_PROGRESS_7)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Generating C++ code from ptam_com/KeyFrame_srv.srv"
	cd /home/sun/catkin_ws/build/ethzasl_ptam/ptam_com && ../../catkin_generated/env_cached.sh /usr/bin/python /opt/ros/indigo/share/gencpp/cmake/../../../lib/gencpp/gen_cpp.py /home/sun/catkin_ws/src/ethzasl_ptam/ptam_com/srv/KeyFrame_srv.srv -Iptam_com:/home/sun/catkin_ws/src/ethzasl_ptam/ptam_com/msg -Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg -Igeometry_msgs:/opt/ros/indigo/share/geometry_msgs/cmake/../msg -Isensor_msgs:/opt/ros/indigo/share/sensor_msgs/cmake/../msg -p ptam_com -o /home/sun/catkin_ws/devel/include/ptam_com -e /opt/ros/indigo/share/gencpp/cmake/..

/home/sun/catkin_ws/devel/include/ptam_com/PointCloud.h: /opt/ros/indigo/lib/gencpp/gen_cpp.py
/home/sun/catkin_ws/devel/include/ptam_com/PointCloud.h: /home/sun/catkin_ws/src/ethzasl_ptam/ptam_com/srv/PointCloud.srv
/home/sun/catkin_ws/devel/include/ptam_com/PointCloud.h: /opt/ros/indigo/share/sensor_msgs/msg/PointField.msg
/home/sun/catkin_ws/devel/include/ptam_com/PointCloud.h: /opt/ros/indigo/share/sensor_msgs/msg/PointCloud2.msg
/home/sun/catkin_ws/devel/include/ptam_com/PointCloud.h: /opt/ros/indigo/share/std_msgs/msg/Header.msg
/home/sun/catkin_ws/devel/include/ptam_com/PointCloud.h: /opt/ros/indigo/share/gencpp/msg.h.template
/home/sun/catkin_ws/devel/include/ptam_com/PointCloud.h: /opt/ros/indigo/share/gencpp/srv.h.template
	$(CMAKE_COMMAND) -E cmake_progress_report /home/sun/catkin_ws/build/CMakeFiles $(CMAKE_PROGRESS_8)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Generating C++ code from ptam_com/PointCloud.srv"
	cd /home/sun/catkin_ws/build/ethzasl_ptam/ptam_com && ../../catkin_generated/env_cached.sh /usr/bin/python /opt/ros/indigo/share/gencpp/cmake/../../../lib/gencpp/gen_cpp.py /home/sun/catkin_ws/src/ethzasl_ptam/ptam_com/srv/PointCloud.srv -Iptam_com:/home/sun/catkin_ws/src/ethzasl_ptam/ptam_com/msg -Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg -Igeometry_msgs:/opt/ros/indigo/share/geometry_msgs/cmake/../msg -Isensor_msgs:/opt/ros/indigo/share/sensor_msgs/cmake/../msg -p ptam_com -o /home/sun/catkin_ws/devel/include/ptam_com -e /opt/ros/indigo/share/gencpp/cmake/..

ptam_com_generate_messages_cpp: ethzasl_ptam/ptam_com/CMakeFiles/ptam_com_generate_messages_cpp
ptam_com_generate_messages_cpp: /home/sun/catkin_ws/devel/include/ptam_com/ptam_info.h
ptam_com_generate_messages_cpp: /home/sun/catkin_ws/devel/include/ptam_com/OctoMapScan.h
ptam_com_generate_messages_cpp: /home/sun/catkin_ws/devel/include/ptam_com/OctoMapPointStamped.h
ptam_com_generate_messages_cpp: /home/sun/catkin_ws/devel/include/ptam_com/Vector3Array.h
ptam_com_generate_messages_cpp: /home/sun/catkin_ws/devel/include/ptam_com/KeyFrame_msg.h
ptam_com_generate_messages_cpp: /home/sun/catkin_ws/devel/include/ptam_com/OctoMapPointArray.h
ptam_com_generate_messages_cpp: /home/sun/catkin_ws/devel/include/ptam_com/KeyFrame_srv.h
ptam_com_generate_messages_cpp: /home/sun/catkin_ws/devel/include/ptam_com/PointCloud.h
ptam_com_generate_messages_cpp: ethzasl_ptam/ptam_com/CMakeFiles/ptam_com_generate_messages_cpp.dir/build.make
.PHONY : ptam_com_generate_messages_cpp

# Rule to build all files generated by this target.
ethzasl_ptam/ptam_com/CMakeFiles/ptam_com_generate_messages_cpp.dir/build: ptam_com_generate_messages_cpp
.PHONY : ethzasl_ptam/ptam_com/CMakeFiles/ptam_com_generate_messages_cpp.dir/build

ethzasl_ptam/ptam_com/CMakeFiles/ptam_com_generate_messages_cpp.dir/clean:
	cd /home/sun/catkin_ws/build/ethzasl_ptam/ptam_com && $(CMAKE_COMMAND) -P CMakeFiles/ptam_com_generate_messages_cpp.dir/cmake_clean.cmake
.PHONY : ethzasl_ptam/ptam_com/CMakeFiles/ptam_com_generate_messages_cpp.dir/clean

ethzasl_ptam/ptam_com/CMakeFiles/ptam_com_generate_messages_cpp.dir/depend:
	cd /home/sun/catkin_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/sun/catkin_ws/src /home/sun/catkin_ws/src/ethzasl_ptam/ptam_com /home/sun/catkin_ws/build /home/sun/catkin_ws/build/ethzasl_ptam/ptam_com /home/sun/catkin_ws/build/ethzasl_ptam/ptam_com/CMakeFiles/ptam_com_generate_messages_cpp.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : ethzasl_ptam/ptam_com/CMakeFiles/ptam_com_generate_messages_cpp.dir/depend

