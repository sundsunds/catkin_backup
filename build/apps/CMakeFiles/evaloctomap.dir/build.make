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

# Include any dependencies generated for this target.
include apps/CMakeFiles/evaloctomap.dir/depend.make

# Include the progress variables for this target.
include apps/CMakeFiles/evaloctomap.dir/progress.make

# Include the compile flags for this target's objects.
include apps/CMakeFiles/evaloctomap.dir/flags.make

apps/CMakeFiles/evaloctomap.dir/evaloctomap.o: apps/CMakeFiles/evaloctomap.dir/flags.make
apps/CMakeFiles/evaloctomap.dir/evaloctomap.o: /home/sun/catkin_ws/src/apps/evaloctomap.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/sun/catkin_ws/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object apps/CMakeFiles/evaloctomap.dir/evaloctomap.o"
	cd /home/sun/catkin_ws/build/apps && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/evaloctomap.dir/evaloctomap.o -c /home/sun/catkin_ws/src/apps/evaloctomap.cpp

apps/CMakeFiles/evaloctomap.dir/evaloctomap.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/evaloctomap.dir/evaloctomap.i"
	cd /home/sun/catkin_ws/build/apps && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/sun/catkin_ws/src/apps/evaloctomap.cpp > CMakeFiles/evaloctomap.dir/evaloctomap.i

apps/CMakeFiles/evaloctomap.dir/evaloctomap.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/evaloctomap.dir/evaloctomap.s"
	cd /home/sun/catkin_ws/build/apps && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/sun/catkin_ws/src/apps/evaloctomap.cpp -o CMakeFiles/evaloctomap.dir/evaloctomap.s

apps/CMakeFiles/evaloctomap.dir/evaloctomap.o.requires:
.PHONY : apps/CMakeFiles/evaloctomap.dir/evaloctomap.o.requires

apps/CMakeFiles/evaloctomap.dir/evaloctomap.o.provides: apps/CMakeFiles/evaloctomap.dir/evaloctomap.o.requires
	$(MAKE) -f apps/CMakeFiles/evaloctomap.dir/build.make apps/CMakeFiles/evaloctomap.dir/evaloctomap.o.provides.build
.PHONY : apps/CMakeFiles/evaloctomap.dir/evaloctomap.o.provides

apps/CMakeFiles/evaloctomap.dir/evaloctomap.o.provides.build: apps/CMakeFiles/evaloctomap.dir/evaloctomap.o

# Object files for target evaloctomap
evaloctomap_OBJECTS = \
"CMakeFiles/evaloctomap.dir/evaloctomap.o"

# External object files for target evaloctomap
evaloctomap_EXTERNAL_OBJECTS =

/home/sun/catkin_ws/src/apps/bin/evaloctomap: apps/CMakeFiles/evaloctomap.dir/evaloctomap.o
/home/sun/catkin_ws/src/apps/bin/evaloctomap: apps/CMakeFiles/evaloctomap.dir/build.make
/home/sun/catkin_ws/src/apps/bin/evaloctomap: /opt/ros/indigo/lib/liboctomap.so
/home/sun/catkin_ws/src/apps/bin/evaloctomap: /opt/ros/indigo/lib/liboctomath.so
/home/sun/catkin_ws/src/apps/bin/evaloctomap: /opt/ros/indigo/lib/liboctomap.so
/home/sun/catkin_ws/src/apps/bin/evaloctomap: /opt/ros/indigo/lib/liboctomath.so
/home/sun/catkin_ws/src/apps/bin/evaloctomap: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/sun/catkin_ws/src/apps/bin/evaloctomap: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/sun/catkin_ws/src/apps/bin/evaloctomap: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
/home/sun/catkin_ws/src/apps/bin/evaloctomap: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/sun/catkin_ws/src/apps/bin/evaloctomap: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/sun/catkin_ws/src/apps/bin/evaloctomap: /home/sun/catkin_ws/devel/lib/libks.so
/home/sun/catkin_ws/src/apps/bin/evaloctomap: /home/sun/catkin_ws/devel/lib/libelas.so
/home/sun/catkin_ws/src/apps/bin/evaloctomap: /usr/local/lib/libopencv_viz.so.2.4.9
/home/sun/catkin_ws/src/apps/bin/evaloctomap: /usr/local/lib/libopencv_videostab.so.2.4.9
/home/sun/catkin_ws/src/apps/bin/evaloctomap: /usr/local/lib/libopencv_superres.so.2.4.9
/home/sun/catkin_ws/src/apps/bin/evaloctomap: /usr/local/lib/libopencv_stitching.so.2.4.9
/home/sun/catkin_ws/src/apps/bin/evaloctomap: /usr/local/lib/libopencv_contrib.so.2.4.9
/home/sun/catkin_ws/src/apps/bin/evaloctomap: /usr/local/lib/libopencv_nonfree.so.2.4.9
/home/sun/catkin_ws/src/apps/bin/evaloctomap: /usr/local/lib/libopencv_ocl.so.2.4.9
/home/sun/catkin_ws/src/apps/bin/evaloctomap: /usr/local/lib/libopencv_gpu.so.2.4.9
/home/sun/catkin_ws/src/apps/bin/evaloctomap: /usr/local/lib/libopencv_photo.so.2.4.9
/home/sun/catkin_ws/src/apps/bin/evaloctomap: /usr/local/lib/libopencv_objdetect.so.2.4.9
/home/sun/catkin_ws/src/apps/bin/evaloctomap: /usr/local/lib/libopencv_legacy.so.2.4.9
/home/sun/catkin_ws/src/apps/bin/evaloctomap: /usr/local/lib/libopencv_video.so.2.4.9
/home/sun/catkin_ws/src/apps/bin/evaloctomap: /usr/local/lib/libopencv_ml.so.2.4.9
/home/sun/catkin_ws/src/apps/bin/evaloctomap: /usr/local/lib/libopencv_calib3d.so.2.4.9
/home/sun/catkin_ws/src/apps/bin/evaloctomap: /usr/local/lib/libopencv_features2d.so.2.4.9
/home/sun/catkin_ws/src/apps/bin/evaloctomap: /usr/local/lib/libopencv_highgui.so.2.4.9
/home/sun/catkin_ws/src/apps/bin/evaloctomap: /usr/local/lib/libopencv_imgproc.so.2.4.9
/home/sun/catkin_ws/src/apps/bin/evaloctomap: /usr/local/lib/libopencv_flann.so.2.4.9
/home/sun/catkin_ws/src/apps/bin/evaloctomap: /usr/local/lib/libopencv_core.so.2.4.9
/home/sun/catkin_ws/src/apps/bin/evaloctomap: /opt/ros/indigo/lib/libcv_bridge.so
/home/sun/catkin_ws/src/apps/bin/evaloctomap: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.2.4.8
/home/sun/catkin_ws/src/apps/bin/evaloctomap: /usr/lib/x86_64-linux-gnu/libopencv_video.so.2.4.8
/home/sun/catkin_ws/src/apps/bin/evaloctomap: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.2.4.8
/home/sun/catkin_ws/src/apps/bin/evaloctomap: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.2.4.8
/home/sun/catkin_ws/src/apps/bin/evaloctomap: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.2.4.8
/home/sun/catkin_ws/src/apps/bin/evaloctomap: /usr/lib/x86_64-linux-gnu/libopencv_ocl.so.2.4.8
/home/sun/catkin_ws/src/apps/bin/evaloctomap: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.2.4.8
/home/sun/catkin_ws/src/apps/bin/evaloctomap: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.2.4.8
/home/sun/catkin_ws/src/apps/bin/evaloctomap: /usr/lib/x86_64-linux-gnu/libopencv_legacy.so.2.4.8
/home/sun/catkin_ws/src/apps/bin/evaloctomap: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.2.4.8
/home/sun/catkin_ws/src/apps/bin/evaloctomap: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.2.4.8
/home/sun/catkin_ws/src/apps/bin/evaloctomap: /usr/lib/x86_64-linux-gnu/libopencv_gpu.so.2.4.8
/home/sun/catkin_ws/src/apps/bin/evaloctomap: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.2.4.8
/home/sun/catkin_ws/src/apps/bin/evaloctomap: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.2.4.8
/home/sun/catkin_ws/src/apps/bin/evaloctomap: /usr/lib/x86_64-linux-gnu/libopencv_core.so.2.4.8
/home/sun/catkin_ws/src/apps/bin/evaloctomap: /usr/lib/x86_64-linux-gnu/libopencv_contrib.so.2.4.8
/home/sun/catkin_ws/src/apps/bin/evaloctomap: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.2.4.8
/home/sun/catkin_ws/src/apps/bin/evaloctomap: /opt/ros/indigo/lib/libroscpp.so
/home/sun/catkin_ws/src/apps/bin/evaloctomap: /usr/lib/x86_64-linux-gnu/libboost_signals.so
/home/sun/catkin_ws/src/apps/bin/evaloctomap: /opt/ros/indigo/lib/librosconsole.so
/home/sun/catkin_ws/src/apps/bin/evaloctomap: /opt/ros/indigo/lib/librosconsole_log4cxx.so
/home/sun/catkin_ws/src/apps/bin/evaloctomap: /opt/ros/indigo/lib/librosconsole_backend_interface.so
/home/sun/catkin_ws/src/apps/bin/evaloctomap: /usr/lib/liblog4cxx.so
/home/sun/catkin_ws/src/apps/bin/evaloctomap: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/sun/catkin_ws/src/apps/bin/evaloctomap: /opt/ros/indigo/lib/libxmlrpcpp.so
/home/sun/catkin_ws/src/apps/bin/evaloctomap: /opt/ros/indigo/lib/libroscpp_serialization.so
/home/sun/catkin_ws/src/apps/bin/evaloctomap: /opt/ros/indigo/lib/librostime.so
/home/sun/catkin_ws/src/apps/bin/evaloctomap: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/sun/catkin_ws/src/apps/bin/evaloctomap: /opt/ros/indigo/lib/libcpp_common.so
/home/sun/catkin_ws/src/apps/bin/evaloctomap: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so
/home/sun/catkin_ws/src/apps/bin/evaloctomap: /usr/lib/x86_64-linux-gnu/libSDLmain.a
/home/sun/catkin_ws/src/apps/bin/evaloctomap: /usr/lib/x86_64-linux-gnu/libSDL.so
/home/sun/catkin_ws/src/apps/bin/evaloctomap: /usr/lib/libflycapture.so
/home/sun/catkin_ws/src/apps/bin/evaloctomap: /opt/ros/indigo/lib/libcv_bridge.so
/home/sun/catkin_ws/src/apps/bin/evaloctomap: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.2.4.8
/home/sun/catkin_ws/src/apps/bin/evaloctomap: /usr/lib/x86_64-linux-gnu/libopencv_video.so.2.4.8
/home/sun/catkin_ws/src/apps/bin/evaloctomap: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.2.4.8
/home/sun/catkin_ws/src/apps/bin/evaloctomap: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.2.4.8
/home/sun/catkin_ws/src/apps/bin/evaloctomap: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.2.4.8
/home/sun/catkin_ws/src/apps/bin/evaloctomap: /usr/lib/x86_64-linux-gnu/libopencv_ocl.so.2.4.8
/home/sun/catkin_ws/src/apps/bin/evaloctomap: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.2.4.8
/home/sun/catkin_ws/src/apps/bin/evaloctomap: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.2.4.8
/home/sun/catkin_ws/src/apps/bin/evaloctomap: /usr/lib/x86_64-linux-gnu/libopencv_legacy.so.2.4.8
/home/sun/catkin_ws/src/apps/bin/evaloctomap: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.2.4.8
/home/sun/catkin_ws/src/apps/bin/evaloctomap: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.2.4.8
/home/sun/catkin_ws/src/apps/bin/evaloctomap: /usr/lib/x86_64-linux-gnu/libopencv_gpu.so.2.4.8
/home/sun/catkin_ws/src/apps/bin/evaloctomap: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.2.4.8
/home/sun/catkin_ws/src/apps/bin/evaloctomap: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.2.4.8
/home/sun/catkin_ws/src/apps/bin/evaloctomap: /usr/lib/x86_64-linux-gnu/libopencv_core.so.2.4.8
/home/sun/catkin_ws/src/apps/bin/evaloctomap: /usr/lib/x86_64-linux-gnu/libopencv_contrib.so.2.4.8
/home/sun/catkin_ws/src/apps/bin/evaloctomap: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.2.4.8
/home/sun/catkin_ws/src/apps/bin/evaloctomap: /opt/ros/indigo/lib/libroscpp.so
/home/sun/catkin_ws/src/apps/bin/evaloctomap: /usr/lib/x86_64-linux-gnu/libboost_signals.so
/home/sun/catkin_ws/src/apps/bin/evaloctomap: /opt/ros/indigo/lib/librosconsole.so
/home/sun/catkin_ws/src/apps/bin/evaloctomap: /opt/ros/indigo/lib/librosconsole_log4cxx.so
/home/sun/catkin_ws/src/apps/bin/evaloctomap: /opt/ros/indigo/lib/librosconsole_backend_interface.so
/home/sun/catkin_ws/src/apps/bin/evaloctomap: /usr/lib/liblog4cxx.so
/home/sun/catkin_ws/src/apps/bin/evaloctomap: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/sun/catkin_ws/src/apps/bin/evaloctomap: /opt/ros/indigo/lib/libxmlrpcpp.so
/home/sun/catkin_ws/src/apps/bin/evaloctomap: /opt/ros/indigo/lib/libroscpp_serialization.so
/home/sun/catkin_ws/src/apps/bin/evaloctomap: /opt/ros/indigo/lib/librostime.so
/home/sun/catkin_ws/src/apps/bin/evaloctomap: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/sun/catkin_ws/src/apps/bin/evaloctomap: /opt/ros/indigo/lib/libcpp_common.so
/home/sun/catkin_ws/src/apps/bin/evaloctomap: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so
/home/sun/catkin_ws/src/apps/bin/evaloctomap: /usr/lib/x86_64-linux-gnu/libSDLmain.a
/home/sun/catkin_ws/src/apps/bin/evaloctomap: /usr/lib/x86_64-linux-gnu/libSDL.so
/home/sun/catkin_ws/src/apps/bin/evaloctomap: /usr/lib/libflycapture.so
/home/sun/catkin_ws/src/apps/bin/evaloctomap: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/sun/catkin_ws/src/apps/bin/evaloctomap: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/sun/catkin_ws/src/apps/bin/evaloctomap: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
/home/sun/catkin_ws/src/apps/bin/evaloctomap: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/sun/catkin_ws/src/apps/bin/evaloctomap: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/sun/catkin_ws/src/apps/bin/evaloctomap: apps/CMakeFiles/evaloctomap.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable /home/sun/catkin_ws/src/apps/bin/evaloctomap"
	cd /home/sun/catkin_ws/build/apps && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/evaloctomap.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
apps/CMakeFiles/evaloctomap.dir/build: /home/sun/catkin_ws/src/apps/bin/evaloctomap
.PHONY : apps/CMakeFiles/evaloctomap.dir/build

apps/CMakeFiles/evaloctomap.dir/requires: apps/CMakeFiles/evaloctomap.dir/evaloctomap.o.requires
.PHONY : apps/CMakeFiles/evaloctomap.dir/requires

apps/CMakeFiles/evaloctomap.dir/clean:
	cd /home/sun/catkin_ws/build/apps && $(CMAKE_COMMAND) -P CMakeFiles/evaloctomap.dir/cmake_clean.cmake
.PHONY : apps/CMakeFiles/evaloctomap.dir/clean

apps/CMakeFiles/evaloctomap.dir/depend:
	cd /home/sun/catkin_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/sun/catkin_ws/src /home/sun/catkin_ws/src/apps /home/sun/catkin_ws/build /home/sun/catkin_ws/build/apps /home/sun/catkin_ws/build/apps/CMakeFiles/evaloctomap.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : apps/CMakeFiles/evaloctomap.dir/depend
