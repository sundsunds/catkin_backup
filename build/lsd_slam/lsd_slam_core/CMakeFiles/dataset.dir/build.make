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
include lsd_slam/lsd_slam_core/CMakeFiles/dataset.dir/depend.make

# Include the progress variables for this target.
include lsd_slam/lsd_slam_core/CMakeFiles/dataset.dir/progress.make

# Include the compile flags for this target's objects.
include lsd_slam/lsd_slam_core/CMakeFiles/dataset.dir/flags.make

lsd_slam/lsd_slam_core/CMakeFiles/dataset.dir/src/main_on_images.cpp.o: lsd_slam/lsd_slam_core/CMakeFiles/dataset.dir/flags.make
lsd_slam/lsd_slam_core/CMakeFiles/dataset.dir/src/main_on_images.cpp.o: /home/sun/catkin_ws/src/lsd_slam/lsd_slam_core/src/main_on_images.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/sun/catkin_ws/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object lsd_slam/lsd_slam_core/CMakeFiles/dataset.dir/src/main_on_images.cpp.o"
	cd /home/sun/catkin_ws/build/lsd_slam/lsd_slam_core && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/dataset.dir/src/main_on_images.cpp.o -c /home/sun/catkin_ws/src/lsd_slam/lsd_slam_core/src/main_on_images.cpp

lsd_slam/lsd_slam_core/CMakeFiles/dataset.dir/src/main_on_images.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/dataset.dir/src/main_on_images.cpp.i"
	cd /home/sun/catkin_ws/build/lsd_slam/lsd_slam_core && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/sun/catkin_ws/src/lsd_slam/lsd_slam_core/src/main_on_images.cpp > CMakeFiles/dataset.dir/src/main_on_images.cpp.i

lsd_slam/lsd_slam_core/CMakeFiles/dataset.dir/src/main_on_images.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/dataset.dir/src/main_on_images.cpp.s"
	cd /home/sun/catkin_ws/build/lsd_slam/lsd_slam_core && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/sun/catkin_ws/src/lsd_slam/lsd_slam_core/src/main_on_images.cpp -o CMakeFiles/dataset.dir/src/main_on_images.cpp.s

lsd_slam/lsd_slam_core/CMakeFiles/dataset.dir/src/main_on_images.cpp.o.requires:
.PHONY : lsd_slam/lsd_slam_core/CMakeFiles/dataset.dir/src/main_on_images.cpp.o.requires

lsd_slam/lsd_slam_core/CMakeFiles/dataset.dir/src/main_on_images.cpp.o.provides: lsd_slam/lsd_slam_core/CMakeFiles/dataset.dir/src/main_on_images.cpp.o.requires
	$(MAKE) -f lsd_slam/lsd_slam_core/CMakeFiles/dataset.dir/build.make lsd_slam/lsd_slam_core/CMakeFiles/dataset.dir/src/main_on_images.cpp.o.provides.build
.PHONY : lsd_slam/lsd_slam_core/CMakeFiles/dataset.dir/src/main_on_images.cpp.o.provides

lsd_slam/lsd_slam_core/CMakeFiles/dataset.dir/src/main_on_images.cpp.o.provides.build: lsd_slam/lsd_slam_core/CMakeFiles/dataset.dir/src/main_on_images.cpp.o

# Object files for target dataset
dataset_OBJECTS = \
"CMakeFiles/dataset.dir/src/main_on_images.cpp.o"

# External object files for target dataset
dataset_EXTERNAL_OBJECTS =

/home/sun/catkin_ws/devel/lib/lsd_slam_core/dataset: lsd_slam/lsd_slam_core/CMakeFiles/dataset.dir/src/main_on_images.cpp.o
/home/sun/catkin_ws/devel/lib/lsd_slam_core/dataset: lsd_slam/lsd_slam_core/CMakeFiles/dataset.dir/build.make
/home/sun/catkin_ws/devel/lib/lsd_slam_core/dataset: /home/sun/catkin_ws/devel/lib/liblsdslam.so
/home/sun/catkin_ws/devel/lib/lsd_slam_core/dataset: /opt/ros/indigo/lib/libcv_bridge.so
/home/sun/catkin_ws/devel/lib/lsd_slam_core/dataset: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.2.4.8
/home/sun/catkin_ws/devel/lib/lsd_slam_core/dataset: /usr/lib/x86_64-linux-gnu/libopencv_video.so.2.4.8
/home/sun/catkin_ws/devel/lib/lsd_slam_core/dataset: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.2.4.8
/home/sun/catkin_ws/devel/lib/lsd_slam_core/dataset: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.2.4.8
/home/sun/catkin_ws/devel/lib/lsd_slam_core/dataset: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.2.4.8
/home/sun/catkin_ws/devel/lib/lsd_slam_core/dataset: /usr/lib/x86_64-linux-gnu/libopencv_ocl.so.2.4.8
/home/sun/catkin_ws/devel/lib/lsd_slam_core/dataset: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.2.4.8
/home/sun/catkin_ws/devel/lib/lsd_slam_core/dataset: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.2.4.8
/home/sun/catkin_ws/devel/lib/lsd_slam_core/dataset: /usr/lib/x86_64-linux-gnu/libopencv_legacy.so.2.4.8
/home/sun/catkin_ws/devel/lib/lsd_slam_core/dataset: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.2.4.8
/home/sun/catkin_ws/devel/lib/lsd_slam_core/dataset: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.2.4.8
/home/sun/catkin_ws/devel/lib/lsd_slam_core/dataset: /usr/lib/x86_64-linux-gnu/libopencv_gpu.so.2.4.8
/home/sun/catkin_ws/devel/lib/lsd_slam_core/dataset: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.2.4.8
/home/sun/catkin_ws/devel/lib/lsd_slam_core/dataset: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.2.4.8
/home/sun/catkin_ws/devel/lib/lsd_slam_core/dataset: /usr/lib/x86_64-linux-gnu/libopencv_core.so.2.4.8
/home/sun/catkin_ws/devel/lib/lsd_slam_core/dataset: /usr/lib/x86_64-linux-gnu/libopencv_contrib.so.2.4.8
/home/sun/catkin_ws/devel/lib/lsd_slam_core/dataset: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.2.4.8
/home/sun/catkin_ws/devel/lib/lsd_slam_core/dataset: /opt/ros/indigo/lib/libdynamic_reconfigure_config_init_mutex.so
/home/sun/catkin_ws/devel/lib/lsd_slam_core/dataset: /opt/ros/indigo/lib/libimage_transport.so
/home/sun/catkin_ws/devel/lib/lsd_slam_core/dataset: /opt/ros/indigo/lib/libmessage_filters.so
/home/sun/catkin_ws/devel/lib/lsd_slam_core/dataset: /usr/lib/x86_64-linux-gnu/libtinyxml.so
/home/sun/catkin_ws/devel/lib/lsd_slam_core/dataset: /opt/ros/indigo/lib/libclass_loader.so
/home/sun/catkin_ws/devel/lib/lsd_slam_core/dataset: /usr/lib/libPocoFoundation.so
/home/sun/catkin_ws/devel/lib/lsd_slam_core/dataset: /usr/lib/x86_64-linux-gnu/libdl.so
/home/sun/catkin_ws/devel/lib/lsd_slam_core/dataset: /opt/ros/indigo/lib/libroslib.so
/home/sun/catkin_ws/devel/lib/lsd_slam_core/dataset: /opt/ros/indigo/lib/librosbag.so
/home/sun/catkin_ws/devel/lib/lsd_slam_core/dataset: /opt/ros/indigo/lib/librosbag_storage.so
/home/sun/catkin_ws/devel/lib/lsd_slam_core/dataset: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
/home/sun/catkin_ws/devel/lib/lsd_slam_core/dataset: /opt/ros/indigo/lib/libroslz4.so
/home/sun/catkin_ws/devel/lib/lsd_slam_core/dataset: /usr/lib/x86_64-linux-gnu/liblz4.so
/home/sun/catkin_ws/devel/lib/lsd_slam_core/dataset: /opt/ros/indigo/lib/libtopic_tools.so
/home/sun/catkin_ws/devel/lib/lsd_slam_core/dataset: /opt/ros/indigo/lib/libroscpp.so
/home/sun/catkin_ws/devel/lib/lsd_slam_core/dataset: /usr/lib/x86_64-linux-gnu/libboost_signals.so
/home/sun/catkin_ws/devel/lib/lsd_slam_core/dataset: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/sun/catkin_ws/devel/lib/lsd_slam_core/dataset: /opt/ros/indigo/lib/librosconsole.so
/home/sun/catkin_ws/devel/lib/lsd_slam_core/dataset: /opt/ros/indigo/lib/librosconsole_log4cxx.so
/home/sun/catkin_ws/devel/lib/lsd_slam_core/dataset: /opt/ros/indigo/lib/librosconsole_backend_interface.so
/home/sun/catkin_ws/devel/lib/lsd_slam_core/dataset: /usr/lib/liblog4cxx.so
/home/sun/catkin_ws/devel/lib/lsd_slam_core/dataset: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/sun/catkin_ws/devel/lib/lsd_slam_core/dataset: /opt/ros/indigo/lib/libroscpp_serialization.so
/home/sun/catkin_ws/devel/lib/lsd_slam_core/dataset: /opt/ros/indigo/lib/librostime.so
/home/sun/catkin_ws/devel/lib/lsd_slam_core/dataset: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/sun/catkin_ws/devel/lib/lsd_slam_core/dataset: /opt/ros/indigo/lib/libxmlrpcpp.so
/home/sun/catkin_ws/devel/lib/lsd_slam_core/dataset: /opt/ros/indigo/lib/libcpp_common.so
/home/sun/catkin_ws/devel/lib/lsd_slam_core/dataset: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/sun/catkin_ws/devel/lib/lsd_slam_core/dataset: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/sun/catkin_ws/devel/lib/lsd_slam_core/dataset: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/sun/catkin_ws/devel/lib/lsd_slam_core/dataset: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so
/home/sun/catkin_ws/devel/lib/lsd_slam_core/dataset: /opt/ros/indigo/lib/libg2o_csparse_extension.so
/home/sun/catkin_ws/devel/lib/lsd_slam_core/dataset: /opt/ros/indigo/lib/libg2o_core.so
/home/sun/catkin_ws/devel/lib/lsd_slam_core/dataset: /opt/ros/indigo/lib/libg2o_stuff.so
/home/sun/catkin_ws/devel/lib/lsd_slam_core/dataset: /opt/ros/indigo/lib/libg2o_types_slam3d.so
/home/sun/catkin_ws/devel/lib/lsd_slam_core/dataset: /opt/ros/indigo/lib/libg2o_solver_cholmod.so
/home/sun/catkin_ws/devel/lib/lsd_slam_core/dataset: /opt/ros/indigo/lib/libg2o_solver_pcg.so
/home/sun/catkin_ws/devel/lib/lsd_slam_core/dataset: /opt/ros/indigo/lib/libg2o_solver_csparse.so
/home/sun/catkin_ws/devel/lib/lsd_slam_core/dataset: /opt/ros/indigo/lib/libg2o_incremental.so
/home/sun/catkin_ws/devel/lib/lsd_slam_core/dataset: /opt/ros/indigo/lib/libg2o_types_sba.so
/home/sun/catkin_ws/devel/lib/lsd_slam_core/dataset: lsd_slam/lsd_slam_core/CMakeFiles/dataset.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable /home/sun/catkin_ws/devel/lib/lsd_slam_core/dataset"
	cd /home/sun/catkin_ws/build/lsd_slam/lsd_slam_core && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/dataset.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
lsd_slam/lsd_slam_core/CMakeFiles/dataset.dir/build: /home/sun/catkin_ws/devel/lib/lsd_slam_core/dataset
.PHONY : lsd_slam/lsd_slam_core/CMakeFiles/dataset.dir/build

lsd_slam/lsd_slam_core/CMakeFiles/dataset.dir/requires: lsd_slam/lsd_slam_core/CMakeFiles/dataset.dir/src/main_on_images.cpp.o.requires
.PHONY : lsd_slam/lsd_slam_core/CMakeFiles/dataset.dir/requires

lsd_slam/lsd_slam_core/CMakeFiles/dataset.dir/clean:
	cd /home/sun/catkin_ws/build/lsd_slam/lsd_slam_core && $(CMAKE_COMMAND) -P CMakeFiles/dataset.dir/cmake_clean.cmake
.PHONY : lsd_slam/lsd_slam_core/CMakeFiles/dataset.dir/clean

lsd_slam/lsd_slam_core/CMakeFiles/dataset.dir/depend:
	cd /home/sun/catkin_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/sun/catkin_ws/src /home/sun/catkin_ws/src/lsd_slam/lsd_slam_core /home/sun/catkin_ws/build /home/sun/catkin_ws/build/lsd_slam/lsd_slam_core /home/sun/catkin_ws/build/lsd_slam/lsd_slam_core/CMakeFiles/dataset.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : lsd_slam/lsd_slam_core/CMakeFiles/dataset.dir/depend

