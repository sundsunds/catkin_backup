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
include libelas/CMakeFiles/elas_demo.dir/depend.make

# Include the progress variables for this target.
include libelas/CMakeFiles/elas_demo.dir/progress.make

# Include the compile flags for this target's objects.
include libelas/CMakeFiles/elas_demo.dir/flags.make

libelas/CMakeFiles/elas_demo.dir/src/main.o: libelas/CMakeFiles/elas_demo.dir/flags.make
libelas/CMakeFiles/elas_demo.dir/src/main.o: /home/sun/catkin_ws/src/libelas/src/main.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/sun/catkin_ws/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object libelas/CMakeFiles/elas_demo.dir/src/main.o"
	cd /home/sun/catkin_ws/build/libelas && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/elas_demo.dir/src/main.o -c /home/sun/catkin_ws/src/libelas/src/main.cpp

libelas/CMakeFiles/elas_demo.dir/src/main.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/elas_demo.dir/src/main.i"
	cd /home/sun/catkin_ws/build/libelas && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/sun/catkin_ws/src/libelas/src/main.cpp > CMakeFiles/elas_demo.dir/src/main.i

libelas/CMakeFiles/elas_demo.dir/src/main.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/elas_demo.dir/src/main.s"
	cd /home/sun/catkin_ws/build/libelas && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/sun/catkin_ws/src/libelas/src/main.cpp -o CMakeFiles/elas_demo.dir/src/main.s

libelas/CMakeFiles/elas_demo.dir/src/main.o.requires:
.PHONY : libelas/CMakeFiles/elas_demo.dir/src/main.o.requires

libelas/CMakeFiles/elas_demo.dir/src/main.o.provides: libelas/CMakeFiles/elas_demo.dir/src/main.o.requires
	$(MAKE) -f libelas/CMakeFiles/elas_demo.dir/build.make libelas/CMakeFiles/elas_demo.dir/src/main.o.provides.build
.PHONY : libelas/CMakeFiles/elas_demo.dir/src/main.o.provides

libelas/CMakeFiles/elas_demo.dir/src/main.o.provides.build: libelas/CMakeFiles/elas_demo.dir/src/main.o

# Object files for target elas_demo
elas_demo_OBJECTS = \
"CMakeFiles/elas_demo.dir/src/main.o"

# External object files for target elas_demo
elas_demo_EXTERNAL_OBJECTS =

/home/sun/catkin_ws/devel/lib/libelas/elas_demo: libelas/CMakeFiles/elas_demo.dir/src/main.o
/home/sun/catkin_ws/devel/lib/libelas/elas_demo: libelas/CMakeFiles/elas_demo.dir/build.make
/home/sun/catkin_ws/devel/lib/libelas/elas_demo: /home/sun/catkin_ws/devel/lib/libelas.so
/home/sun/catkin_ws/devel/lib/libelas/elas_demo: /usr/local/lib/libopencv_viz.so.2.4.9
/home/sun/catkin_ws/devel/lib/libelas/elas_demo: /usr/local/lib/libopencv_videostab.so.2.4.9
/home/sun/catkin_ws/devel/lib/libelas/elas_demo: /usr/local/lib/libopencv_superres.so.2.4.9
/home/sun/catkin_ws/devel/lib/libelas/elas_demo: /usr/local/lib/libopencv_stitching.so.2.4.9
/home/sun/catkin_ws/devel/lib/libelas/elas_demo: /usr/local/lib/libopencv_contrib.so.2.4.9
/home/sun/catkin_ws/devel/lib/libelas/elas_demo: /usr/local/lib/libopencv_nonfree.so.2.4.9
/home/sun/catkin_ws/devel/lib/libelas/elas_demo: /usr/local/lib/libopencv_ocl.so.2.4.9
/home/sun/catkin_ws/devel/lib/libelas/elas_demo: /usr/local/lib/libopencv_gpu.so.2.4.9
/home/sun/catkin_ws/devel/lib/libelas/elas_demo: /usr/local/lib/libopencv_photo.so.2.4.9
/home/sun/catkin_ws/devel/lib/libelas/elas_demo: /usr/local/lib/libopencv_objdetect.so.2.4.9
/home/sun/catkin_ws/devel/lib/libelas/elas_demo: /usr/local/lib/libopencv_legacy.so.2.4.9
/home/sun/catkin_ws/devel/lib/libelas/elas_demo: /usr/local/lib/libopencv_video.so.2.4.9
/home/sun/catkin_ws/devel/lib/libelas/elas_demo: /usr/local/lib/libopencv_ml.so.2.4.9
/home/sun/catkin_ws/devel/lib/libelas/elas_demo: /usr/local/lib/libopencv_calib3d.so.2.4.9
/home/sun/catkin_ws/devel/lib/libelas/elas_demo: /usr/local/lib/libopencv_features2d.so.2.4.9
/home/sun/catkin_ws/devel/lib/libelas/elas_demo: /usr/local/lib/libopencv_highgui.so.2.4.9
/home/sun/catkin_ws/devel/lib/libelas/elas_demo: /usr/local/lib/libopencv_imgproc.so.2.4.9
/home/sun/catkin_ws/devel/lib/libelas/elas_demo: /usr/local/lib/libopencv_flann.so.2.4.9
/home/sun/catkin_ws/devel/lib/libelas/elas_demo: /usr/local/lib/libopencv_core.so.2.4.9
/home/sun/catkin_ws/devel/lib/libelas/elas_demo: libelas/CMakeFiles/elas_demo.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable /home/sun/catkin_ws/devel/lib/libelas/elas_demo"
	cd /home/sun/catkin_ws/build/libelas && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/elas_demo.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
libelas/CMakeFiles/elas_demo.dir/build: /home/sun/catkin_ws/devel/lib/libelas/elas_demo
.PHONY : libelas/CMakeFiles/elas_demo.dir/build

libelas/CMakeFiles/elas_demo.dir/requires: libelas/CMakeFiles/elas_demo.dir/src/main.o.requires
.PHONY : libelas/CMakeFiles/elas_demo.dir/requires

libelas/CMakeFiles/elas_demo.dir/clean:
	cd /home/sun/catkin_ws/build/libelas && $(CMAKE_COMMAND) -P CMakeFiles/elas_demo.dir/cmake_clean.cmake
.PHONY : libelas/CMakeFiles/elas_demo.dir/clean

libelas/CMakeFiles/elas_demo.dir/depend:
	cd /home/sun/catkin_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/sun/catkin_ws/src /home/sun/catkin_ws/src/libelas /home/sun/catkin_ws/build /home/sun/catkin_ws/build/libelas /home/sun/catkin_ws/build/libelas/CMakeFiles/elas_demo.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : libelas/CMakeFiles/elas_demo.dir/depend

