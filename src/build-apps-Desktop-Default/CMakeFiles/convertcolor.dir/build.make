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
CMAKE_SOURCE_DIR = /home/sun/catkin_ws/src/apps

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/sun/catkin_ws/src/build-apps-Desktop-Default

# Include any dependencies generated for this target.
include CMakeFiles/convertcolor.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/convertcolor.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/convertcolor.dir/flags.make

CMakeFiles/convertcolor.dir/convertcolor.cpp.o: CMakeFiles/convertcolor.dir/flags.make
CMakeFiles/convertcolor.dir/convertcolor.cpp.o: /home/sun/catkin_ws/src/apps/convertcolor.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/sun/catkin_ws/src/build-apps-Desktop-Default/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/convertcolor.dir/convertcolor.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/convertcolor.dir/convertcolor.cpp.o -c /home/sun/catkin_ws/src/apps/convertcolor.cpp

CMakeFiles/convertcolor.dir/convertcolor.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/convertcolor.dir/convertcolor.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/sun/catkin_ws/src/apps/convertcolor.cpp > CMakeFiles/convertcolor.dir/convertcolor.cpp.i

CMakeFiles/convertcolor.dir/convertcolor.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/convertcolor.dir/convertcolor.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/sun/catkin_ws/src/apps/convertcolor.cpp -o CMakeFiles/convertcolor.dir/convertcolor.cpp.s

CMakeFiles/convertcolor.dir/convertcolor.cpp.o.requires:
.PHONY : CMakeFiles/convertcolor.dir/convertcolor.cpp.o.requires

CMakeFiles/convertcolor.dir/convertcolor.cpp.o.provides: CMakeFiles/convertcolor.dir/convertcolor.cpp.o.requires
	$(MAKE) -f CMakeFiles/convertcolor.dir/build.make CMakeFiles/convertcolor.dir/convertcolor.cpp.o.provides.build
.PHONY : CMakeFiles/convertcolor.dir/convertcolor.cpp.o.provides

CMakeFiles/convertcolor.dir/convertcolor.cpp.o.provides.build: CMakeFiles/convertcolor.dir/convertcolor.cpp.o

# Object files for target convertcolor
convertcolor_OBJECTS = \
"CMakeFiles/convertcolor.dir/convertcolor.cpp.o"

# External object files for target convertcolor
convertcolor_EXTERNAL_OBJECTS =

/home/sun/catkin_ws/src/apps/bin/convertcolor: CMakeFiles/convertcolor.dir/convertcolor.cpp.o
/home/sun/catkin_ws/src/apps/bin/convertcolor: CMakeFiles/convertcolor.dir/build.make
/home/sun/catkin_ws/src/apps/bin/convertcolor: /opt/ros/indigo/lib/liboctomap.so
/home/sun/catkin_ws/src/apps/bin/convertcolor: /opt/ros/indigo/lib/liboctomath.so
/home/sun/catkin_ws/src/apps/bin/convertcolor: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/sun/catkin_ws/src/apps/bin/convertcolor: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/sun/catkin_ws/src/apps/bin/convertcolor: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
/home/sun/catkin_ws/src/apps/bin/convertcolor: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/sun/catkin_ws/src/apps/bin/convertcolor: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/sun/catkin_ws/src/apps/bin/convertcolor: /home/sun/catkin_ws/devel/lib/libks.so
/home/sun/catkin_ws/src/apps/bin/convertcolor: CMakeFiles/convertcolor.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable /home/sun/catkin_ws/src/apps/bin/convertcolor"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/convertcolor.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/convertcolor.dir/build: /home/sun/catkin_ws/src/apps/bin/convertcolor
.PHONY : CMakeFiles/convertcolor.dir/build

CMakeFiles/convertcolor.dir/requires: CMakeFiles/convertcolor.dir/convertcolor.cpp.o.requires
.PHONY : CMakeFiles/convertcolor.dir/requires

CMakeFiles/convertcolor.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/convertcolor.dir/cmake_clean.cmake
.PHONY : CMakeFiles/convertcolor.dir/clean

CMakeFiles/convertcolor.dir/depend:
	cd /home/sun/catkin_ws/src/build-apps-Desktop-Default && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/sun/catkin_ws/src/apps /home/sun/catkin_ws/src/apps /home/sun/catkin_ws/src/build-apps-Desktop-Default /home/sun/catkin_ws/src/build-apps-Desktop-Default /home/sun/catkin_ws/src/build-apps-Desktop-Default/CMakeFiles/convertcolor.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/convertcolor.dir/depend
