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

# Utility rule file for conv_newcollege.

# Include the progress variables for this target.
include apps/CMakeFiles/conv_newcollege.dir/progress.make

conv_newcollege: apps/CMakeFiles/conv_newcollege.dir/build.make
.PHONY : conv_newcollege

# Rule to build all files generated by this target.
apps/CMakeFiles/conv_newcollege.dir/build: conv_newcollege
.PHONY : apps/CMakeFiles/conv_newcollege.dir/build

apps/CMakeFiles/conv_newcollege.dir/clean:
	cd /home/sun/catkin_ws/build/apps && $(CMAKE_COMMAND) -P CMakeFiles/conv_newcollege.dir/cmake_clean.cmake
.PHONY : apps/CMakeFiles/conv_newcollege.dir/clean

apps/CMakeFiles/conv_newcollege.dir/depend:
	cd /home/sun/catkin_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/sun/catkin_ws/src /home/sun/catkin_ws/src/apps /home/sun/catkin_ws/build /home/sun/catkin_ws/build/apps /home/sun/catkin_ws/build/apps/CMakeFiles/conv_newcollege.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : apps/CMakeFiles/conv_newcollege.dir/depend

