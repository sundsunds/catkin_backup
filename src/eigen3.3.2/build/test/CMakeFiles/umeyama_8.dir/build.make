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
CMAKE_SOURCE_DIR = /home/sun/catkin_ws/src/eigen3.3.2

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/sun/catkin_ws/src/eigen3.3.2/build

# Include any dependencies generated for this target.
include test/CMakeFiles/umeyama_8.dir/depend.make

# Include the progress variables for this target.
include test/CMakeFiles/umeyama_8.dir/progress.make

# Include the compile flags for this target's objects.
include test/CMakeFiles/umeyama_8.dir/flags.make

test/CMakeFiles/umeyama_8.dir/umeyama.cpp.o: test/CMakeFiles/umeyama_8.dir/flags.make
test/CMakeFiles/umeyama_8.dir/umeyama.cpp.o: ../test/umeyama.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/sun/catkin_ws/src/eigen3.3.2/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object test/CMakeFiles/umeyama_8.dir/umeyama.cpp.o"
	cd /home/sun/catkin_ws/src/eigen3.3.2/build/test && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/umeyama_8.dir/umeyama.cpp.o -c /home/sun/catkin_ws/src/eigen3.3.2/test/umeyama.cpp

test/CMakeFiles/umeyama_8.dir/umeyama.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/umeyama_8.dir/umeyama.cpp.i"
	cd /home/sun/catkin_ws/src/eigen3.3.2/build/test && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/sun/catkin_ws/src/eigen3.3.2/test/umeyama.cpp > CMakeFiles/umeyama_8.dir/umeyama.cpp.i

test/CMakeFiles/umeyama_8.dir/umeyama.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/umeyama_8.dir/umeyama.cpp.s"
	cd /home/sun/catkin_ws/src/eigen3.3.2/build/test && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/sun/catkin_ws/src/eigen3.3.2/test/umeyama.cpp -o CMakeFiles/umeyama_8.dir/umeyama.cpp.s

test/CMakeFiles/umeyama_8.dir/umeyama.cpp.o.requires:
.PHONY : test/CMakeFiles/umeyama_8.dir/umeyama.cpp.o.requires

test/CMakeFiles/umeyama_8.dir/umeyama.cpp.o.provides: test/CMakeFiles/umeyama_8.dir/umeyama.cpp.o.requires
	$(MAKE) -f test/CMakeFiles/umeyama_8.dir/build.make test/CMakeFiles/umeyama_8.dir/umeyama.cpp.o.provides.build
.PHONY : test/CMakeFiles/umeyama_8.dir/umeyama.cpp.o.provides

test/CMakeFiles/umeyama_8.dir/umeyama.cpp.o.provides.build: test/CMakeFiles/umeyama_8.dir/umeyama.cpp.o

# Object files for target umeyama_8
umeyama_8_OBJECTS = \
"CMakeFiles/umeyama_8.dir/umeyama.cpp.o"

# External object files for target umeyama_8
umeyama_8_EXTERNAL_OBJECTS =

test/umeyama_8: test/CMakeFiles/umeyama_8.dir/umeyama.cpp.o
test/umeyama_8: test/CMakeFiles/umeyama_8.dir/build.make
test/umeyama_8: test/CMakeFiles/umeyama_8.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable umeyama_8"
	cd /home/sun/catkin_ws/src/eigen3.3.2/build/test && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/umeyama_8.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
test/CMakeFiles/umeyama_8.dir/build: test/umeyama_8
.PHONY : test/CMakeFiles/umeyama_8.dir/build

test/CMakeFiles/umeyama_8.dir/requires: test/CMakeFiles/umeyama_8.dir/umeyama.cpp.o.requires
.PHONY : test/CMakeFiles/umeyama_8.dir/requires

test/CMakeFiles/umeyama_8.dir/clean:
	cd /home/sun/catkin_ws/src/eigen3.3.2/build/test && $(CMAKE_COMMAND) -P CMakeFiles/umeyama_8.dir/cmake_clean.cmake
.PHONY : test/CMakeFiles/umeyama_8.dir/clean

test/CMakeFiles/umeyama_8.dir/depend:
	cd /home/sun/catkin_ws/src/eigen3.3.2/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/sun/catkin_ws/src/eigen3.3.2 /home/sun/catkin_ws/src/eigen3.3.2/test /home/sun/catkin_ws/src/eigen3.3.2/build /home/sun/catkin_ws/src/eigen3.3.2/build/test /home/sun/catkin_ws/src/eigen3.3.2/build/test/CMakeFiles/umeyama_8.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : test/CMakeFiles/umeyama_8.dir/depend
