# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /rahome/ait_jellal/projects/ws_mav/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /rahome/ait_jellal/projects/ws_mav/src

# Include any dependencies generated for this target.
include nodes/libviso2/CMakeFiles/viso2.dir/depend.make

# Include the progress variables for this target.
include nodes/libviso2/CMakeFiles/viso2.dir/progress.make

# Include the compile flags for this target's objects.
include nodes/libviso2/CMakeFiles/viso2.dir/flags.make

nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/filter.cpp.o: nodes/libviso2/CMakeFiles/viso2.dir/flags.make
nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/filter.cpp.o: nodes/libviso2/libviso2/src/filter.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /rahome/ait_jellal/projects/ws_mav/src/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/filter.cpp.o"
	cd /rahome/ait_jellal/projects/ws_mav/src/nodes/libviso2 && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/viso2.dir/libviso2/src/filter.cpp.o -c /rahome/ait_jellal/projects/ws_mav/src/nodes/libviso2/libviso2/src/filter.cpp

nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/filter.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/viso2.dir/libviso2/src/filter.cpp.i"
	cd /rahome/ait_jellal/projects/ws_mav/src/nodes/libviso2 && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /rahome/ait_jellal/projects/ws_mav/src/nodes/libviso2/libviso2/src/filter.cpp > CMakeFiles/viso2.dir/libviso2/src/filter.cpp.i

nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/filter.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/viso2.dir/libviso2/src/filter.cpp.s"
	cd /rahome/ait_jellal/projects/ws_mav/src/nodes/libviso2 && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /rahome/ait_jellal/projects/ws_mav/src/nodes/libviso2/libviso2/src/filter.cpp -o CMakeFiles/viso2.dir/libviso2/src/filter.cpp.s

nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/filter.cpp.o.requires:
.PHONY : nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/filter.cpp.o.requires

nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/filter.cpp.o.provides: nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/filter.cpp.o.requires
	$(MAKE) -f nodes/libviso2/CMakeFiles/viso2.dir/build.make nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/filter.cpp.o.provides.build
.PHONY : nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/filter.cpp.o.provides

nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/filter.cpp.o.provides.build: nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/filter.cpp.o

nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/matcher.cpp.o: nodes/libviso2/CMakeFiles/viso2.dir/flags.make
nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/matcher.cpp.o: nodes/libviso2/libviso2/src/matcher.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /rahome/ait_jellal/projects/ws_mav/src/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/matcher.cpp.o"
	cd /rahome/ait_jellal/projects/ws_mav/src/nodes/libviso2 && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/viso2.dir/libviso2/src/matcher.cpp.o -c /rahome/ait_jellal/projects/ws_mav/src/nodes/libviso2/libviso2/src/matcher.cpp

nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/matcher.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/viso2.dir/libviso2/src/matcher.cpp.i"
	cd /rahome/ait_jellal/projects/ws_mav/src/nodes/libviso2 && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /rahome/ait_jellal/projects/ws_mav/src/nodes/libviso2/libviso2/src/matcher.cpp > CMakeFiles/viso2.dir/libviso2/src/matcher.cpp.i

nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/matcher.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/viso2.dir/libviso2/src/matcher.cpp.s"
	cd /rahome/ait_jellal/projects/ws_mav/src/nodes/libviso2 && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /rahome/ait_jellal/projects/ws_mav/src/nodes/libviso2/libviso2/src/matcher.cpp -o CMakeFiles/viso2.dir/libviso2/src/matcher.cpp.s

nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/matcher.cpp.o.requires:
.PHONY : nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/matcher.cpp.o.requires

nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/matcher.cpp.o.provides: nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/matcher.cpp.o.requires
	$(MAKE) -f nodes/libviso2/CMakeFiles/viso2.dir/build.make nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/matcher.cpp.o.provides.build
.PHONY : nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/matcher.cpp.o.provides

nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/matcher.cpp.o.provides.build: nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/matcher.cpp.o

nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/matrix.cpp.o: nodes/libviso2/CMakeFiles/viso2.dir/flags.make
nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/matrix.cpp.o: nodes/libviso2/libviso2/src/matrix.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /rahome/ait_jellal/projects/ws_mav/src/CMakeFiles $(CMAKE_PROGRESS_3)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/matrix.cpp.o"
	cd /rahome/ait_jellal/projects/ws_mav/src/nodes/libviso2 && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/viso2.dir/libviso2/src/matrix.cpp.o -c /rahome/ait_jellal/projects/ws_mav/src/nodes/libviso2/libviso2/src/matrix.cpp

nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/matrix.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/viso2.dir/libviso2/src/matrix.cpp.i"
	cd /rahome/ait_jellal/projects/ws_mav/src/nodes/libviso2 && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /rahome/ait_jellal/projects/ws_mav/src/nodes/libviso2/libviso2/src/matrix.cpp > CMakeFiles/viso2.dir/libviso2/src/matrix.cpp.i

nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/matrix.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/viso2.dir/libviso2/src/matrix.cpp.s"
	cd /rahome/ait_jellal/projects/ws_mav/src/nodes/libviso2 && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /rahome/ait_jellal/projects/ws_mav/src/nodes/libviso2/libviso2/src/matrix.cpp -o CMakeFiles/viso2.dir/libviso2/src/matrix.cpp.s

nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/matrix.cpp.o.requires:
.PHONY : nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/matrix.cpp.o.requires

nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/matrix.cpp.o.provides: nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/matrix.cpp.o.requires
	$(MAKE) -f nodes/libviso2/CMakeFiles/viso2.dir/build.make nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/matrix.cpp.o.provides.build
.PHONY : nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/matrix.cpp.o.provides

nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/matrix.cpp.o.provides.build: nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/matrix.cpp.o

nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/reconstruction.cpp.o: nodes/libviso2/CMakeFiles/viso2.dir/flags.make
nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/reconstruction.cpp.o: nodes/libviso2/libviso2/src/reconstruction.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /rahome/ait_jellal/projects/ws_mav/src/CMakeFiles $(CMAKE_PROGRESS_4)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/reconstruction.cpp.o"
	cd /rahome/ait_jellal/projects/ws_mav/src/nodes/libviso2 && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/viso2.dir/libviso2/src/reconstruction.cpp.o -c /rahome/ait_jellal/projects/ws_mav/src/nodes/libviso2/libviso2/src/reconstruction.cpp

nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/reconstruction.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/viso2.dir/libviso2/src/reconstruction.cpp.i"
	cd /rahome/ait_jellal/projects/ws_mav/src/nodes/libviso2 && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /rahome/ait_jellal/projects/ws_mav/src/nodes/libviso2/libviso2/src/reconstruction.cpp > CMakeFiles/viso2.dir/libviso2/src/reconstruction.cpp.i

nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/reconstruction.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/viso2.dir/libviso2/src/reconstruction.cpp.s"
	cd /rahome/ait_jellal/projects/ws_mav/src/nodes/libviso2 && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /rahome/ait_jellal/projects/ws_mav/src/nodes/libviso2/libviso2/src/reconstruction.cpp -o CMakeFiles/viso2.dir/libviso2/src/reconstruction.cpp.s

nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/reconstruction.cpp.o.requires:
.PHONY : nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/reconstruction.cpp.o.requires

nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/reconstruction.cpp.o.provides: nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/reconstruction.cpp.o.requires
	$(MAKE) -f nodes/libviso2/CMakeFiles/viso2.dir/build.make nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/reconstruction.cpp.o.provides.build
.PHONY : nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/reconstruction.cpp.o.provides

nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/reconstruction.cpp.o.provides.build: nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/reconstruction.cpp.o

nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/triangle.cpp.o: nodes/libviso2/CMakeFiles/viso2.dir/flags.make
nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/triangle.cpp.o: nodes/libviso2/libviso2/src/triangle.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /rahome/ait_jellal/projects/ws_mav/src/CMakeFiles $(CMAKE_PROGRESS_5)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/triangle.cpp.o"
	cd /rahome/ait_jellal/projects/ws_mav/src/nodes/libviso2 && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/viso2.dir/libviso2/src/triangle.cpp.o -c /rahome/ait_jellal/projects/ws_mav/src/nodes/libviso2/libviso2/src/triangle.cpp

nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/triangle.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/viso2.dir/libviso2/src/triangle.cpp.i"
	cd /rahome/ait_jellal/projects/ws_mav/src/nodes/libviso2 && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /rahome/ait_jellal/projects/ws_mav/src/nodes/libviso2/libviso2/src/triangle.cpp > CMakeFiles/viso2.dir/libviso2/src/triangle.cpp.i

nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/triangle.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/viso2.dir/libviso2/src/triangle.cpp.s"
	cd /rahome/ait_jellal/projects/ws_mav/src/nodes/libviso2 && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /rahome/ait_jellal/projects/ws_mav/src/nodes/libviso2/libviso2/src/triangle.cpp -o CMakeFiles/viso2.dir/libviso2/src/triangle.cpp.s

nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/triangle.cpp.o.requires:
.PHONY : nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/triangle.cpp.o.requires

nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/triangle.cpp.o.provides: nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/triangle.cpp.o.requires
	$(MAKE) -f nodes/libviso2/CMakeFiles/viso2.dir/build.make nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/triangle.cpp.o.provides.build
.PHONY : nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/triangle.cpp.o.provides

nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/triangle.cpp.o.provides.build: nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/triangle.cpp.o

nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/viso.cpp.o: nodes/libviso2/CMakeFiles/viso2.dir/flags.make
nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/viso.cpp.o: nodes/libviso2/libviso2/src/viso.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /rahome/ait_jellal/projects/ws_mav/src/CMakeFiles $(CMAKE_PROGRESS_6)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/viso.cpp.o"
	cd /rahome/ait_jellal/projects/ws_mav/src/nodes/libviso2 && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/viso2.dir/libviso2/src/viso.cpp.o -c /rahome/ait_jellal/projects/ws_mav/src/nodes/libviso2/libviso2/src/viso.cpp

nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/viso.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/viso2.dir/libviso2/src/viso.cpp.i"
	cd /rahome/ait_jellal/projects/ws_mav/src/nodes/libviso2 && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /rahome/ait_jellal/projects/ws_mav/src/nodes/libviso2/libviso2/src/viso.cpp > CMakeFiles/viso2.dir/libviso2/src/viso.cpp.i

nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/viso.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/viso2.dir/libviso2/src/viso.cpp.s"
	cd /rahome/ait_jellal/projects/ws_mav/src/nodes/libviso2 && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /rahome/ait_jellal/projects/ws_mav/src/nodes/libviso2/libviso2/src/viso.cpp -o CMakeFiles/viso2.dir/libviso2/src/viso.cpp.s

nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/viso.cpp.o.requires:
.PHONY : nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/viso.cpp.o.requires

nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/viso.cpp.o.provides: nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/viso.cpp.o.requires
	$(MAKE) -f nodes/libviso2/CMakeFiles/viso2.dir/build.make nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/viso.cpp.o.provides.build
.PHONY : nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/viso.cpp.o.provides

nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/viso.cpp.o.provides.build: nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/viso.cpp.o

nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/viso_mono.cpp.o: nodes/libviso2/CMakeFiles/viso2.dir/flags.make
nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/viso_mono.cpp.o: nodes/libviso2/libviso2/src/viso_mono.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /rahome/ait_jellal/projects/ws_mav/src/CMakeFiles $(CMAKE_PROGRESS_7)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/viso_mono.cpp.o"
	cd /rahome/ait_jellal/projects/ws_mav/src/nodes/libviso2 && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/viso2.dir/libviso2/src/viso_mono.cpp.o -c /rahome/ait_jellal/projects/ws_mav/src/nodes/libviso2/libviso2/src/viso_mono.cpp

nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/viso_mono.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/viso2.dir/libviso2/src/viso_mono.cpp.i"
	cd /rahome/ait_jellal/projects/ws_mav/src/nodes/libviso2 && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /rahome/ait_jellal/projects/ws_mav/src/nodes/libviso2/libviso2/src/viso_mono.cpp > CMakeFiles/viso2.dir/libviso2/src/viso_mono.cpp.i

nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/viso_mono.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/viso2.dir/libviso2/src/viso_mono.cpp.s"
	cd /rahome/ait_jellal/projects/ws_mav/src/nodes/libviso2 && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /rahome/ait_jellal/projects/ws_mav/src/nodes/libviso2/libviso2/src/viso_mono.cpp -o CMakeFiles/viso2.dir/libviso2/src/viso_mono.cpp.s

nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/viso_mono.cpp.o.requires:
.PHONY : nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/viso_mono.cpp.o.requires

nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/viso_mono.cpp.o.provides: nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/viso_mono.cpp.o.requires
	$(MAKE) -f nodes/libviso2/CMakeFiles/viso2.dir/build.make nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/viso_mono.cpp.o.provides.build
.PHONY : nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/viso_mono.cpp.o.provides

nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/viso_mono.cpp.o.provides.build: nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/viso_mono.cpp.o

nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/viso_stereo.cpp.o: nodes/libviso2/CMakeFiles/viso2.dir/flags.make
nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/viso_stereo.cpp.o: nodes/libviso2/libviso2/src/viso_stereo.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /rahome/ait_jellal/projects/ws_mav/src/CMakeFiles $(CMAKE_PROGRESS_8)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/viso_stereo.cpp.o"
	cd /rahome/ait_jellal/projects/ws_mav/src/nodes/libviso2 && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/viso2.dir/libviso2/src/viso_stereo.cpp.o -c /rahome/ait_jellal/projects/ws_mav/src/nodes/libviso2/libviso2/src/viso_stereo.cpp

nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/viso_stereo.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/viso2.dir/libviso2/src/viso_stereo.cpp.i"
	cd /rahome/ait_jellal/projects/ws_mav/src/nodes/libviso2 && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /rahome/ait_jellal/projects/ws_mav/src/nodes/libviso2/libviso2/src/viso_stereo.cpp > CMakeFiles/viso2.dir/libviso2/src/viso_stereo.cpp.i

nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/viso_stereo.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/viso2.dir/libviso2/src/viso_stereo.cpp.s"
	cd /rahome/ait_jellal/projects/ws_mav/src/nodes/libviso2 && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /rahome/ait_jellal/projects/ws_mav/src/nodes/libviso2/libviso2/src/viso_stereo.cpp -o CMakeFiles/viso2.dir/libviso2/src/viso_stereo.cpp.s

nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/viso_stereo.cpp.o.requires:
.PHONY : nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/viso_stereo.cpp.o.requires

nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/viso_stereo.cpp.o.provides: nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/viso_stereo.cpp.o.requires
	$(MAKE) -f nodes/libviso2/CMakeFiles/viso2.dir/build.make nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/viso_stereo.cpp.o.provides.build
.PHONY : nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/viso_stereo.cpp.o.provides

nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/viso_stereo.cpp.o.provides.build: nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/viso_stereo.cpp.o

# Object files for target viso2
viso2_OBJECTS = \
"CMakeFiles/viso2.dir/libviso2/src/filter.cpp.o" \
"CMakeFiles/viso2.dir/libviso2/src/matcher.cpp.o" \
"CMakeFiles/viso2.dir/libviso2/src/matrix.cpp.o" \
"CMakeFiles/viso2.dir/libviso2/src/reconstruction.cpp.o" \
"CMakeFiles/viso2.dir/libviso2/src/triangle.cpp.o" \
"CMakeFiles/viso2.dir/libviso2/src/viso.cpp.o" \
"CMakeFiles/viso2.dir/libviso2/src/viso_mono.cpp.o" \
"CMakeFiles/viso2.dir/libviso2/src/viso_stereo.cpp.o"

# External object files for target viso2
viso2_EXTERNAL_OBJECTS =

/rahome/ait_jellal/projects/ws_mav/devel/lib/libviso2.so: nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/filter.cpp.o
/rahome/ait_jellal/projects/ws_mav/devel/lib/libviso2.so: nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/matcher.cpp.o
/rahome/ait_jellal/projects/ws_mav/devel/lib/libviso2.so: nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/matrix.cpp.o
/rahome/ait_jellal/projects/ws_mav/devel/lib/libviso2.so: nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/reconstruction.cpp.o
/rahome/ait_jellal/projects/ws_mav/devel/lib/libviso2.so: nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/triangle.cpp.o
/rahome/ait_jellal/projects/ws_mav/devel/lib/libviso2.so: nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/viso.cpp.o
/rahome/ait_jellal/projects/ws_mav/devel/lib/libviso2.so: nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/viso_mono.cpp.o
/rahome/ait_jellal/projects/ws_mav/devel/lib/libviso2.so: nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/viso_stereo.cpp.o
/rahome/ait_jellal/projects/ws_mav/devel/lib/libviso2.so: nodes/libviso2/CMakeFiles/viso2.dir/build.make
/rahome/ait_jellal/projects/ws_mav/devel/lib/libviso2.so: nodes/libviso2/CMakeFiles/viso2.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX shared library /rahome/ait_jellal/projects/ws_mav/devel/lib/libviso2.so"
	cd /rahome/ait_jellal/projects/ws_mav/src/nodes/libviso2 && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/viso2.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
nodes/libviso2/CMakeFiles/viso2.dir/build: /rahome/ait_jellal/projects/ws_mav/devel/lib/libviso2.so
.PHONY : nodes/libviso2/CMakeFiles/viso2.dir/build

nodes/libviso2/CMakeFiles/viso2.dir/requires: nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/filter.cpp.o.requires
nodes/libviso2/CMakeFiles/viso2.dir/requires: nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/matcher.cpp.o.requires
nodes/libviso2/CMakeFiles/viso2.dir/requires: nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/matrix.cpp.o.requires
nodes/libviso2/CMakeFiles/viso2.dir/requires: nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/reconstruction.cpp.o.requires
nodes/libviso2/CMakeFiles/viso2.dir/requires: nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/triangle.cpp.o.requires
nodes/libviso2/CMakeFiles/viso2.dir/requires: nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/viso.cpp.o.requires
nodes/libviso2/CMakeFiles/viso2.dir/requires: nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/viso_mono.cpp.o.requires
nodes/libviso2/CMakeFiles/viso2.dir/requires: nodes/libviso2/CMakeFiles/viso2.dir/libviso2/src/viso_stereo.cpp.o.requires
.PHONY : nodes/libviso2/CMakeFiles/viso2.dir/requires

nodes/libviso2/CMakeFiles/viso2.dir/clean:
	cd /rahome/ait_jellal/projects/ws_mav/src/nodes/libviso2 && $(CMAKE_COMMAND) -P CMakeFiles/viso2.dir/cmake_clean.cmake
.PHONY : nodes/libviso2/CMakeFiles/viso2.dir/clean

nodes/libviso2/CMakeFiles/viso2.dir/depend:
	cd /rahome/ait_jellal/projects/ws_mav/src && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /rahome/ait_jellal/projects/ws_mav/src /rahome/ait_jellal/projects/ws_mav/src/nodes/libviso2 /rahome/ait_jellal/projects/ws_mav/src /rahome/ait_jellal/projects/ws_mav/src/nodes/libviso2 /rahome/ait_jellal/projects/ws_mav/src/nodes/libviso2/CMakeFiles/viso2.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : nodes/libviso2/CMakeFiles/viso2.dir/depend
