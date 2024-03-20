# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
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
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/kth/ros2_ws/src/YDLidar-SDK

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/kth/ros2_ws/src/YDLidar-SDK/build/temp.linux-x86_64-3.10

# Include any dependencies generated for this target.
include examples/CMakeFiles/gs_test.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include examples/CMakeFiles/gs_test.dir/compiler_depend.make

# Include the progress variables for this target.
include examples/CMakeFiles/gs_test.dir/progress.make

# Include the compile flags for this target's objects.
include examples/CMakeFiles/gs_test.dir/flags.make

examples/CMakeFiles/gs_test.dir/gs_test.cpp.o: examples/CMakeFiles/gs_test.dir/flags.make
examples/CMakeFiles/gs_test.dir/gs_test.cpp.o: ../../examples/gs_test.cpp
examples/CMakeFiles/gs_test.dir/gs_test.cpp.o: examples/CMakeFiles/gs_test.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/kth/ros2_ws/src/YDLidar-SDK/build/temp.linux-x86_64-3.10/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object examples/CMakeFiles/gs_test.dir/gs_test.cpp.o"
	cd /home/kth/ros2_ws/src/YDLidar-SDK/build/temp.linux-x86_64-3.10/examples && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT examples/CMakeFiles/gs_test.dir/gs_test.cpp.o -MF CMakeFiles/gs_test.dir/gs_test.cpp.o.d -o CMakeFiles/gs_test.dir/gs_test.cpp.o -c /home/kth/ros2_ws/src/YDLidar-SDK/examples/gs_test.cpp

examples/CMakeFiles/gs_test.dir/gs_test.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/gs_test.dir/gs_test.cpp.i"
	cd /home/kth/ros2_ws/src/YDLidar-SDK/build/temp.linux-x86_64-3.10/examples && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/kth/ros2_ws/src/YDLidar-SDK/examples/gs_test.cpp > CMakeFiles/gs_test.dir/gs_test.cpp.i

examples/CMakeFiles/gs_test.dir/gs_test.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/gs_test.dir/gs_test.cpp.s"
	cd /home/kth/ros2_ws/src/YDLidar-SDK/build/temp.linux-x86_64-3.10/examples && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/kth/ros2_ws/src/YDLidar-SDK/examples/gs_test.cpp -o CMakeFiles/gs_test.dir/gs_test.cpp.s

# Object files for target gs_test
gs_test_OBJECTS = \
"CMakeFiles/gs_test.dir/gs_test.cpp.o"

# External object files for target gs_test
gs_test_EXTERNAL_OBJECTS =

gs_test: examples/CMakeFiles/gs_test.dir/gs_test.cpp.o
gs_test: examples/CMakeFiles/gs_test.dir/build.make
gs_test: libydlidar_sdk.a
gs_test: examples/CMakeFiles/gs_test.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/kth/ros2_ws/src/YDLidar-SDK/build/temp.linux-x86_64-3.10/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../gs_test"
	cd /home/kth/ros2_ws/src/YDLidar-SDK/build/temp.linux-x86_64-3.10/examples && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/gs_test.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
examples/CMakeFiles/gs_test.dir/build: gs_test
.PHONY : examples/CMakeFiles/gs_test.dir/build

examples/CMakeFiles/gs_test.dir/clean:
	cd /home/kth/ros2_ws/src/YDLidar-SDK/build/temp.linux-x86_64-3.10/examples && $(CMAKE_COMMAND) -P CMakeFiles/gs_test.dir/cmake_clean.cmake
.PHONY : examples/CMakeFiles/gs_test.dir/clean

examples/CMakeFiles/gs_test.dir/depend:
	cd /home/kth/ros2_ws/src/YDLidar-SDK/build/temp.linux-x86_64-3.10 && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/kth/ros2_ws/src/YDLidar-SDK /home/kth/ros2_ws/src/YDLidar-SDK/examples /home/kth/ros2_ws/src/YDLidar-SDK/build/temp.linux-x86_64-3.10 /home/kth/ros2_ws/src/YDLidar-SDK/build/temp.linux-x86_64-3.10/examples /home/kth/ros2_ws/src/YDLidar-SDK/build/temp.linux-x86_64-3.10/examples/CMakeFiles/gs_test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : examples/CMakeFiles/gs_test.dir/depend

