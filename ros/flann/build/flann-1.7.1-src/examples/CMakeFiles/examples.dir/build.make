# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canoncical targets will work.
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
CMAKE_SOURCE_DIR = /home/magnate/ros/flann/build/flann-1.7.1-src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/magnate/ros/flann/build/flann-1.7.1-src

# Utility rule file for examples.

examples/CMakeFiles/examples:

examples: examples/CMakeFiles/examples
examples: examples/CMakeFiles/examples.dir/build.make
.PHONY : examples

# Rule to build all files generated by this target.
examples/CMakeFiles/examples.dir/build: examples
.PHONY : examples/CMakeFiles/examples.dir/build

examples/CMakeFiles/examples.dir/clean:
	cd /home/magnate/ros/flann/build/flann-1.7.1-src/examples && $(CMAKE_COMMAND) -P CMakeFiles/examples.dir/cmake_clean.cmake
.PHONY : examples/CMakeFiles/examples.dir/clean

examples/CMakeFiles/examples.dir/depend:
	cd /home/magnate/ros/flann/build/flann-1.7.1-src && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/magnate/ros/flann/build/flann-1.7.1-src /home/magnate/ros/flann/build/flann-1.7.1-src/examples /home/magnate/ros/flann/build/flann-1.7.1-src /home/magnate/ros/flann/build/flann-1.7.1-src/examples /home/magnate/ros/flann/build/flann-1.7.1-src/examples/CMakeFiles/examples.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : examples/CMakeFiles/examples.dir/depend

