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
CMAKE_SOURCE_DIR = /home/magnate/ros/nodelet_core/nodelet

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/magnate/ros/nodelet_core/nodelet/build

# Include any dependencies generated for this target.
include CMakeFiles/nodelet.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/nodelet.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/nodelet.dir/flags.make

CMakeFiles/nodelet.dir/src/nodelet.o: CMakeFiles/nodelet.dir/flags.make
CMakeFiles/nodelet.dir/src/nodelet.o: ../src/nodelet.cpp
CMakeFiles/nodelet.dir/src/nodelet.o: ../manifest.xml
CMakeFiles/nodelet.dir/src/nodelet.o: /opt/ros/electric/ros/tools/rospack/manifest.xml
CMakeFiles/nodelet.dir/src/nodelet.o: /opt/ros/electric/ros/core/roslib/manifest.xml
CMakeFiles/nodelet.dir/src/nodelet.o: /opt/ros/electric/stacks/ros_comm/utilities/cpp_common/manifest.xml
CMakeFiles/nodelet.dir/src/nodelet.o: /opt/ros/electric/stacks/ros_comm/utilities/rostime/manifest.xml
CMakeFiles/nodelet.dir/src/nodelet.o: /opt/ros/electric/stacks/ros_comm/tools/rosconsole/manifest.xml
CMakeFiles/nodelet.dir/src/nodelet.o: /opt/ros/electric/stacks/pluginlib/manifest.xml
CMakeFiles/nodelet.dir/src/nodelet.o: /opt/ros/electric/ros/core/rosbuild/manifest.xml
CMakeFiles/nodelet.dir/src/nodelet.o: /opt/ros/electric/ros/core/roslang/manifest.xml
CMakeFiles/nodelet.dir/src/nodelet.o: /opt/ros/electric/stacks/ros_comm/clients/cpp/roscpp_traits/manifest.xml
CMakeFiles/nodelet.dir/src/nodelet.o: /opt/ros/electric/stacks/ros_comm/clients/cpp/roscpp_serialization/manifest.xml
CMakeFiles/nodelet.dir/src/nodelet.o: /opt/ros/electric/stacks/ros_comm/utilities/xmlrpcpp/manifest.xml
CMakeFiles/nodelet.dir/src/nodelet.o: /opt/ros/electric/stacks/ros_comm/messages/std_msgs/manifest.xml
CMakeFiles/nodelet.dir/src/nodelet.o: /opt/ros/electric/stacks/ros_comm/messages/rosgraph_msgs/manifest.xml
CMakeFiles/nodelet.dir/src/nodelet.o: /opt/ros/electric/stacks/ros_comm/clients/cpp/roscpp/manifest.xml
CMakeFiles/nodelet.dir/src/nodelet.o: /opt/ros/electric/stacks/ros_comm/clients/rospy/manifest.xml
CMakeFiles/nodelet.dir/src/nodelet.o: /opt/ros/electric/stacks/bond_core/bond/manifest.xml
CMakeFiles/nodelet.dir/src/nodelet.o: /opt/ros/electric/stacks/bond_core/smclib/manifest.xml
CMakeFiles/nodelet.dir/src/nodelet.o: /opt/ros/electric/stacks/bond_core/bondcpp/manifest.xml
CMakeFiles/nodelet.dir/src/nodelet.o: /opt/ros/electric/stacks/ros_comm/messages/std_msgs/msg_gen/generated
CMakeFiles/nodelet.dir/src/nodelet.o: /opt/ros/electric/stacks/ros_comm/messages/rosgraph_msgs/msg_gen/generated
CMakeFiles/nodelet.dir/src/nodelet.o: /opt/ros/electric/stacks/ros_comm/clients/cpp/roscpp/msg_gen/generated
CMakeFiles/nodelet.dir/src/nodelet.o: /opt/ros/electric/stacks/ros_comm/clients/cpp/roscpp/srv_gen/generated
CMakeFiles/nodelet.dir/src/nodelet.o: /opt/ros/electric/stacks/bond_core/bond/msg_gen/generated
	$(CMAKE_COMMAND) -E cmake_progress_report /home/magnate/ros/nodelet_core/nodelet/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/nodelet.dir/src/nodelet.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -o CMakeFiles/nodelet.dir/src/nodelet.o -c /home/magnate/ros/nodelet_core/nodelet/src/nodelet.cpp

CMakeFiles/nodelet.dir/src/nodelet.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/nodelet.dir/src/nodelet.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -E /home/magnate/ros/nodelet_core/nodelet/src/nodelet.cpp > CMakeFiles/nodelet.dir/src/nodelet.i

CMakeFiles/nodelet.dir/src/nodelet.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/nodelet.dir/src/nodelet.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -S /home/magnate/ros/nodelet_core/nodelet/src/nodelet.cpp -o CMakeFiles/nodelet.dir/src/nodelet.s

CMakeFiles/nodelet.dir/src/nodelet.o.requires:
.PHONY : CMakeFiles/nodelet.dir/src/nodelet.o.requires

CMakeFiles/nodelet.dir/src/nodelet.o.provides: CMakeFiles/nodelet.dir/src/nodelet.o.requires
	$(MAKE) -f CMakeFiles/nodelet.dir/build.make CMakeFiles/nodelet.dir/src/nodelet.o.provides.build
.PHONY : CMakeFiles/nodelet.dir/src/nodelet.o.provides

CMakeFiles/nodelet.dir/src/nodelet.o.provides.build: CMakeFiles/nodelet.dir/src/nodelet.o
.PHONY : CMakeFiles/nodelet.dir/src/nodelet.o.provides.build

# Object files for target nodelet
nodelet_OBJECTS = \
"CMakeFiles/nodelet.dir/src/nodelet.o"

# External object files for target nodelet
nodelet_EXTERNAL_OBJECTS =

../bin/nodelet: CMakeFiles/nodelet.dir/src/nodelet.o
../bin/nodelet: ../lib/libnodeletlib.so
../bin/nodelet: CMakeFiles/nodelet.dir/build.make
../bin/nodelet: CMakeFiles/nodelet.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable ../bin/nodelet"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/nodelet.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/nodelet.dir/build: ../bin/nodelet
.PHONY : CMakeFiles/nodelet.dir/build

CMakeFiles/nodelet.dir/requires: CMakeFiles/nodelet.dir/src/nodelet.o.requires
.PHONY : CMakeFiles/nodelet.dir/requires

CMakeFiles/nodelet.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/nodelet.dir/cmake_clean.cmake
.PHONY : CMakeFiles/nodelet.dir/clean

CMakeFiles/nodelet.dir/depend:
	cd /home/magnate/ros/nodelet_core/nodelet/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/magnate/ros/nodelet_core/nodelet /home/magnate/ros/nodelet_core/nodelet /home/magnate/ros/nodelet_core/nodelet/build /home/magnate/ros/nodelet_core/nodelet/build /home/magnate/ros/nodelet_core/nodelet/build/CMakeFiles/nodelet.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/nodelet.dir/depend
