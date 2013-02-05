cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)
set(ROS_BUILD_TYPE Release)

#add_definitions (-O3 -funroll-loops -DNDEBUG -O3 -msse3 -mssse3)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


# Add a ROS C++ macro to tell use ROS logging instead of printf and cout/cerr
add_definitions(-DOCTOMAP_ROS)

#
# math:
#
SET (octomath_SRCS
  src/octomap/math/Vector3.cpp
  src/octomap/math/Quaternion.cpp
  src/octomap/math/Pose6D.cpp
)

rosbuild_add_library( octomath ${octomath_SRCS})

#
# octomap library:
#
SET (octomap_SRCS
  src/octomap/Pointcloud.cpp
  src/octomap/ScanGraph.cpp
  src/octomap/CountingOcTree.cpp
  src/octomap/OcTree.cpp
  src/octomap/OcTreeStamped.cpp
  src/octomap/OcTreeNode.cpp
  src/octomap/OcTreeNodeStamped.cpp
#  src/octomap/OcTreeNodeLabeled.cpp
#  src/octomap/OcTreeLabeled.cpp		# ignore labeled for now...
  src/octomap/OcTreeLUT.cpp
)




rosbuild_add_library(octomap ${octomap_SRCS})
TARGET_LINK_LIBRARIES(octomap octomath)


#
# octomap binaries:
#

rosbuild_add_executable(graph2tree src/octomap/graph2tree.cpp)
TARGET_LINK_LIBRARIES(graph2tree octomap)

rosbuild_add_executable(log2graph src/octomap/log2graph.cpp)
TARGET_LINK_LIBRARIES(log2graph octomap)

rosbuild_add_executable(binvox2bt src/octomap/binvox2bt.cpp)
TARGET_LINK_LIBRARIES(binvox2bt octomap)

rosbuild_add_executable(bt2vrml src/octomap/bt2vrml.cpp)
TARGET_LINK_LIBRARIES(bt2vrml octomap)

rosbuild_add_executable(edit_octree src/octomap/edit_octree.cpp)
TARGET_LINK_LIBRARIES(edit_octree octomap)

rosbuild_add_executable(simple src/octomap/simple.cpp)
TARGET_LINK_LIBRARIES(simple octomap)

#rosbuild_add_executable(labeled_demo src/octomap/labeled_demo.cpp)
#TARGET_LINK_LIBRARIES(labeled_demo octomap)

rosbuild_add_executable(test_raycasting src/octomap/test_raycasting.cpp)
TARGET_LINK_LIBRARIES(test_raycasting octomap)

rosbuild_add_executable(test_scans src/octomap/test_scans.cpp)
TARGET_LINK_LIBRARIES(test_scans octomap)

rosbuild_add_executable(test_iterators src/octomap/test_iterators.cpp)
TARGET_LINK_LIBRARIES(test_iterators octomap)
