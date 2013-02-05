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

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


#
# octovis viewer
#

# TODO: put in own package
SET (CMAKE_MODULE_PATH ${CMAKE_MODULES_PATH} ${PROJECT_SOURCE_DIR}/CMakeModules)

find_package(Qt4 REQUIRED)
set(QT_USE_QTOPENGL TRUE)
set(QT_USE_QTXML TRUE)
# include the files enabled above
include(${QT_USE_FILE})
FIND_PACKAGE(QGLViewer)
#found QGLViewer lib dir
link_directories(${QGLViewer_LIBRARY_DIR})

INCLUDE_DIRECTORIES(${QGLViewer_INCLUDE_DIR})

# sources for octovis library
set(viewerlib_SRCS
	src/octovis/SceneObject.cpp
	src/octovis/PointcloudDrawer.cpp
	src/octovis/OcTreeDrawer.cpp
	src/octovis/TrajectoryDrawer.cpp
)

# sources for viewer binary
set(viewer_SRCS
	src/octovis/main.cpp	
	src/octovis/ViewerGui.cpp
	src/octovis/ViewerWidget.cpp
	src/octovis/ViewerSettings.cpp
	src/octovis/ViewerSettingsPanel.cpp
	src/octovis/ViewerSettingsPanelCamera.cpp
	src/octovis/CameraFollowMode.cpp
)	

#class headers which need MOCs:
SET(viewer_MOC_HDRS
	src/octovis/ViewerGui.h
	src/octovis/ViewerWidget.h
	src/octovis/ViewerSettings.h
	src/octovis/ViewerSettingsPanel.h
	src/octovis/ViewerSettingsPanelCamera.h
	src/octovis/CameraFollowMode.h
)

# generate list of MOC srcs:
QT4_WRAP_CPP(viewer_MOC_SRCS ${viewer_MOC_HDRS})

# let cmake generate ui*.h files from .ui files (Qt Designer):
SET(viewer_UIS
    src/octovis/ViewerGui.ui
    src/octovis/ViewerSettings.ui
    src/octovis/ViewerSettingsPanel.ui
    src/octovis/ViewerSettingsPanelCamera.ui
)
QT4_WRAP_UI(viewer_UIS_H ${viewer_UIS})

# Don't forget to include output directory, otherwise
# the UI file won't be wrapped!
include_directories(${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_BINARY_DIR})


# Resource files (icons, ...)
QT4_ADD_RESOURCES(viewer_RES src/octovis/icons.qrc)

add_library(octovis_library ${viewerlib_SRCS})
target_link_libraries(octovis_library)
set_target_properties(octovis_library PROPERTIES COMPILE_FLAGS "-fPIC")	
set_target_properties(octovis_library PROPERTIES OUTPUT_NAME octovis)

# Now add these generated files to the ADD_EXECUTABLE step
# If this is NOT done, then the ui_*.h files will not be generated
rosbuild_add_executable(octovis ${viewer_SRCS} ${viewer_UIS_H} ${viewer_MOC_SRCS} ${viewer_RES})

# link the target against the Qt libraries. which libraries exactly, is defined by the "include(${QT_USE_FILE})" line above, which sets up this variable.
target_link_libraries(octovis 
    ${QGLViewer_LIBRARIES}
	${QT_LIBRARIES}
	octovis_library
)
