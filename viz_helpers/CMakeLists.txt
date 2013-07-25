cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Debug)

rosbuild_init()
rosbuild_genmsg()
rosbuild_gensrv()

FIND_PACKAGE(Qt4 REQUIRED)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

INCLUDE(${CMAKE_CURRENT_SOURCE_DIR}/CMake/qt.cmake)
include_directories(${CMAKE_CURRENT_BINARY_DIR})

SET (CAMERA_VIEW_HEADERS
	src/CameraView.h
	src/CameraWidget.h
	src/QSingleItemSquareLayout.h
	src/Dashboard.h
	src/Commander.h
)

SET (CAMERA_VIEW_SOURCES
	src/CameraView.cpp
	src/CameraWidget.cpp
	src/QSingleItemSquareLayout.cpp
	src/CameraViewServer.cpp
	src/Dashboard.cpp
	src/CameraViewPlugin.cpp
	src/Commander.cpp
	src/CommandPanelServer.cpp
)

ADD_QT_LIBRARY(viz_helpers SOURCES ${CAMERA_VIEW_SOURCES} HEADERS ${CAMERA_VIEW_HEADERS} FORMS src/Dashboard.ui)

#######################################################################3

ADD_QT_EXECUTABLE(camera_viewer SOURCES src/camera_viewer.cpp)
target_link_libraries(camera_viewer viz_helpers)

###############################################################################

ADD_QT_EXECUTABLE(dashboard SOURCES src/dashboard.cpp)
target_link_libraries(dashboard viz_helpers)


###############################################################################

ADD_QT_EXECUTABLE(commander SOURCES src/commander.cpp)
target_link_libraries(commander viz_helpers)

###############################################################################

FIND_PACKAGE(Bullet)

rosbuild_add_executable(cloth_simulator src/ClothSimulatorServer.cpp src/Physics.cpp)
target_link_libraries(cloth_simulator BulletSoftBody)

