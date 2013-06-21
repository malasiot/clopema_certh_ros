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

FIND_PACKAGE(Qt4 REQUIRED)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

INCLUDE(${QT_USE_FILE})

SET (CAMERA_VIEW_HEADERS
	src/CameraView.h
	src/CameraWidget.h
	src/QSingleItemSquareLayout.h

)

SET (CAMERA_VIEW_SOURCES
	src/CameraView.cpp
	src/CameraWidget.cpp
	src/QSingleItemSquareLayout.cpp
	src/CameraViewServer.cpp
)

QT4_WRAP_CPP(CAMERA_VIEW_MOC_SOURCES ${CAMERA_VIEW_HEADERS} )
QT4_WRAP_UI(CAMERA_VIEW_UI_HEADERS ${CAMERA_VIEW_FORMS} )
QT4_ADD_RESOURCES(CAMERA_VIEW_RC_SOURCES ${CAMERA_VIEW_RESOURCES} )

rosbuild_add_library(camera_view ${CAMERA_VIEW_SOURCES} ${CAMERA_VIEW_MOC_SOURCES} ${CAMERA_VIEW_RC_SOURCES} ${CAMERA_VIEW_UI_HEADERS})
target_link_libraries(camera_view ${QT_LIBRARIES})

#######################################################################3

SET (CAMERA_VIEWER_HEADERS
)

SET (CAMERA_VIEWER_SOURCES
	src/camera_viewer.cpp
)

QT4_WRAP_CPP(CAMERA_VIEWER_MOC_SOURCES ${CAMERA_VIEWER_HEADERS} )
QT4_WRAP_UI(CAMERA_VIEWER_UI_HEADERS ${CAMERA_VIEWER_FORMS} )
QT4_ADD_RESOURCES(CAMERA_VIEWER_RC_SOURCES ${CAMERA_VIEWER_RESOURCES} )

rosbuild_add_executable(camera_viewer ${CAMERA_VIEWER_SOURCES} ${CAMERA_VIEWER_MOC_SOURCES} ${CAMERA_VIEWER_RC_SOURCES} ${CAMERA_VIEWER_UI_HEADERS})
target_link_libraries(camera_viewer camera_view ${QT_LIBRARIES})

###############################################################################

SET (CAMERA_VIEW_PLUGIN_HEADERS

)

SET (CAMERA_VIEW_PLUGIN_SOURCES
	src/CameraViewPlugin.cpp
)

QT4_WRAP_CPP(CAMERA_VIEW_PLUGIN_MOC_SOURCES ${CAMERA_VIEW_PLUGIN_HEADERS} )
QT4_WRAP_UI(CAMERA_VIEW_PLUGIN_UI_HEADERS ${CAMERA_VIEW_PLUGIN_FORMS} )
QT4_ADD_RESOURCES(CAMERA_VIEW_PLUGIN_RC_SOURCES ${CAMERA_VIEW_PLUGIN_RESOURCES} )

rosbuild_add_library(camera_view_plugin ${CAMERA_VIEW_PLUGIN_SOURCES} ${CAMERA_VIEW_PLUGIN_MOC_SOURCES} ${CAMERA_VIEW_PLUGIN_RC_SOURCES} ${CAMERA_VIEW_PLUGIN_HEADERS})
target_link_libraries(camera_view_plugin camera_view ${QT_LIBRARIES})


