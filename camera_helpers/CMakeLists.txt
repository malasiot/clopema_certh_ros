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
rosbuild_gensrv()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/CMake/"   )

rosbuild_add_boost_directories()

rosbuild_add_library(camera_helpers 
	src/OpenNICapture.cpp 
	src/OpenNIServiceClient.cpp
	src/CollisionMask.cpp
#	src/robot_head.cpp 
	src/depth_filter.cpp)
rosbuild_link_boost(camera_helpers signals)

rosbuild_add_executable(openni_service src/OpenNIService.cpp)
rosbuild_link_boost(openni_service signals)

#rosbuild_add_executable(robot_head_virtual src/robot_head_virtual.cpp)

rosbuild_add_executable(virtual_cam src/virtual_cam_publisher.cpp )
target_link_libraries(virtual_cam "log4cxx")
rosbuild_link_boost(virtual_cam system regex filesystem)


FIND_PACKAGE(OSMesa)
FIND_PACKAGE(OpenGL)

IF ( OSMESA_FOUND )
rosbuild_add_executable(offscreen_render src/OffscreenRenderer.cpp)
target_link_libraries(offscreen_render  ${OSMESA_LIBRARY} ${OPENGL_LIBRARIES})
ENDIF( OSMESA_FOUND )

FIND_PACKAGE(OpenNI2)
IF( OPENNI2_FOUND )
include_directories(${OPENNI2_INCLUDE_DIRS})
rosbuild_add_executable(openni2_camera_node src/OpenNI2Driver.cpp)
target_link_libraries(openni2_camera_node  ${OPENNI2_LIBRARIES})
ENDIF( OPENNI2_FOUND )

### testers

rosbuild_add_executable(test_openni_capture test/test_openni_capture.cpp)
target_link_libraries(test_openni_capture camera_helpers)

#rosbuild_add_executable(test_rh_capture test/test_rh_capture.cpp)
#target_link_libraries(test_rh_capture camera_helpers )

