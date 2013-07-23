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

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

find_package(Eigen REQUIRED)
find_package(OpenCV REQUIRED)

rosbuild_add_boost_directories()

rosbuild_add_library(certh_libs_geometry 
	src/geometry/TriangleMesh2D.cpp
	src/geometry/Line2D.cpp
	src/geometry/Polygon2D.cpp
	src/geometry/PolygonScanIterator.cpp
	src/geometry/triangle.c
	src/geometry/triangle.h
	src/geometry/Triangle2D.cpp
	src/geometry/TriangleMesh2D.cpp
	src/geometry/TriangleMeshTopology.cpp
)

target_link_libraries(certh_libs_geometry ${Eigen_LIBRARIES})

rosbuild_add_library(certh_libs_imageproc
	src/image/cvHelpers.cpp
)


