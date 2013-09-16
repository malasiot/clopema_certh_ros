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

find_package(LAPACK REQUIRED)
find_package(BLAS REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(OMPL REQUIRED ompl)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_add_boost_directories()

rosbuild_add_library(robot_helpers
	src/RobotHelpers.cpp
	src/RobotUtils.cpp
	src/Geometry.cpp
	src/KinematicsInterface.cpp
	src/JointTrajectory.cpp
	src/GoalRegion.cpp
	src/TaskSpace.cpp
	src/Planner.cpp
	src/ikfast_ma1400_plugin.cpp
	src/ikfast_xtion_plugin.cpp
	src/GraspHanging.cpp
	src/SensorPlanning.cpp
)

target_link_libraries(robot_helpers ${LAPACK_LIBRARIES} ${BLAS_LIBRARIES} ${OMPL_LIBRARIES})

rosbuild_add_executable(test_helpers test/test_helpers.cpp)
target_link_libraries(test_helpers robot_helpers "GLU" "OSMesa32")

