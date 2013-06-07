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

include(FindPkgConfig)
pkg_check_modules(OPENNI libopenni)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_add_boost_directories()


include_directories(${OPENNI_INCLUDE_DIRS})

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

rosbuild_add_library(camera src/OpenNICapture.cpp)
rosbuild_link_boost(camera signals)

rosbuild_add_executable(test_openni_capture src/test_openni_capture.cpp)
target_link_libraries(test_openni_capture camera)

rosbuild_add_library(openni_driver2 src/openni_driver.cpp )
rosbuild_link_boost(openni_driver2 regex)
target_link_libraries(openni_driver2 ${OPENNI_LIBRARIES})

rosbuild_add_executable(test_driver src/test_driver.cpp)
target_link_libraries(test_driver openni_driver2)

rosbuild_add_executable(openni_capture src/openni_capture.cpp)
rosbuild_link_boost(openni_capture signals)

rosbuild_add_executable(lowest src/lowestPoint.cpp src/main.cpp)
rosbuild_link_boost(lowest signals)

rosbuild_add_executable(test_openni src/test_openni.cpp)
