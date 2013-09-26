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

#rosbuild_add_boost_directories()

rosbuild_add_executable(unfolding src/unfolding.cpp src/utils.cpp src/grasp_from_floor.cpp)
rosbuild_add_executable(grasp_two_times_lowest_point src/grasp_two_times_lowest_point.cpp src/utils.cpp )

#rosbuild_add_executable(partial_unfold src/partial_unfolding.cpp src/utils.cpp src/fold_detector/fold_detector.cpp src/fold_detector/call_main.cpp src/fold_detector/choose_a_corner.cpp src/fold_detector/canny_image.cpp src/fold_detector/true_pixels.cpp src/fold_detector/thinning.cpp src/fold_detector/following_edges.cpp src/fold_detector/colors.cpp src/fold_detector/connect2_junctions src/fold_detector/a_corner_check.cpp src/fold_detector/douglas_peucker.cpp src/fold_detector/edges_in_detail.cpp src/fold_detector/follow.cpp src/fold_detector/unify_junctions.cpp src/fold_detector/tiding_up.cpp src/fold_detector/check_depths.cpp src/fold_detector/a_corner_depths.cpp src/fold_detector/check_disconnected.cpp src/fold_detector/normal_distance.cpp src/fold_detector/grasp_point.cpp)
#rosbuild_link_boost(partial_unfold signals)

rosbuild_add_executable(partial_unfold2
    src/partial_unfolding2.cpp
    src/RotateAndGrab.cpp

    src/fold_detector/fold_detector.cpp
    src/fold_detector/call_main.cpp
    src/fold_detector/choose_a_corner.cpp
    src/fold_detector/canny_image.cpp
    src/fold_detector/true_pixels.cpp
    src/fold_detector/thinning.cpp
    src/fold_detector/following_edges.cpp
    src/fold_detector/colors.cpp
    src/fold_detector/connect2_junctions.cpp
    src/fold_detector/a_corner_check.cpp
    src/fold_detector/douglas_peucker.cpp
    src/fold_detector/edges_in_detail.cpp
    src/fold_detector/follow.cpp
    src/fold_detector/unify_junctions.cpp
    src/fold_detector/tiding_up.cpp
    src/fold_detector/check_depths.cpp
    src/fold_detector/a_corner_depths.cpp
    src/fold_detector/check_disconnected.cpp
    src/fold_detector/normal_distance.cpp
    src/fold_detector/grasp_point.cpp
)
rosbuild_link_boost(partial_unfold2 signals)

#rosbuild_add_executable(grasp_from_floor src/grasp_from_floor.cpp src/utils.cpp)



