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

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
find_package(Eigen)

#Beginer - JOINT

rosbuild_add_executable(move_joint_r1 src/move_joint_r1.cpp)

rosbuild_add_executable(move_above_table_r1 src/move_above_table_r1.cpp)
rosbuild_link_boost(move_above_table_r1 signals)

rosbuild_add_executable(send_traj src/send_traj.cpp)
rosbuild_add_executable(circleTraj src/circleTraj.cpp)
target_link_libraries(circleTraj clopema_move)
rosbuild_add_boost_directories()

rosbuild_add_executable(rsp src/main.cpp)
rosbuild_link_boost(rsp signals)

rosbuild_add_executable(xtion3Calibration src/xtion3Calibration.cpp)
target_link_libraries(xtion3Calibration clopema_move)
rosbuild_link_boost(xtion3Calibration signals)

rosbuild_add_executable(rotate_and_grab src/rotate_and_grab.cpp)
rosbuild_link_boost(rotate_and_grab signals)

rosbuild_add_executable(plan_avoiding_cloth src/plan_avoiding_cloth.cpp)

rosbuild_add_executable(calibrate_fixed_xtion src/calibrate_fixed_xtion.cpp)
rosbuild_link_boost(calibrate_fixed_xtion signals)

rosbuild_add_executable(find_lowest src/find_lowest.cpp src/lowestPoint2.cpp)

rosbuild_add_executable(cloth_simulator src/cloth_simulator.cpp)

rosbuild_add_executable(grippers_testing src/grippers_testing.cpp)

include_directories(${Eigen_INCLUDE_DIRS})

#rosbuild_add_executable(test_calib src/test_calib.cpp src/lowestPoint)
#target_link_libraries(test_calib clopema_move)

#rosbuild_add_executable(test_ src/test_.cpp )



#Intermediate - JOINT
rosbuild_add_executable(move_home src/move_home.cpp)
rosbuild_add_executable(move_joint_arms src/move_joint_arms.cpp)
rosbuild_add_executable(move_joint_ext src/move_joint_ext.cpp)

target_link_libraries(move_home clopema_move)
target_link_libraries(move_joint_arms clopema_move)
target_link_libraries(move_joint_ext clopema_move)

#Intermediate - POSE
rosbuild_add_executable(move_pose_r2 src/move_pose_r2.cpp)
rosbuild_add_executable(move_pose_arms src/move_pose_arms.cpp)

target_link_libraries(move_pose_r2 clopema_move)
target_link_libraries(move_pose_arms clopema_move)

#Advanced
rosbuild_add_executable(more_points src/more_points.cpp)

target_link_libraries(more_points clopema_move)

#Vision
rosbuild_add_executable(plan_out_of_vision src/plan_out_of_vision.cpp)

#Experimental
rosbuild_add_executable(move_constraints_r1 src/move_constraints_r1.cpp)
rosbuild_add_executable(move_constraints_arms src/move_constraints_arms.cpp)
rosbuild_add_executable(not_tearing src/not_tearing.cpp)
rosbuild_add_executable(pick_up src/pick_up.cpp)
rosbuild_add_executable(interpolation src/interpolation.cpp)

rosbuild_add_executable(planner src/planner.cpp)
rosbuild_add_executable(gui src/gui.cpp)

target_link_libraries(move_constraints_r1 clopema_move)
target_link_libraries(move_constraints_arms clopema_move)
target_link_libraries(planner clopema_move)
target_link_libraries(gui clopema_move)
target_link_libraries(not_tearing clopema_move)
target_link_libraries(pick_up clopema_move)
target_link_libraries(interpolation clopema_move)


rosbuild_add_executable(shake_it src/shake_it.cpp)
target_link_libraries(shake_it clopema_move)

