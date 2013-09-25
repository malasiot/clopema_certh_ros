#ifndef __ROBOT_HELPERS_UTILS_H__
#define __ROBOT_HELPERS_UTILS_H__

#include "robot_helpers/Robot.h"
#include <Eigen/Geometry>

namespace robot_helpers {

// plan end-point to reach given pose

bool planArmToPose(const std::string &armName, const Eigen::Vector3d &pos, const Eigen::Quaterniond &q, trajectory_msgs::JointTrajectory &traj) ;

// plan Xtion frame to pose

bool planXtionToPose(const std::string &armName, const Eigen::Vector3d &pos, const Eigen::Quaterniond &q, trajectory_msgs::JointTrajectory &traj) ;

// plan reach specified joint goal
bool planToJointGoal(const std::string &armName, const sensor_msgs::JointState &js, trajectory_msgs::JointTrajectory &traj) ;

// move robot to home position

bool moveHome(MoveRobot &cmove) ;

// move robot arm so that the tip of the gripper is at pos with orientation q

bool moveGripper(MoveRobot &cmove, const std::string &armName, const Eigen::Vector3d &pos, const Eigen::Quaterniond &q) ;

// move robot arm so that the gripper is pointing down and the tip is at the specified location

bool moveGripperPointingDown(MoveRobot &cmove, const std::string &armName, double X, double Y, double Z) ;

// rotate gripper the given amount

bool rotateGripper(MoveRobot &cmove, const std::string &armName, double theta) ;

// We add a collision object as attached object to the gripper. the parent frame is set to the baselink since otherwise the
// cone will rotate as the gripper moves. Thus this function may be used for one arm static or by calling resetCollisionModel, moving the arm,
// and then setting again the collision object. This is a peculiarity of non-rigid attachement.

bool addConeToCollisionModel(const std::string &armName, double length, double radius) ;
bool addSphereToCollisionModel(const std::string &armName, double radius);
bool addBoxToCollisionModel(float x, float y , float z, float sx, float sy , float sz) ;
bool attachBoxToXtionInCollisionModel(std::string armName);

//bool addBoxToCollisionModel(const std::string &armName, double x, double y, double z );

bool resetCollisionModel() ;

// get inverse kinematics solution for the tip of given arm

bool getIK(const std::string &armName, const Eigen::Vector3d pos, const Eigen::Quaterniond &q, sensor_msgs::JointState &state) ;

// get inverse kinematics solution for the Xtion on the given arm (note that this will solve for direction of the viewing axis (the roll is undefined)

bool getIKXtion(const std::string &armName, const Eigen::Vector3d pos, const Eigen::Quaterniond &q, sensor_msgs::JointState &state) ;

// filter trajectory

bool filterTrajectory(const std::string &groupName, const trajectory_msgs::JointTrajectory &traj_in, trajectory_msgs::JointTrajectory &traj_out) ;

////////andreas

float getArmsDistance(std::string frameName = "_ee");

int moveArmConstrains(geometry_msgs::Pose pose, const std::string &armName, float radious);

void setPathConstraints(clopema_arm_navigation::ClopemaMotionPlan & mp, float radious , const std::string &armName,  geometry_msgs::Quaternion q);

void rotateGripper(float angle, const std::string &armName);

int moveArmBetweenSpheres( std::string armName, bool up, geometry_msgs::Pose goalPose);

int moveArm(geometry_msgs::Pose pose, const std::string &armName,  const std::string &frameID = "base_link");

int moveArms( geometry_msgs::Pose pose1, geometry_msgs::Pose pose2,  const std::string &arm1Name = "r1", const std::string &arm2Name = "r2");

int moveArmsNoTearing( geometry_msgs::Pose pose1, geometry_msgs::Pose pose2,  const std::string &arm1Name = "r1", const std::string &arm2Name = "r2", float radious = getArmsDistance()+0.03 );

bool moveHomeArm(const std::string &armName);

int moveArmThrough(std::vector <geometry_msgs::Pose> poses , const std::string &armName);

//void grabFromXtion(cv::Mat rgb, cv::Mat depth, pcl::PointCloud<pcl::PointXYZ> pc);

geometry_msgs::Pose getArmPose( const std::string &armName, const std::string &frameName = "base_link");

Eigen::Matrix4d getTranformationMatrix(const std::string &frameName, const std::string &coordSys = "base_link" );

tf::StampedTransform getTranformation(const std::string &frameName, const std::string &coordSys = "base_link" );

void publishLowestPointMarker(ros::Publisher &vis_pub, const Eigen::Vector3d &p, const Eigen::Vector3d &n);

void publishPointMarker(ros::Publisher &vis_pub, const Eigen::Vector4d &p, int ID);


}

#endif
