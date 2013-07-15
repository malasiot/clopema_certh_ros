#ifndef __ROBOT_HELPERS_ROBOT_H__
#define __ROBOT_HELPERS_ROBOT_H__

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <clopema_motoros/SetPowerOff.h>
#include <arm_navigation_msgs/GetRobotState.h>
#include <clopema_arm_navigation/ClopemaMotionPlan.h>
#include <arm_navigation_msgs/convert_messages.h>
#include <planning_environment/models/collision_models.h>
#include <boost/signals.hpp>
#include <Eigen/Geometry>

namespace robot_helpers {


class MoveRobot {

public:
	MoveRobot();
	virtual ~MoveRobot();

    // execute the trajectory. optionally leave the servos open when finished
    bool execTrajectory(const trajectory_msgs::JointTrajectory &traj) ;

    /*
     * Send goal to the server and wait for result for maximum 30 sec.
     */
    bool doGoal(const control_msgs::FollowJointTrajectoryGoal & goal);

    // set this true to close servos after each movement (default)

    void setServoMode(bool closeWhenDone) {
        closeServoWhenDone = closeWhenDone ;
    }

	/*
	 * Function calls service to turn power off
	 * \param force - turn servo power immediately
	 * \return false if error occurred
	 */
	bool setServoPowerOff(bool force);

    void reset() ;

public:

    // connect these signal to callback functions if need to be

    boost::signal<void ()> actionCompleted ;
    boost::signal<void ()> actionStarted ;

private:

    typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ClopemaMoveClient;

    void activeCb() ;

    void doneCb(const actionlib::SimpleClientGoalState& state, const control_msgs::FollowJointTrajectoryResultConstPtr & result);

	ClopemaMoveClient cmc_;
	const std::string getServerName() {
		return "/clopema_controller/follow_joint_trajectory";
	}

    bool closeServoWhenDone ;
} ;

/*
	 * This function will plan and filter trajectory for request specified in mp param
	 * \return false if plan can't be created
	 */
bool plan(clopema_arm_navigation::ClopemaMotionPlan & mp);

/*
	 * Return all joints in group specified in first argument
	 * Allowed group: r1_arm, r2_arm, arms, ext_axis
 */

std::vector<std::string> getJointsInGroup(std::string group);

/*
	 * Convert pose to the motion plan as a goal constraints
	*/
void poseToClopemaMotionPlan(clopema_arm_navigation::ClopemaMotionPlan & mp, arm_navigation_msgs::SimplePoseConstraint & pose);

/*
	 * Function set robot state (param) to be equal with actual robot position
	 * \param RobotState - output robot state
	 * \return false if error occurred
*/
bool getRobotState(arm_navigation_msgs::RobotState & rs);

// get the pose of the end effector

Eigen::Affine3d getPose(const std::string &armName, const std::string &base_link = std::string(), const ros::Time &ts = ros::Time(0)) ;

// get current pose
Eigen::Affine3d getCurrentPose(const std::string &armName) ;

// set robot speed
bool setRobotSpeed(float speed) ;


// set/get gripper state. Return false on error

bool setGripperState(const std::string &armName, bool open = true) ;
bool getGripperState(const std::string &armName, bool &open) ;


}

#endif
