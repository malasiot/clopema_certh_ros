/*
 * ClopemaMove.h
 * Wrapper for useful functions used in clopema_planning_tutorials package
 *  Created on: Aug 10, 2012
 *      Author: Vladimír Petrík
 */

#ifndef CLOPEMAMOVE_H_
#define CLOPEMAMOVE_H_

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <clopema_motoros/SetPowerOff.h>
#include <arm_navigation_msgs/GetRobotState.h>
#include <clopema_arm_navigation/ClopemaMotionPlan.h>
#include <arm_navigation_msgs/convert_messages.h>
#include <planning_environment/models/collision_models.h>

class ClopemaMove {
	typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ClopemaMoveClient;

public:
	ClopemaMove();
	virtual ~ClopemaMove();

	/*
	 * Send goal to the server and wait for result for maximum 30 sec.
	 */
	void doGoal(const control_msgs::FollowJointTrajectoryGoal & goal);
	/*
	 * This method is called only once, when goal is done.
	 * The process of execution:
	 * 	1. Print status of the goal (result)
	 * 	2. Turn servo power off
	 */
	void doneCb(const actionlib::SimpleClientGoalState& state, const control_msgs::FollowJointTrajectoryResultConstPtr & result);

	/*
	 * Function calls service to turn power off
	 * \param force - turn servo power immediately
	 * \return false if error occurred
	 */
	bool setServoPowerOff(bool force);
	/*
	 * Function set robot state (param) to be equal with actual robot position
	 * \param RobotState - output robot state
	 * \return false if error occurred
	 */
	bool getRobotState(arm_navigation_msgs::RobotState & rs);
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
private:
	ClopemaMoveClient cmc_;
	const std::string getServerName() {
		return "/clopema_controller/follow_joint_trajectory";
	}

};

#endif /* CLOPEMAMOVE_H_ */
