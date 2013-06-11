#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <clopema_arm_navigation/ClopemaMotionPlan.h>
#include <arm_navigation_msgs/GetRobotState.h>
#include <arm_navigation_msgs/convert_messages.h>
#include <clopema_motoros/SetPowerOff.h>
#include <clopema_motoros/WriteIO.h>
#include <tf/transform_listener.h>

//-----openni----
#include <ros/ros.h>

#include <certh_libs/OpenniCapture.h>

// OpenCV includes-----
#include <opencv2/highgui/highgui.hpp>
using namespace std ;

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>
namespace enc = sensor_msgs::image_encodings;

using namespace std;


class OpenniGrabber {

public:
    OpenniGrabber()
    {

         client = nh.serviceClient<certh_libs::OpenniCapture>("capture");
    }

    bool grab(cv::Mat &rgb, cv::Mat &depth, pcl::PointCloud<pcl::PointXYZ>
&pc)
    {
        certh_libs::OpenniCapture srv;

        if (client.call(srv))
        {
            if ( srv.response.success )
            {

                // Save stamps for info
                ros::Time stamp_rgb = srv.response.rgb.header.stamp;
                ros::Time stamp_depth = srv.response.depth.header.stamp;
                ros::Time stamp_cloud = srv.response.cloud.header.stamp;

                cv_bridge::CvImagePtr rgb_ = cv_bridge::toCvCopy(srv.response.rgb, enc::BGR8);
                cv_bridge::CvImagePtr depth_ =cv_bridge::toCvCopy(srv.response.depth, "");

                pcl::fromROSMsg(srv.response.cloud, pc) ;

                rgb = rgb_->image ;
                depth = depth_->image ;

                return true ;
            }
            else return false ;
         }
        else return false ;

    }

private:

    ros::NodeHandle nh ;
    ros::ServiceClient client ;

};
bool getRobotState(arm_navigation_msgs::RobotState & rs) {
	ros::service::waitForService("/environment_server/get_robot_state");
	arm_navigation_msgs::GetRobotState r;
	if (!ros::service::call("/environment_server/get_robot_state", r)) {
		ROS_ERROR("Can't get current robot state.");
		return false;
	}
	rs = r.response.robot_state;
	return true;
}

arm_navigation_msgs::MotionPlanRequest createPlan() {
	arm_navigation_msgs::MotionPlanRequest req;
	req.group_name = "r1_arm";
	req.allowed_planning_time = ros::Duration(5.0);
	std::vector<std::string> names;
	names.push_back("r1_joint_s");
	names.push_back("r1_joint_l");
	names.push_back("r1_joint_u");
	names.push_back("r1_joint_r");
	names.push_back("r1_joint_b");
	names.push_back("r1_joint_t");
	req.goal_constraints.joint_constraints.resize(names.size());
	for (unsigned int i = 0; i < req.goal_constraints.joint_constraints.size(); ++i) {
		req.goal_constraints.joint_constraints[i].joint_name = names[i];
		req.goal_constraints.joint_constraints[i].tolerance_below = 0.1;
		req.goal_constraints.joint_constraints[i].tolerance_above = 0.1;
		
	}



	req.goal_constraints.joint_constraints[0].position = (0.1/8223)*45435;
	req.goal_constraints.joint_constraints[1].position = (0.1/7450)*45861;
	req.goal_constraints.joint_constraints[2].position = (0.1/7887)*89489;
	req.goal_constraints.joint_constraints[3].position = (0.1/5704)*14728;
	req.goal_constraints.joint_constraints[4].position = (0.1/4720)*(-30711);
	req.goal_constraints.joint_constraints[5].position = (3.14/-63046)*(-70065);
	
//~ old ones
	//~ req.goal_constraints.joint_constraints[0].position = (0.1/8223)*36677;
	//~ req.goal_constraints.joint_constraints[1].position = (0.1/7450)*54346;
	//~ req.goal_constraints.joint_constraints[2].position = (0.1/7887)*82754;
	//~ req.goal_constraints.joint_constraints[3].position = (0.1/5704)*12122;
	//~ req.goal_constraints.joint_constraints[4].position = (0.1/4720)*(-21262);
	//~ req.goal_constraints.joint_constraints[5].position = (3.14/-63046)*(-70065);


return req;
}

arm_navigation_msgs::MotionPlanRequest createRotation(double tAxis) {
	arm_navigation_msgs::MotionPlanRequest req;
	req.group_name = "r1_arm";
	req.allowed_planning_time = ros::Duration(5.0);
	std::vector<std::string> names;
	names.push_back("r1_joint_s");
	names.push_back("r1_joint_l");
	names.push_back("r1_joint_u");
	names.push_back("r1_joint_r");
	names.push_back("r1_joint_b");
	names.push_back("r1_joint_t");
	req.goal_constraints.joint_constraints.resize(names.size());
	for (unsigned int i = 0; i < req.goal_constraints.joint_constraints.size(); ++i) {
		req.goal_constraints.joint_constraints[i].joint_name = names[i];
		req.goal_constraints.joint_constraints[i].tolerance_below = 0.1;
		req.goal_constraints.joint_constraints[i].tolerance_above = 0.1;
		
	}

	req.goal_constraints.joint_constraints[0].position = (0.1/8223)*45435;
	req.goal_constraints.joint_constraints[1].position = (0.1/7450)*45861;
	req.goal_constraints.joint_constraints[2].position = (0.1/7887)*89489;
	req.goal_constraints.joint_constraints[3].position = (0.1/5704)*14728;
	req.goal_constraints.joint_constraints[4].position = (0.1/4720)*(-30711);
	req.goal_constraints.joint_constraints[5].position = (3.14/-63046)*(70065);

	
return req;
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "move_joint_r1");
	ros::NodeHandle nh;

	clopema_arm_navigation::ClopemaMotionPlan mp;

	mp.request.motion_plan_req = createPlan();
	if (!getRobotState(mp.request.motion_plan_req.start_state)) {
		ROS_ERROR("Can't get current robot state");
		return -1;
	}

	ros::service::waitForService("/clopema_planner/plan");
	if (!ros::service::call("/clopema_planner/plan", mp)) {
		ROS_ERROR("Can't call service clopema_planner/plan");
		return -1;
	}

	if (mp.response.error_code.val == arm_navigation_msgs::ArmNavigationErrorCodes::SUCCESS) {
		actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> move("/clopema_controller/follow_joint_trajectory", true);
		move.waitForServer();
		control_msgs::FollowJointTrajectoryGoal goal;
		goal.trajectory = mp.response.joint_trajectory;
		move.sendGoal(goal);
		bool finished_within_time = move.waitForResult(ros::Duration(45.0));
		if (!finished_within_time) {
			move.cancelGoal();
			ROS_INFO("Timed out achieving goal");
		} else {
			actionlib::SimpleClientGoalState state = move.getState();
			bool success = (state == actionlib::SimpleClientGoalState::SUCCEEDED);
			if (success)
				ROS_INFO("Action finished: %s", state.toString().c_str());
			else {
				ROS_INFO("Action failed: %s", state.toString().c_str());
				ROS_WARN("Addition information: %s", state.text_.c_str());
				control_msgs::FollowJointTrajectoryResult r;
				ROS_WARN("Error code: %d", move.getResult()->error_code);
			}
		}
	} else {
		ROS_WARN("Can't plan trajectory: %s", arm_navigation_msgs::armNavigationErrorCodeToString(mp.response.error_code).data());
	}
	
	
// Making the rotation 	

	mp.request.motion_plan_req = createRotation(3.15);
	if (!getRobotState(mp.request.motion_plan_req.start_state)) {
		ROS_ERROR("Can't get current robot state");
		return -1;
	}
	
	ros::service::waitForService("/clopema_planner/plan");
	if (!ros::service::call("/clopema_planner/plan", mp)) {
		ROS_ERROR("Can't call service clopema_planner/plan");
		return -1;
	}

	if (mp.response.error_code.val == arm_navigation_msgs::ArmNavigationErrorCodes::SUCCESS) {
		actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> move("/clopema_controller/follow_joint_trajectory", true);
		
		move.waitForServer();
		control_msgs::FollowJointTrajectoryGoal goal;
		goal.trajectory = mp.response.joint_trajectory;
		move.sendGoal(goal);
		bool finished_within_time = move.waitForResult(ros::Duration(45.0));
		if (!finished_within_time) {
			move.cancelGoal();
			ROS_INFO("Timed out achieving goal");
		} else {
			actionlib::SimpleClientGoalState state = move.getState();
			bool success = (state == actionlib::SimpleClientGoalState::SUCCEEDED);
			if (success)
				ROS_INFO("Action finished: %s", state.toString().c_str());
			else {
				ROS_INFO("Action failed: %s", state.toString().c_str());
				ROS_WARN("Addition information: %s", state.text_.c_str());
				control_msgs::FollowJointTrajectoryResult r;
				ROS_WARN("Error code: %d", move.getResult()->error_code);
			}
		}
	} else {
		ROS_WARN("Can't plan trajectory: %s", arm_navigation_msgs::armNavigationErrorCodeToString(mp.response.error_code).data());
	}
	//~ 
	
	tf::TransformListener listener;
    tf::StampedTransform transform;
    
	try {
		listener.waitForTransform("base_link", "r1_ee", ros::Time(0), ros::Duration(10.0) );
		listener.lookupTransform("base_link", "r1_ee", ros::Time(0), transform);
	} catch (tf::TransformException ex) {
		ROS_ERROR("%s",ex.what());
	}

    
	cout<<"------->"<< transform.getOrigin().x()<< "\n " << "------->"<<transform.getOrigin().y()<< "\n"<<"------->" <<transform.getOrigin().z() << "\n";


//Set servo power off
	clopema_motoros::SetPowerOff soff;
	soff.request.force = false;
	ros::service::waitForService("/joint_trajectory_action/set_power_off");
	if (!ros::service::call("/joint_trajectory_action/set_power_off", soff)) {
		ROS_ERROR("Can't call service set_power_off");
		return -1;
	}
	return 1;
}
