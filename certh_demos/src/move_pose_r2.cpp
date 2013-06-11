#include <LinearMath/btMatrix3x3.h>
#include <clopema_motoros/ClopemaMove.h>
#include <clopema_arm_navigation/ClopemaLinearInterpolation.h>
#include <tf/transform_listener.h>
using namespace std;

int main(int argc, char **argv) {
	ros::init(argc, argv, "move_pose_dual");
	ros::NodeHandle nh;
	
	ClopemaMove cmove;

	//Create plan
	clopema_arm_navigation::ClopemaMotionPlan mp;
	mp.request.motion_plan_req.group_name = "r2_arm";
	mp.request.motion_plan_req.allowed_planning_time = ros::Duration(5.0);

	//Set start state
	if (!cmove.getRobotState(mp.request.motion_plan_req.start_state))
		return -1;


	arm_navigation_msgs::SimplePoseConstraint desired_pose;
	desired_pose.header.frame_id = "base_link";
	desired_pose.header.stamp = ros::Time::now();
	desired_pose.link_name = "r2_ee";
	desired_pose.pose.position.x = 0;
	desired_pose.pose.position.y = -1;
	desired_pose.pose.position.z = 1.5;
	cout<<"gimme coords \n";
	cin>>desired_pose.pose.position.x>>desired_pose.pose.position.y>>desired_pose.pose.position.z;


	
	//~ btVector3 matrix(0.5,-1,-1);
	btQuaternion q;
	//~ q.setRotation( matrix, (btScalar) 3.14/5);
	
	//~ q.setEulerZYX( (btScalar)0 ,(btScalar) 0,(btScalar) 3.14/2);
	//~ 
	//~ ROS_INFO("%f %f %f %f",q.x(),q.y(),q.z(),q.w());
	//~ 
	//~ desired_pose.pose.orientation.x = q.x();
	//~ desired_pose.pose.orientation.y = q.y();
	//~ desired_pose.pose.orientation.z = q.z();
	//~ desired_pose.pose.orientation.w = q.w();
	//~ 
	
	//desired_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(3.14, 3.14/2, 3.14/4 );
	
	float roll , pitch, yaw;
	//~ cout<< "input roll pich yaw : \n";
	//~ cin>> roll >> pitch >> yaw;
	//~ roll=roll*M_PI/180;
	//~ pitch=pitch*M_PI/180;
	//~ yaw=yaw*M_PI/180;
	
	//btMatrix3x3 rotMat( 
	//~ (btScalar) 1, (btScalar) -1, (btScalar) -1,
	//~ (btScalar) 0, (btScalar) 0, (btScalar) 0,
	//~ (btScalar) 0, (btScalar) -1, (btScalar) 1); 
	
	
		//~ btMatrix3x3 rotMat( 
		//~ (btScalar) 0, (btScalar) 0, (btScalar) -1,
		//~ (btScalar) -1, (btScalar) 0, (btScalar) 0,
		//~ (btScalar) 0, (btScalar) 1, (btScalar) 0);
		
		btMatrix3x3 rotMat( 
		(btScalar) 1, (btScalar) 0, (btScalar) 0,
		(btScalar) 0, (btScalar) -1, (btScalar) 0,
		(btScalar) 0, (btScalar) 0, (btScalar) -1);
		
	 
	roll= atan2f(rotMat[2][1],rotMat[2][2] );
	pitch= atan2f(-rotMat[2][0],sqrt(pow(rotMat[2][2],2)+pow(rotMat[2][1],2)));
	yaw= atan2f(rotMat[1][0],rotMat[0][0]);
	//~ //yaw=3.14; roll=0; pitch=0;
	//ROS_INFO("%f %f %f ",yaw, pitch , roll);
	
	//ROS_INFO("%f %f %f ",rotMat[3][2], rotMat[3][2] , rotMat[3][2]);
	
	desired_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw );
	//~ rotMat.getRotation(q);
	
	//~ desired_pose.pose.orientation.x = q.x();
	//~ desired_pose.pose.orientation.y = q.y();
	//~ desired_pose.pose.orientation.z = q.z();
	//~ desired_pose.pose.orientation.w = q.w();
	
	
	desired_pose.absolute_position_tolerance.x = 0.002;
	desired_pose.absolute_position_tolerance.y = 0.002;
	desired_pose.absolute_position_tolerance.z = 0.002;
	desired_pose.absolute_roll_tolerance = 0.04;
	desired_pose.absolute_pitch_tolerance = 0.04;
	desired_pose.absolute_yaw_tolerance = 0.04;
	cmove.poseToClopemaMotionPlan(mp, desired_pose);
	
	
	
	
	
	

   
    //~ 
    //~ 
	ROS_INFO("Planning");
	if (!cmove.plan(mp))
		return -1;
	ROS_INFO("Executing");
	control_msgs::FollowJointTrajectoryGoal goal;
	goal.trajectory = mp.response.joint_trajectory;
	cmove.doGoal(goal);

	tf::TransformListener listener;
    tf::StampedTransform transform;
    
	try {
		listener.waitForTransform("base_link", "r2_ee", ros::Time(0), ros::Duration(10.0) );
		listener.lookupTransform("base_link", "r2_ee", ros::Time(0), transform);
	} catch (tf::TransformException ex) {
		ROS_ERROR("%s",ex.what());
	}

    
	cout<<"------->"<< transform.getOrigin().x()<< "\n " << "------->"<<transform.getOrigin().y()<< "\n"<<"------->" <<transform.getOrigin().z() << "\n";
	ros::shutdown();
	return 1;
}
