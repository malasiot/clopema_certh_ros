#include <LinearMath/btMatrix3x3.h>
#include <clopema_motoros/ClopemaMove.h>
#include <clopema_arm_navigation/ClopemaLinearInterpolation.h>
#include <tf/transform_listener.h>

#include <nodelet/nodelet.h>
#include <nodelet/loader.h>

#include <iostream>
#include <fstream>
//-----openni----
#include <ros/ros.h>

#include "/home/clopema/ROS/clopema_stack/certh_libs/srv_gen/cpp/include/certh_libs/OpenniCapture.h"

// OpenCV includes
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

bool cont;
int cx, cy;
bool stop=false;
namespace enc = sensor_msgs::image_encodings;

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

int setPose(float x,float y ,float z ){
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
	desired_pose.pose.position.x = x;
	desired_pose.pose.position.y = y;
	desired_pose.pose.position.z = z;

	btQuaternion q;

	btMatrix3x3 rotMat(
	(btScalar) 0, (btScalar) 0, (btScalar) -1,
	(btScalar) -1, (btScalar) 0, (btScalar) 0,
	(btScalar) 0, (btScalar) 1, (btScalar) 0);

	rotMat.getRotation(q);

	desired_pose.pose.orientation.x = q.x();
	desired_pose.pose.orientation.y = q.y();
	desired_pose.pose.orientation.z = q.z();
	desired_pose.pose.orientation.w = q.w();

	desired_pose.absolute_position_tolerance.x = 0.002;
	desired_pose.absolute_position_tolerance.y = 0.002;
	desired_pose.absolute_position_tolerance.z = 0.002;

	desired_pose.absolute_roll_tolerance = 0.04;
	desired_pose.absolute_pitch_tolerance = 0.04;
	desired_pose.absolute_yaw_tolerance = 0.04;

	cmove.poseToClopemaMotionPlan(mp, desired_pose);

	ROS_INFO("Planning");
	if (!cmove.plan(mp))
		return -1;
	ROS_INFO("Executing");
	control_msgs::FollowJointTrajectoryGoal goal;
	goal.trajectory = mp.response.joint_trajectory;
	cmove.doGoal(goal);

}


void mouse_callback( int event, int x, int y, int flags, void* param){	
	if(event == CV_EVENT_LBUTTONDOWN){
		cx = x;
		cy = y;
		cont = true;
	}
		if(event == CV_EVENT_RBUTTONDOWN){
		stop=true;
	}
	
}


int main(int argc, char **argv) {
	ros::init(argc, argv, "move_pose_dual");
	ros::NodeHandle nh;

	//grabbing the image
	OpenniGrabber grabber ;

    cv::Mat rgb, depth ;
    pcl::PointCloud<pcl::PointXYZ> pc ;



    

	//~ Input the points from the file
	float points[9][3];
	cv::Mat meanRobot = cv::Mat(3, 1, CV_32FC1, cv::Scalar::all(0));	
	ifstream stream;
	stream.open("/home/clopema/ROS/clopema_stack/clopema_planning_tutorials/files/calibrationPoints.txt");
	if (stream.is_open()){		
		for (unsigned int i=0;i<9;i++){			
			for (unsigned int j=0;j<3;j++){
				stream >> points[i][j];
				meanRobot.at<float>(j, 0) += (points[i][j]/9.0f);				
			}
		}					
	}
	stream.close();

	//~ Seting the Points and move the robot
	cvNamedWindow("calibration");
	cvSetMouseCallback( "calibration", mouse_callback, NULL);			
	float xtionPoints[9][3];
	cv::Mat meanXtion = cv::Mat(3, 1, CV_32FC1, cv::Scalar::all(0));	
	for (unsigned int i=0;i<9;i++){
			setPose(points[i][0],points[i][1],points[i][2]);
			
			if ( grabber.grab(rgb, depth, pc) )
			{
				cont = false;
				while(!cont){		
					cv::imshow("calibration", rgb);					
					int k = cv::waitKey(1);	
					if(stop)
					return 0;			
				}	
				pcl::PointXYZ p = pc.at(cx, cy);
				
				xtionPoints[i][0] = p.x;
				xtionPoints[i][1] = p.y;
				xtionPoints[i][2] = p.z;
								
				cout << "Xtion points: " << p.x << " " << p.y << " " << p.z << std::endl;
				
				meanXtion.at<float>(0, 0) += p.x/9.0f;
				meanXtion.at<float>(1, 0) += p.y/9.0f;
				meanXtion.at<float>(2, 0) += p.z/9.0f;
			}
			else
			{
				ROS_ERROR("Failed to call service");
				return 1;
			}			
	}
	
	cv::Mat H = cv::Mat(3, 3, CV_32FC1, cv::Scalar::all(0));		
	cv::Mat Pa = cv::Mat(3, 1, CV_32FC1, cv::Scalar::all(0));
	cv::Mat Pb = cv::Mat(1, 3, CV_32FC1, cv::Scalar::all(0));
	for(int i=0; i<9; ++i){
		for(int j=0; j<3; ++j){
			Pa.at<float>(j, 0) = xtionPoints[i][j] - meanXtion.at<float>(j, 0);
			Pb.at<float>(0, j) = points[i][j] - meanRobot.at<float>(j, 0);
		}		
		H += Pa * Pb;				
	}
	cv::SVD svd(H, cv::SVD::FULL_UV);
	cv::Mat R = svd.vt.t() * svd.u.t();	
	cv::Mat t = (-1)*R*meanXtion + meanRobot;
	
	ofstream fout;
	fout.open("/home/clopema/ROS/clopema_stack/clopema_planning_tutorials/files/CalibMat.txt");
	
	for(int i=0; i<3; ++i){
		for(int j=0; j<3; ++j)
			fout << R.at<float>(i, j) << " ";
		fout << t.at<float>(i, 0) << std::endl;
	}
	fout << 0 << " " << 0 << " " << 0 << " " << 1 << std::endl;
	
	fout.close();

	ros::shutdown();
	return 1;
}
