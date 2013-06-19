//----motion planning-----
#include <LinearMath/btMatrix3x3.h>
#include <clopema_motoros/ClopemaMove.h>
#include <clopema_arm_navigation/ClopemaLinearInterpolation.h>
#include <clopema_motoros/SetGripperState.h>
#include <tf/transform_listener.h>
//-----grabber-----

#include <nodelet/nodelet.h>
#include <nodelet/loader.h>

#include <iostream>
#include <fstream>
//-----openni----
#include <ros/ros.h>

#include <camera_helpers/OpenNICapture.h>

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

bool cont;
int cx, cy;

bool stop=false;
namespace enc = sensor_msgs::image_encodings;
bool ZFar;

btMatrix3x3 rotMat(
(btScalar) 0, (btScalar) 0, (btScalar) -1,
(btScalar) -1, (btScalar) 0, (btScalar) 0,
(btScalar) 0, (btScalar) 1, (btScalar) 0);

geometry_msgs::Quaternion G_Orientation;
/*
class OpenniGrabber {

public:
    OpenniGrabber()
    {

         client = nh.serviceClient<certh_libs::OpenniCapture>("capture");
    }

    bool grab(cv::Mat &rgb, cv::Mat &depth, pcl::PointCloud<pcl::PointXYZ>&pc)
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
*/

void mouse_callback( int event, int x, int y, int flags, void* param){
    if(event == CV_EVENT_LBUTTONDOWN){
        cx = x;
        cy = y;
        cont = true;
    }
    if(event == CV_EVENT_RBUTTONDOWN){
        stop=true;
    }

    if(event == CV_EVENT_MBUTTONDOWN){
        ZFar=true;
        cont = true;
        cx=1;
        cy=1;
    }
}

void setGrippersOpen(){
    ros::service::waitForService("/r2_gripper/set_open");
    clopema_motoros::SetGripperState sopen;
    sopen.request.open=true;
    ros::service::call("/r2_gripper/set_open", sopen);
}
void setGrippersClose(){
    ros::service::waitForService("/r2_gripper/set_open");
    clopema_motoros::SetGripperState sopen;
    sopen.request.open=false;
    ros::service::call("/r2_gripper/set_open", sopen);
}



void setPathConstraints(clopema_arm_navigation::ClopemaMotionPlan & mp, float radious) {

    mp.request.motion_plan_req.path_constraints.position_constraints.resize(1);
    mp.request.motion_plan_req.path_constraints.position_constraints[0].header.frame_id = "r2_ee";
    mp.request.motion_plan_req.path_constraints.position_constraints[0].header.stamp = ros::Time::now();
    mp.request.motion_plan_req.path_constraints.position_constraints[0].link_name = "r1_ee";
    mp.request.motion_plan_req.path_constraints.position_constraints[0].position.x = 0.0;
    mp.request.motion_plan_req.path_constraints.position_constraints[0].position.y = 0.0;
    mp.request.motion_plan_req.path_constraints.position_constraints[0].position.z = 0.0;
    mp.request.motion_plan_req.path_constraints.position_constraints[0].constraint_region_orientation= G_Orientation;

    mp.request.motion_plan_req.path_constraints.position_constraints[0].constraint_region_shape.type = arm_navigation_msgs::Shape::SPHERE;
    mp.request.motion_plan_req.path_constraints.position_constraints[0].constraint_region_shape.dimensions.push_back(radious+2); //radius
    mp.request.motion_plan_req.path_constraints.position_constraints[0].constraint_region_shape.dimensions.push_back(radious+2);
    mp.request.motion_plan_req.path_constraints.position_constraints[0].constraint_region_shape.dimensions.push_back(radious+2);

    mp.request.motion_plan_req.path_constraints.position_constraints[0].weight = 1.0;

}

float getDistance(geometry_msgs::Point point1, geometry_msgs::Point point2){

    return sqrt((point1.x - point2.x)*(point1.x - point2.x) + (point1.y - point2.y)*(point1.y - point2.y)+(point1.z - point2.z)*(point1.z - point2.z));
};

float getArmsDistance(){

    tf::TransformListener listener;
    tf::StampedTransform transform;

    try {
        listener.waitForTransform("base_link", "r2_ee", ros::Time(0), ros::Duration(10.0) );
        listener.lookupTransform("base_link", "r2_ee", ros::Time(0), transform);
    } catch (tf::TransformException ex) {
        ROS_ERROR("%s",ex.what());
    }
    geometry_msgs::Point tmp_point1;
    tmp_point1.x= transform.getOrigin().x();
    tmp_point1.y= transform.getOrigin().y();
    tmp_point1.z= transform.getOrigin().z();

    try {
        listener.waitForTransform("base_link", "r1_ee", ros::Time(0), ros::Duration(10.0) );
        listener.lookupTransform("base_link", "r1_ee", ros::Time(0), transform);
    } catch (tf::TransformException ex) {
        ROS_ERROR("%s",ex.what());
    }
    geometry_msgs::Point tmp_point2;
    tmp_point2.x= transform.getOrigin().x();
    tmp_point2.y= transform.getOrigin().y();
    tmp_point2.z= transform.getOrigin().z();

    return getDistance(tmp_point1, tmp_point2);

};


std::vector<geometry_msgs::Point> calculateCirclePoints(geometry_msgs::Point center ,geometry_msgs::Point currentPos, float r, int steps, bool up) {

    std::vector<geometry_msgs::Point> points;
    float currTheta;
    float step;
    float theta;
    float tmp_x, tmp_z;
    geometry_msgs::Point point;

    if(up){
        currTheta =M_PI- atan((currentPos.x-center.x)/ (center.z-currentPos.z));
        step=3.14/2/steps;//(abs(M_PI-currTheta))/(float)steps;
        cout<< "step  ----> " << step <<"\n ";
        for (unsigned int i = 0; i <steps; i ++){

            tmp_x=r*sin(currTheta);
            tmp_z=r*cos(currTheta);
            cout<< "temp_x= "<< tmp_x << " "<<"temp_z= "<< tmp_z<<"\n";
            cout<<"theta = "<< currTheta <<"\n";
            currTheta-=step;
            point.x = center.x+tmp_x;
            point.z = center.z+tmp_z;
            point.y = currentPos.y;
            points.push_back(point);
            cout<< "x= "<<point.x<<" "<< "y= "<<point.y<<" "<< "z= "<<point.z<<"\n";
        }
        return points;
    }
}

void moveTo(geometry_msgs::Pose pose){
        //Create plan
        clopema_arm_navigation::ClopemaMotionPlan mp;
        ClopemaMove cmove;

        mp.request.motion_plan_req.group_name = "r2_arm";
        mp.request.motion_plan_req.allowed_planning_time = ros::Duration(5.0);

        //Set start state
        cmove.getRobotState(mp.request.motion_plan_req.start_state);

        arm_navigation_msgs::SimplePoseConstraint desired_pose;

        desired_pose.header.frame_id = "base_link";
        desired_pose.header.stamp = ros::Time::now();
        desired_pose.link_name = "r2_ee";

        desired_pose.pose = pose;


        cout<< "\n going to --- >  "<< desired_pose.pose.position.x<< " "  << desired_pose.pose.position.y<<"  " << desired_pose.pose.position.z <<"\n";

        desired_pose.absolute_position_tolerance.x = 0.002;
        desired_pose.absolute_position_tolerance.y = 0.002;
        desired_pose.absolute_position_tolerance.z = 0.002;
        desired_pose.absolute_roll_tolerance = 0.004;
        desired_pose.absolute_pitch_tolerance = 0.004;
        desired_pose.absolute_yaw_tolerance = 0.004;
        cmove.poseToClopemaMotionPlan(mp, desired_pose);

        ROS_INFO("Planning");
        cmove.plan(mp);

        ROS_INFO("Executing");
        control_msgs::FollowJointTrajectoryGoal goal;
        goal.trajectory = mp.response.joint_trajectory;
        cmove.doGoal(goal);
}


/*
 * Convert joint trajectory point to robot state
 */
void pointToRobotState(arm_navigation_msgs::RobotState &state, trajectory_msgs::JointTrajectoryPoint &point, std::vector<std::string> & joint_names) {
    for (unsigned int i = 0; i < joint_names.size(); i++) {
        for (unsigned int j = 0; j < state.joint_state.name.size(); j++) {
            if (state.joint_state.name.at(j) == joint_names.at(i)) {
                state.joint_state.position.at(j) = point.positions.at(i);
                break;
            }
        }
    }
}



void moveThrough(	std::vector<geometry_msgs::Point> traj){

    ClopemaMove cmove;
    trajectory_msgs::JointTrajectory wholeTraj;

    //Create plan
    clopema_arm_navigation::ClopemaMotionPlan mp;
    mp.request.motion_plan_req.group_name = "r2_arm";
    mp.request.motion_plan_req.allowed_planning_time = ros::Duration(5.0);
    cmove.getRobotState(mp.request.motion_plan_req.start_state);

    arm_navigation_msgs::SimplePoseConstraint desired_pose;
    desired_pose.header.frame_id = "base_link";
    desired_pose.header.stamp = ros::Time::now();
    desired_pose.link_name = "r2_ee";
    desired_pose.pose.position = traj[0];
    desired_pose.pose.orientation = G_Orientation;
    desired_pose.absolute_position_tolerance.x = 0.02;
    desired_pose.absolute_position_tolerance.y = 0.02;
    desired_pose.absolute_position_tolerance.z = 0.02;
    desired_pose.absolute_roll_tolerance = 0.04;
    desired_pose.absolute_pitch_tolerance = 0.04;
    desired_pose.absolute_yaw_tolerance = 0.04;

    cmove.poseToClopemaMotionPlan(mp, desired_pose);
    cmove.plan(mp);

    wholeTraj = mp.response.joint_trajectory;

    for(unsigned int i=1;i<traj.size();i++){

        mp.request.motion_plan_req.goal_constraints.orientation_constraints.clear();
        mp.request.motion_plan_req.goal_constraints.position_constraints.clear();
        desired_pose.pose.position = traj[i];
        cmove.poseToClopemaMotionPlan(mp, desired_pose);
        pointToRobotState(mp.request.motion_plan_req.start_state, wholeTraj.points.back(), wholeTraj.joint_names);
        cmove.plan(mp);

        wholeTraj.points.insert(wholeTraj.points.end(), mp.response.joint_trajectory.points.begin(), mp.response.joint_trajectory.points.end());
    }

    ROS_INFO("Executing trajectory size: %d", wholeTraj.points.size());
    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory = wholeTraj;
    cmove.doGoal(goal);

}

void makeCircle(){
    std::vector<geometry_msgs::Point> traj;
    //~ float radious = getArmsDistance();
    //~ cout<<"radious = " << radious<<"\n";
    //~ setPathConstraints(mp,radious);

    tf::TransformListener listener;
    tf::StampedTransform transform;

    try {
        listener.waitForTransform("base_link", "r2_ee", ros::Time(0), ros::Duration(10.0) );
        listener.lookupTransform("base_link", "r2_ee", ros::Time(0), transform);
    } catch (tf::TransformException ex) {
        ROS_ERROR("%s",ex.what());
    }

    geometry_msgs::Point tmp_point1;
    tmp_point1.x= transform.getOrigin().x();
    tmp_point1.y= transform.getOrigin().y();
    tmp_point1.z= transform.getOrigin().z();

    try {
        listener.waitForTransform("base_link", "r1_ee", ros::Time(0), ros::Duration(10.0) );
        listener.lookupTransform("base_link", "r1_ee", ros::Time(0), transform);
    } catch (tf::TransformException ex) {
        ROS_ERROR("%s",ex.what());
    }
    geometry_msgs::Point tmp_point2;
    tmp_point2.x= transform.getOrigin().x();
    tmp_point2.y= transform.getOrigin().y();
    tmp_point2.z= transform.getOrigin().z();
    traj = calculateCirclePoints(tmp_point2, tmp_point1, getArmsDistance()-0.03, 10, true );
    moveThrough(traj);
}
int main(int argc, char **argv) {
    ros::init(argc, argv, "move_pose_dual");
    ros::NodeHandle nh;

    camera_helpers::OpenNICaptureAll grabber("xtion3") ;
    grabber.connect() ;

    cv::Mat rgb, depth ;
    pcl::PointCloud<pcl::PointXYZ> pc ;
    cv::Mat R = cv::Mat(4, 4, CV_32FC1, cv::Scalar::all(0));
    ifstream f;
    f.open("/home/clopema/ROS/clopema_stack/clopema_planning_tutorials/files/CalibMat.txt");
    if(f.is_open()){
        for(int i=0; i<4; ++i)
            for(int j=0; j<4; ++j)
                f >> R.at<float>(i, j);

    }else{
        cout << "Cannot open CalibMat.txt";
        return(-1);
    }

    cvNamedWindow("calibration");
    cvSetMouseCallback( "calibration", mouse_callback, NULL);
    cv::Mat targetP = cv::Mat(4, 1, CV_32FC1, cv::Scalar::all(0));


    while(!stop){
        ZFar = false;

        ros::Time ts ;
         image_geometry::PinholeCameraModel cm ;

        if ( grabber.grab(rgb, depth, pc, ts, cm) ){
            cont = false;
            while(!cont){
                cv::imshow("calibration", rgb);
                int k = cv::waitKey(1);
                if (stop)
                    return 0;
            }
            pcl::PointXYZ p = pc.at(cx, cy);
            cv::Mat pM = cv::Mat(4, 1, CV_32FC1, cv::Scalar::all(0));
            pM.at<float>(0,0) = p.x;
            pM.at<float>(1,0) = p.y;
            pM.at<float>(2,0) = p.z;
            pM.at<float>(3,0) = 1;

            targetP = R*pM;

            //~ cout << cx << " " << cy << std::endl << p.x << " " << p.x << " " << p.z << std::endl;



        }
        else{
            ROS_ERROR("Failed to call service");
            return 1;
        }
        setGrippersOpen();

        geometry_msgs::Pose desPos;
        btQuaternion q;
        float roll , pitch, yaw;

        roll= atan2f(rotMat[2][1],rotMat[2][2] );
        pitch= atan2f(-rotMat[2][0],sqrt(pow(rotMat[2][2],2)+pow(rotMat[2][1],2)));
        yaw= atan2f(rotMat[1][0],rotMat[0][0]);
        G_Orientation=tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw );
        desPos.orientation = G_Orientation;

        if(!ZFar){
            desPos.position.x = targetP.at<float>(0,0);
            desPos.position.y = targetP.at<float>(1,0)- 0.02;
            desPos.position.z = targetP.at<float>(2,0);
            moveTo(desPos);
            setGrippersClose();
            makeCircle();
        }
        else{
            desPos.position.x = 0.3;
            desPos.position.y = -1;
            desPos.position.z = 1.3;
            moveTo(desPos);
        }
        cin.ignore();

        setGrippersOpen();



    }

    ros::shutdown();
    return 1;
}

