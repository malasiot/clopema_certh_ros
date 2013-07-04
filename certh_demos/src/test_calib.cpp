//----motion planning-----
#include <LinearMath/btMatrix3x3.h>
#include <clopema_motoros/ClopemaMove.h>
#include <clopema_arm_navigation/ClopemaLinearInterpolation.h>
#include <clopema_motoros/SetGripperState.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
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
#include <visualization_msgs/Marker.h>

#include <robot_helpers/Robot.h>
#include <robot_helpers/Utils.h>

using namespace robot_helpers ;


extern bool findLowestPoint(const pcl::PointCloud<pcl::PointXYZ> &depth, const Eigen::Vector3f &orig, const Eigen::Vector3f &base, float apperture,  Eigen::Vector3f &p, Eigen::Vector3f &n, int cx , int cy) ;

bool cont;
int cx, cy;

bool stop=false;
namespace enc = sensor_msgs::image_encodings;
bool ZFar;

btMatrix3x3 rotMat(
(btScalar) 0, (btScalar) 1, (btScalar) -1,
(btScalar) -1, (btScalar) 0, (btScalar) 0,
(btScalar) 0, (btScalar) 1, (btScalar) 1);

btMatrix3x3 rotMat2(
(btScalar) 0, (btScalar) 1, (btScalar) -1,
(btScalar) -1, (btScalar) 0, (btScalar) 0,
(btScalar) 0, (btScalar) 1, (btScalar) 1);

btMatrix3x3 rotMat3(
(btScalar) 0, (btScalar) 1, (btScalar) -1,
(btScalar) -1, (btScalar) 0, (btScalar) 0,
(btScalar) 0, (btScalar) 1, (btScalar) 1);


btMatrix3x3 homeRotMat(
        (btScalar) 0, (btScalar) 0, (btScalar) -1,
        (btScalar) -1, (btScalar) 0, (btScalar) 0,
        (btScalar) 0, (btScalar) 1, (btScalar) 0);

geometry_msgs::Quaternion G_Orientation;


extern bool findLowestPoint(const pcl::PointCloud<pcl::PointXYZ> &depth, const Eigen::Vector3f &orig, const Eigen::Vector3f &base, float apperture,  Eigen::Vector3f &p, Eigen::Vector3f &n,int cx, int cy, cv::Mat depthMap) ;


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
    mp.request.motion_plan_req.path_constraints.position_constraints[0].constraint_region_shape.dimensions.push_back(radious); //radius
    mp.request.motion_plan_req.path_constraints.position_constraints[0].constraint_region_shape.dimensions.push_back(radious);
    mp.request.motion_plan_req.path_constraints.position_constraints[0].constraint_region_shape.dimensions.push_back(radious);

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

    float tmp_x, tmp_z;
    geometry_msgs::Point point;

    if(up){
        currTheta =M_PI- atan((currentPos.x-center.x)/ (center.z-currentPos.z));
        step=3.14/2/steps;//(abs(M_PI-currTheta))/(float)steps;
      //  cout<< "step  ----> " << step <<"\n ";
        for ( int i = 0; i <steps; i ++){

            tmp_x=r*sin(currTheta);
            tmp_z=r*cos(currTheta);
          //  cout<< "temp_x= "<< tmp_x << " "<<"temp_z= "<< tmp_z<<"\n";
          //  cout<<"theta = "<< currTheta <<"\n";
            currTheta-=step;
            point.x = center.x+tmp_x;
            point.z = center.z+tmp_z;
            point.y = currentPos.y;
            points.push_back(point);
         //   cout<< "x= "<<point.x<<" "<< "y= "<<point.y<<" "<< "z= "<<point.z<<"\n";
        }
        return points;
    }
    return points;
}

int moveTo(geometry_msgs::Pose pose){
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


       // cout<< "\n going to --- >  "<< desired_pose.pose.position.x<< " "  << desired_pose.pose.position.y<<"  " << desired_pose.pose.position.z <<"\n";

        desired_pose.absolute_position_tolerance.x = 0.002;
        desired_pose.absolute_position_tolerance.y = 0.002;
        desired_pose.absolute_position_tolerance.z = 0.002;
        desired_pose.absolute_roll_tolerance = 0.004;
        desired_pose.absolute_pitch_tolerance = 0.004;
        desired_pose.absolute_yaw_tolerance = 0.004;
        cmove.poseToClopemaMotionPlan(mp, desired_pose);

        ROS_INFO("Planning");
        if (!cmove.plan(mp))
            return -1;

        ROS_INFO("Executing");
        control_msgs::FollowJointTrajectoryGoal goal;
        goal.trajectory = mp.response.joint_trajectory;
        cmove.doGoal(goal);
        return 0;
}


int moveWithConstrains(geometry_msgs::Pose pose){
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


       // cout<< "\n going to --- >  "<< desired_pose.pose.position.x<< " "  << desired_pose.pose.position.y<<"  " << desired_pose.pose.position.z <<"\n";

        desired_pose.absolute_position_tolerance.x = 0.002;
        desired_pose.absolute_position_tolerance.y = 0.002;
        desired_pose.absolute_position_tolerance.z = 0.002;
        desired_pose.absolute_roll_tolerance = 0.004;
        desired_pose.absolute_pitch_tolerance = 0.004;
        desired_pose.absolute_yaw_tolerance = 0.004;

        setPathConstraints(mp,0.8);

        cmove.poseToClopemaMotionPlan(mp, desired_pose);

        ROS_INFO("Planning");
        if (!cmove.plan(mp))
            return -1;

        ROS_INFO("Executing");
        control_msgs::FollowJointTrajectoryGoal goal;
        goal.trajectory = mp.response.joint_trajectory;
        cmove.doGoal(goal);
        return 0;
}

void rotateGripper(float angle){

    //Create plan
    clopema_arm_navigation::ClopemaMotionPlan mp;
    ClopemaMove cmove;

    mp.request.motion_plan_req.group_name = "r1_arm";
    mp.request.motion_plan_req.allowed_planning_time = ros::Duration(5.0);

    //Set start state
    cmove.getRobotState(mp.request.motion_plan_req.start_state);

    arm_navigation_msgs::SimplePoseConstraint desired_pose;

    desired_pose.header.frame_id = "r1_ee";
    desired_pose.header.stamp = ros::Time::now();
    desired_pose.link_name = "r1_ee";

    desired_pose.pose.position.x = 0;
    desired_pose.pose.position.y = 0;
    desired_pose.pose.position.z = 0;
    desired_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, angle );


   // cout<< "\n going to --- >  "<< desired_pose.pose.position.x<< " "  << desired_pose.pose.position.y<<"  " << desired_pose.pose.position.z <<"\n";

    desired_pose.absolute_position_tolerance.x = 0.002;
    desired_pose.absolute_position_tolerance.y = 0.002;
    desired_pose.absolute_position_tolerance.z = 0.002;
    desired_pose.absolute_roll_tolerance = 0.004;
    desired_pose.absolute_pitch_tolerance = 0.004;
    desired_pose.absolute_yaw_tolerance = 0.004;
    cmove.poseToClopemaMotionPlan(mp, desired_pose);

    ROS_INFO("Planning");
    if (!cmove.plan(mp))
        return ;

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

    if (!cmove.plan(mp))
        return ;
    wholeTraj = mp.response.joint_trajectory;

    for(unsigned int i=1;i<traj.size();i++){

        if(i>1){
            float yaw, pitch, roll;
            roll= atan2f(homeRotMat[2][1],homeRotMat[2][2] );
            pitch= atan2f(-homeRotMat[2][0],sqrt(pow(homeRotMat[2][2],2)+pow(homeRotMat[2][1],2)));
            yaw= atan2f(homeRotMat[1][0],homeRotMat[0][0]);
            G_Orientation=tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw );
            desired_pose.pose.orientation = G_Orientation;
        }

        mp.request.motion_plan_req.goal_constraints.orientation_constraints.clear();
        mp.request.motion_plan_req.goal_constraints.position_constraints.clear();
        desired_pose.pose.position = traj[i];
        cmove.poseToClopemaMotionPlan(mp, desired_pose);
        pointToRobotState(mp.request.motion_plan_req.start_state, wholeTraj.points.back(), wholeTraj.joint_names);
        if (!cmove.plan(mp))
            return ;

        wholeTraj.points.insert(wholeTraj.points.end(), mp.response.joint_trajectory.points.begin(), mp.response.joint_trajectory.points.end());
    }

    //ROS_INFO("Executing trajectory size: %d", wholeTraj.points.size());
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


void convertCloud(const cv::Mat &dep, const pcl::PointCloud<pcl::PointXYZ>& cloud, const image_geometry::PinholeCameraModel &model_ )
{

    for(int i=0 ; i<dep.rows ; i++ )
        for(int j=0 ; j<dep.cols ; j++ )
        {
            float center_x = model_.cx();
            float center_y = model_.cy();

            // Combine unit conversion (if necessary) with scaling by focal length for computing (X,Y)
            double unit_scaling = 0.001 ;
            float constant_x = unit_scaling / model_.fx();
            float constant_y = unit_scaling / model_.fy();

            ushort val = dep.at<ushort>(i, j) ;

            if ( val == 0 ) continue ;

            // Fill in XYZ
            cout << (j - center_x) * val * constant_x << ' ';
            cout << (i - center_y) * val * constant_y << ' ' ;
            cout <<  val * 0.001 << endl ;

            cout << cloud.at(j, i) << endl ;


        }

}

void findGraspingOrientation(Eigen::Vector4d vector){

    Eigen::Vector3d N(vector.x(), vector.y(), vector.z());
    N.normalize();
    Eigen::Vector3d A(1,0,-1);
    Eigen::Vector3d B=A-(A.dot(N))*N;
    Eigen::Vector3d C=N.cross(B);

    C.normalize();
    B.normalize();

    rotMat[0][0]=N.x();
    rotMat[1][0]=N.y();
    rotMat[2][0]=N.z();

    rotMat[0][1]=C.x();
    rotMat[1][1]=C.y();
    rotMat[2][1]=C.z();

    rotMat[0][2]=-B.x();
    rotMat[1][2]=-B.y();
    rotMat[2][2]=-B.z();

    cout<<N<<"\n"<<C<<"\n "<<B<<endl;
    cout<<C.dot(N)<< " "<<B.dot(N)<< endl;

}

float findBias(Eigen::Vector4d vector){

    float theta=atan2f(vector.x(),abs(vector.y()));

    return theta;
}

geometry_msgs::Quaternion rotationMatrixToQuaternion(btMatrix3x3 matrix){

    float roll , pitch, yaw;

    roll= atan2f(matrix[2][1],matrix[2][2] );
    pitch= atan2f(-matrix[2][0],sqrt(pow(matrix[2][2],2)+pow(matrix[2][1],2)));
    yaw= atan2f(matrix[1][0],matrix[0][0]);
    G_Orientation=tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw );
    return G_Orientation;
}


int main(int argc, char **argv) {

    ros::init(argc, argv, "move_pose_dual");
    ros::NodeHandle nh;

    camera_helpers::OpenNICaptureAll grabber("xtion3") ;
    grabber.connect() ;

    cv::Mat rgb, depth ;
    pcl::PointCloud<pcl::PointXYZ> pc ;
    cv::Mat R = cv::Mat(4, 4, CV_32FC1, cv::Scalar::all(0));

    tf::TransformListener listener;
    ros::Time ts(0) ;
    tf::StampedTransform transform;
    Eigen::Affine3d pose;
    Eigen::Matrix4d calib;


    try {
        listener.waitForTransform("xtion3_rgb_optical_frame", "base_link", ts, ros::Duration(1) );
        listener.lookupTransform("xtion3_rgb_optical_frame", "base_link", ts, transform);
        tf::TransformTFToEigen(transform, pose);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
    }



    cvNamedWindow("calibration");
    cvSetMouseCallback( "calibration", mouse_callback, NULL);
    //cv::Mat targetP = cv::Mat(4, 1, CV_32FC1, cv::Scalar::all(0));

    calib=pose.matrix();
    Eigen::Vector4d targetP;
    while(!stop){
        ZFar = false;

        ros::Time ts ;
         image_geometry::PinholeCameraModel cm ;

        if ( grabber.grab(rgb, depth, pc, ts, cm) ){

      //      convertCloud(depth, pc, cm) ;
            cont = false;
            while(!cont){
                cv::imshow("calibration", rgb);
                int k = cv::waitKey(1);
                if (stop)
                    return 0;
            }

            pcl::PointXYZ p = pc.at(cx, cy);
            cout<< "click x= "<< cx << " y= "<< cy<<endl;
            Eigen::Vector4d pM(p.x, p.y, p.z, 1);
            cout<< "pc x= "<<pM[0]<<" pc y= "<< pM[1] << "pc z= "<< pM[2]<<endl;
            targetP = calib.inverse()*pM;
        }
        else{
            ROS_ERROR("Failed to call service");
            return 1;
        }
        setGrippersOpen();
        ///////LOWEST POINT/////////
        tf::TransformListener listenR1;
        tf::StampedTransform tanformR1;
        ros::Time time;
        Eigen::Vector3f top;

        try {
            listenR1.waitForTransform( "xtion3_rgb_optical_frame", "r1_ee",   time, ros::Duration(1) );
            listenR1.lookupTransform( "xtion3_rgb_optical_frame", "r1_ee" , time, tanformR1);
        }
        catch (tf::TransformException ex){
          ROS_ERROR("%s",ex.what());
        }

        top.x()=tanformR1.getOrigin().x();
        top.y()=tanformR1.getOrigin().y();
        top.z()=tanformR1.getOrigin().z();

      //  cout<<"top is x="<<top.x()<< " y="<<top.y()<<" z="<<top.z()<<endl;
        Eigen::Vector3f bottom;
        bottom=top;
        bottom.x()-=1;
       // cout<<"bot is x="<<bottom.x()<< " y="<<bottom.y()<<" z="<<bottom.z()<<endl;
        Eigen::Vector3f targetPo;
        Eigen::Vector3f targetN;
        float angle=M_PI_2;


        pcl::PointXYZ val;
        //cout<< "pc width= "<< pc.width << "pc height= "<< pc.height<< endl;


        for(int j=0 ; j< pc.width ; j++ )
        {

            for(int i=0 ; i< pc.height ; i++)
            {


                val  = pc.at(j, i) ;
                if ( !pcl_isfinite(val.z) ) continue ;

               // cout <<"for i="<< i <<"j= "<<j<<"coords are = "<<  val.x << ' ' << val.y << ' ' << val.z << endl;

           }


        }
        findLowestPoint(pc,top,bottom,angle,targetPo,targetN, cx, cy ,depth );
        cout<<"lowest point is x="<<targetPo.x()<< " y="<<targetPo.y()<<" z="<<targetPo.z()<<endl;
        cout<<"target vector x= "<<targetN.x()<< " y="<<targetN.y()<<" z="<<targetN.z()<<endl;

        cout<<"-------------------------------------------"<<endl;

        Eigen::Vector4d tar(targetPo.x(), targetPo.y(), targetPo.z(), 1);
        targetP = calib.inverse()*tar;

       ///////////// DRAW ARROW MARKER //////////////////////

          ros::Rate r(1);
          ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);

          uint32_t shape = visualization_msgs::Marker::ARROW;
          int count=0;
          while (ros::ok())
          {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "/xtion3_rgb_optical_frame";
            marker.header.stamp = ros::Time::now();

            marker.ns = "basic_shapes";
            marker.id = 0;

            marker.type = shape;

            marker.action = visualization_msgs::Marker::ADD;


           marker.points.resize(2);
           marker.points[0].x = targetPo.x();
           marker.points[0].y = targetPo.y();
           marker.points[0].z = targetPo.z();

           marker.points[1].x = targetPo.x()+targetN.x();
           marker.points[1].y = targetPo.y()+targetN.y();
           marker.points[1].z = targetPo.z()+targetN.z();

            marker.scale.x = 0.01;
            marker.scale.y = 0.02;
            marker.scale.z = 0.03;

            marker.color.r = 0.0f;
            marker.color.g = 1.0f;
            marker.color.b = 0.0f;
            marker.color.a = 1.0;

            marker.lifetime = ros::Duration();

            marker_pub.publish(marker);
           r.sleep();
           if(count>4) break;
             count++;
          }

        //////////////////////////

    //////LOWEST POINT END /////



        Eigen::Vector4d norm (targetN.x(), targetN.y(), targetN.z(), 0);
        Eigen::Vector4d targetNo;
        targetNo = calib.inverse()*norm.normalized();
        rotMat[0][0] = targetNo.x();
        rotMat[0][1] = targetNo.y();
        rotMat[0][2] = targetNo.z();
        findGraspingOrientation(targetNo);

        geometry_msgs::Pose desPos;

        desPos.orientation = rotationMatrixToQuaternion(rotMat);

        if(!ZFar){
            addConeToCollisionModel("r1",0.5, 0.2);
            desPos.position.x = targetP.x()+rotMat[0][0]*0.03-rotMat[0][2]*0.07;
            desPos.position.y = targetP.y()+rotMat[1][0]*0.03-rotMat[1][2]*0.07;
            desPos.position.z = targetP.z()+rotMat[2][0]*0.03-rotMat[2][2]*0.07;
            cout<<"HIT ENDER TO MOVE THE ARM"<<endl;
            cin.ignore();
            if(moveTo(desPos)==-1){
                resetCollisionModel() ;
                cout<< "BIAS : " <<findBias(targetNo)<<endl;
                rotateGripper(findBias(targetNo));
                continue;
            }
            resetCollisionModel();
            addSphereToCollisionModel("r1", 0.55);
            cout<<"HIT ENTER TO GRASP"<<endl;
            cin.ignore();

            desPos.position.x = targetP.x()+rotMat[0][2]*0.045+rotMat[0][0]*0.03;
            desPos.position.y = targetP.y()+rotMat[1][2]*0.045+rotMat[1][0]*0.03;
            desPos.position.z = targetP.z()+rotMat[2][2]*0.045+rotMat[2][0]*0.03;
            moveTo(desPos);
            setGrippersClose();
            //makeCircle();
            desPos.position.x += 0.75;
            desPos.position.z += 0.75;
            desPos.orientation = rotationMatrixToQuaternion(homeRotMat);
            moveWithConstrains(desPos);
            resetCollisionModel();
        }
        else{
            desPos.position.x = 0.3;
            desPos.position.y = -1;
            desPos.position.z = 1.3;
            moveTo(desPos);
        }

        cout<<"----HIT ENTER TO OPEN GRIPPERS ---"<<endl;
        cin.ignore();

        setGrippersOpen();



    }

    ros::shutdown();
    return 1;
}

