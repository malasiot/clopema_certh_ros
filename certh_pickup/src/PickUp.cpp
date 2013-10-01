#include "PickUp.h"
#include <clopema_motoros/WriteIO.h>

using namespace certh_libs ;

PickUp::PickUp(string arm){

    parkingPose <<  1.2, 0, 1.3 ;
    armName = arm ;
    defaultDist = 1.0 ;
    defaultTableHeight = 0.75 ;

    camera = ( armName == "r1" ) ? "xtion1" : "xtion2" ;

    grabber = new camera_helpers::OpenNICaptureRGBD (camera) ;
    grabber->connect() ;
    ros::Duration(0.3).sleep() ;
}

PickUp::~PickUp() {
    grabber->disconnect() ;

}

bool PickUp::moveAboveTable(){

    MoveRobot cmove ;
    cmove.setServoMode(false) ;

    if( moveGripperPointingDown(cmove, armName, parkingPose.x(), parkingPose.y(), parkingPose.z() ))
        return true ;
    return false ;

}

int PickUp::openG2(){

    clopema_motoros::WriteIO openGripper;
    openGripper.request.address = 10026;

    for(unsigned int i = 0 ; i < 4 ; i++){
        openGripper.request.value = false;
        if (!ros::service::call("/write_io", openGripper)) {
            ROS_ERROR("Can't call service write_io");
            return -1;
        }
        ros::Duration(0.1).sleep();

        openGripper.request.value = true;
        ros::service::waitForService("/write_io");
        if (!ros::service::call("/write_io", openGripper)) {
            ROS_ERROR("Can't call service write_io");
            return -1;
        }
        ros::Duration(0.5).sleep();
    }

}



Vector3d PickUp::retargetSensor(const cv::Mat dmap, const vector<cv::Point> &hull, const image_geometry::PinholeCameraModel &cm, double Zd)
{
    double minZ, maxZ ;

    cv::Mat clr_ ;
    cv::minMaxLoc(dmap, &minZ, &maxZ) ;

    // compute the minimum distance of the sensor from the table

    minZ = maxZ/1000.0 + 0.7 ;

    // find the center of the object and re-target the sensor

    cv::Point2f center ;
    float rad ;
    cv::minEnclosingCircle(hull, center, rad) ;

    double zS = Zd * rad / std::min(dmap.cols, dmap.rows) ;

    cv::circle(clr_, center, rad, cv::Scalar(255, 0, 255)) ;

    zS = std::max(minZ, zS) ;

    zS = 0.7 + maxZ/1000 ;

    cv::Point3d p = cm.projectPixelTo3dRay(cv::Point2d(center.x, center.y));

    p.x *= zS ; p.y *= zS ; p.z *= zS ;

    cv::imwrite("/tmp/oo.png", clr_);

    return Vector3d(p.x, p.y, p.z) ;

}



bool PickUp::findClothHull( cv::Mat  &depth, cv::Mat &dmap, cv::Mat &mask, vector<cv::Point> &hull, image_geometry::PinholeCameraModel &cm  ){

    ObjectOnPlaneDetector objDet(depth, cm.fx(), cm.fy(), cm.cx(), cm.cy()) ;

    Eigen::Vector3d n ;
    double d ;

    if ( !objDet.findPlane(n, d) ) return false ;

    mask = objDet.findObjectMask(n, d, 0.01, dmap, hull) ;

    return true ;

}



Affine3d PickUp::getSensorPose(const tf::TransformListener &listener, const string &camera)
{
    tf::StampedTransform transform;
    Affine3d pose ;

    try {
        listener.waitForTransform(camera + "_rgb_optical_frame", "base_link", ros::Time(0), ros::Duration(1) );
        listener.lookupTransform(camera + "_rgb_optical_frame", "base_link", ros::Time(0), transform);

        tf::TransformTFToEigen(transform, pose);

        return pose ;
    } catch (tf::TransformException ex) {
        ROS_ERROR("%s",ex.what());
        return pose ;
    }
}


bool PickUp::moveXtionAboveCloth(){

    MoveRobot cmove ;
    cmove.setServoMode(false) ;

    moveAboveTable() ;

    cv::Mat rgb, depth;

    ros::Time ts ;
    image_geometry::PinholeCameraModel cm ;

    if(!grabber->grab(rgb, depth, ts, cm) ){
       cout << " Can't grab image above table!" << endl ;
       return false ;
    }

    prevDepth = depth ;

    cv::Mat dmap, mask ;
    vector<cv::Point> hull ;

    if ( !findClothHull(depth, dmap, mask, hull, cm)){
        cout << "Can't find cloth hull" << endl ;
        return false ;
    }

    tf::TransformListener listener ;
    Affine3d frame = getSensorPose(listener, camera) ;

    Vector3d t = retargetSensor(dmap, hull, cm, defaultDist - defaultTableHeight) ;

    Vector3d target = frame.inverse() * t ;

    double newDist = target.z() +  0.7 ;


    trajectory_msgs::JointTrajectory traj ;
    if ( robot_helpers::planXtionToPose(armName, Vector3d(target.x(), target.y(), newDist ), robot_helpers::lookAt(Vector3d(0, 0, -1)), traj) )
    {
        cmove.execTrajectory(traj) ;
    }

    return true ;
}


bool PickUp::findGraspCandidates( vector<RidgeDetector::GraspCandidate> &gsp, cv::Mat  &depth, image_geometry::PinholeCameraModel &cm  ){

    cv::Mat rgb;

    ros::Time ts ;


    if(!grabber->grab(rgb, depth, ts, cm) ){
       cout << " Can't grab image above table!" << endl ;
       return false ;
    }

    cv::Mat dmap, mask ;
    vector<cv::Point> hull ;

    if ( !findClothHull(depth, dmap, mask, hull, cm)){
        cout << "Can't find cloth hull" << endl ;
        return false ;
    }

    RidgeDetector rdg ;

    rdg.detect(dmap, gsp) ;
    rdg.draw(rgb, gsp) ;

    cv::imwrite("/tmp/gsp.png", rgb) ;

    return true ;
}

geometry_msgs::Quaternion PickUp::findAngle(float theta, Eigen::Matrix4d calib){

    Eigen::Vector4d norm ;
    norm << cos(theta), sin(theta), 0, 0 ;

    Eigen::Vector4d targetN ;
    targetN = calib * norm.normalized() ;

    Eigen::Matrix3d rotMat;
    Eigen::Vector3d Y(targetN.x(), targetN.y(), targetN.z());
    Eigen::Vector3d X;
    Eigen::Vector3d Z(0, 0, -1);

    X = Y.cross(Z);

    rotMat(0, 0)=X.x();
    rotMat(1, 0)=X.y();
    rotMat(2, 0)=X.z();

    rotMat(0, 1)=Y.x();
    rotMat(1, 1)=Y.y();
    rotMat(2, 1)=Y.z();

    rotMat(0, 2)=Z.x();
    rotMat(1, 2)=Z.y();
    rotMat(2, 2)=Z.z();

    //vect << X.x() , X.y() , X.z() ;

    return rotationMatrix3ToQuaternion(rotMat);
}

bool PickUp::isGraspingSucceeded(float threshold){

    MoveRobot cmove ;
    cmove.setServoMode( false);
    moveGripperPointingDown(cmove, armName, parkingPose.x(), parkingPose.y(), parkingPose.z() ) ;

    cv::Mat rgb, depth;

    ros::Time ts ;
    image_geometry::PinholeCameraModel cm ;

    if(!grabber->grab(rgb, depth,  ts, cm)){
        cout<<" Cant grab image!!  " << endl ;
        return false ;
    }

    vector <float> diff ;

    for ( int i = 0; i < 400 ; i++){
        for ( int j = 0; j < 400 ; j++){

            diff.push_back ( ((float)depth.at<unsigned short>(i,j) - (float)prevDepth.at<unsigned short>(i,j)) * (float)(depth.at<unsigned short>(i,j) - (float)prevDepth.at<unsigned short>(i,j))  ) ;

        }
    }

    float sum = 0, meanDepthDiff = 0;

    for ( int i = 0; i < 150000 ; i++)
        sum += diff[i] ;

    meanDepthDiff = (float)sum / 150000.0;

    cout << meanDepthDiff << endl;
    if (meanDepthDiff > threshold)
        return true ;
    openG2() ;
    setGripperState(armName, true);
    return false ;

}

bool PickUp::graspTargetCandidate( vector<RidgeDetector::GraspCandidate> &gsp, cv::Mat &depth, image_geometry::PinholeCameraModel &cm ){

    vector <geometry_msgs::Pose> poses;

    openG2() ;
    setGripperState(armName, true);

    for (unsigned int i = 0 ; i < gsp.size() ; i++ ){

        poses.clear();

        // depth to PC
        unsigned short zval = depth.at<ushort>(gsp[i].y, gsp[i].x) ;

        cv::Point3d val = cm.projectPixelTo3dRay(cv::Point2d(gsp[i].x, gsp[i].y));

        val.x *= zval/1000.0 ;
        val.y *= zval/1000.0 ;
        val.z *= zval/1000.0 ;

        Eigen::Vector3d p(val.x, val.y, val.z) ;

        Eigen::Matrix4d calib = getTranformationMatrix("xtion2_rgb_optical_frame") ;
        Eigen::Vector4d tar (p.x(), p.y(), p.z(), 1) ;
        Eigen::Vector4d targetP ;

        targetP = calib * tar ;

        float offset= 0.10 ;

        ///////////////////

        geometry_msgs::Pose pose ;
       // Eigen::Vector3d vect ;

        pose.orientation = findAngle(gsp[0].alpha, calib) ;
        pose.position.x = targetP.x();
        pose.position.y = targetP.y();
        pose.position.z = targetP.z()+ offset;
        poses.push_back(pose);

        pose.position.z -= offset +0.01 ;
        poses.push_back(pose);

        cout<< " GRASPING POINT = "<< pose.position.x << " " << pose.position.y << " "  <<pose.position.z << endl ;

        if ( moveArmThrough(poses, armName) == -1 ){
            cout<< "cant make the grasp"<< endl ;
            continue ;
        }
        cout << "dominant grasping point i = " <<  i << endl;

        break;
    }

    setGripperState( armName, false) ;

    return true ;
}


bool PickUp::graspClothFromTable(const ros::Duration &dur){

    bool grasp = false;

    ros::Time ts0 = ros::Time::now() + dur ;

    while (!grasp){

        moveXtionAboveCloth() ;

        vector<RidgeDetector::GraspCandidate> gsp ;
        cv::Mat  depth ;
        image_geometry::PinholeCameraModel cm ;

        findGraspCandidates( gsp, depth, cm ) ;
        if( !graspTargetCandidate( gsp, depth, cm ) )
            return false ;

        if ( isGraspingSucceeded(60000.0))
            grasp = true;

        if ( ros::Time::now() > ts0 ) break ;
    }

    moveHomeArm( armName);

    return false ;

}

