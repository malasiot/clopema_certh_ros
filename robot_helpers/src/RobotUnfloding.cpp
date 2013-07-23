#include "robot_helpers/Unfold.h"
#include <opencv2/highgui/highgui.hpp>

#include <cv.h>

#include <Eigen/Core>
#include <Eigen/Eigenvalues> // for cwise access


#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

using namespace std;
namespace robot_helpers{

Unfold::Unfold(const string &armName, ros::Publisher markerPub){
    setHoldingArm(armName);
   marker_pub  = markerPub;
}

Unfold::~Unfold() {
}

int Unfold::setGripperStates(const string &armName  , bool open){
    ros::service::waitForService("/" + armName + "_gripper/set_open");
    clopema_motoros::SetGripperState sopen;
    sopen.request.open=open;
    ros::service::call("/" + armName + "_gripper/set_open", sopen);

    return 0;
}

int Unfold::setGrippersStates( bool open){

    ros::service::waitForService("/r1_gripper/set_open");
    ros::service::waitForService("/r2_gripper/set_open");
    clopema_motoros::SetGripperState sopen;
    sopen.request.open=open;
    ros::service::call("/r1_gripper/set_open", sopen);
    ros::service::call("/r2_gripper/set_open", sopen);

    return 0;
}



geometry_msgs::Quaternion Unfold::rotationMatrix4ToQuaternion(Eigen::Matrix4d matrix){

    float roll , pitch, yaw;

    roll= atan2f(matrix(2, 1),matrix(2, 2) );
    pitch= atan2f(-matrix(2,0),sqrt(pow(matrix(2, 2),2)+pow(matrix(2, 1),2)));
    yaw= atan2f(matrix(1, 0),matrix(0, 0));
    return tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw );
}

geometry_msgs::Quaternion Unfold::rotationMatrix3ToQuaternion(Eigen::Matrix3d matrix){

    float roll , pitch, yaw;

    roll= atan2f(matrix(2, 1),matrix(2, 2) );
    pitch= atan2f(-matrix(2,0),sqrt(pow(matrix(2, 2),2)+pow(matrix(2, 1),2)));
    yaw= atan2f(matrix(1, 0),matrix(0, 0));
    return tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw );
}



void Unfold::rotateHoldingGripper(float angle){

    //Create plan
    clopema_arm_navigation::ClopemaMotionPlan mp;
    MoveRobot cmove;

    mp.request.motion_plan_req.group_name = holdingArm + "_arm";
    mp.request.motion_plan_req.allowed_planning_time = ros::Duration(5.0);

    //Set start state
    getRobotState(mp.request.motion_plan_req.start_state);

    arm_navigation_msgs::SimplePoseConstraint desired_pose;

    desired_pose.header.frame_id = holdingArm + "_ee";
    desired_pose.header.stamp = ros::Time::now();
    desired_pose.link_name = holdingArm + "_ee";

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
    poseToClopemaMotionPlan(mp, desired_pose);

    ROS_INFO("Planning");
    if (!plan(mp))
        return ;

    ROS_INFO("Executing");
    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory = mp.response.joint_trajectory;
    cmove.doGoal(goal);

}

void Unfold::switchArms(){

    string tmp=holdingArm;

    holdingArm=movingArm;
    movingArm=tmp;

}

void Unfold::setHoldingArm(const string &armName){

    holdingArm = armName;
    if(armName == "r1")
        movingArm = "r2";    
    else
        movingArm = "r1";

}


Eigen::Matrix4d Unfold::findLowestPointOrientation(Eigen::Vector4d vector ){

    Eigen::Matrix4d rotMat;
    Eigen::Vector3d N;


    N << -vector.x(), -vector.y(), -vector.z();
    if(holdingArm == "r2")
        N << vector.x(), vector.y(), vector.z();
    N.normalize();

    Eigen::Vector3d A(1,0,-1);
    if(holdingArm == "r2")
         A << -1,0,-1;

    Eigen::Vector3d B=A-(A.dot(N))*N;
    B.normalize();
    Eigen::Vector3d C=N.cross(B);

    C.normalize();


    rotMat(0, 0)=N.x();
    rotMat(1, 0)=N.y();
    rotMat(2, 0)=N.z();

    rotMat(0, 1)=C.x();
    rotMat(1, 1)=C.y();
    rotMat(2, 1)=C.z();

    rotMat(0, 2)=-B.x();
    rotMat(1, 2)=-B.y();
    rotMat(2, 2)=-B.z();


return rotMat;
}

Eigen::Matrix4d Unfold::findGraspingPointOrientation(Eigen::Vector4d vector ){

    Eigen::Matrix4d rotMat;
    Eigen::Vector3d N;


    N << -vector.x(), -vector.y(), -vector.z();
    if(holdingArm == "r2")
        N << vector.x(), vector.y(), vector.z();
    N.normalize();

    Eigen::Vector3d A(-1,0,0);
    if(holdingArm == "r2")
         A << 1,0,0;

    Eigen::Vector3d B=A-(A.dot(N))*N;
    B.normalize();
    Eigen::Vector3d C=N.cross(B);

    C.normalize();


    rotMat(0, 0)=N.x();
    rotMat(1, 0)=N.y();
    rotMat(2, 0)=N.z();

    rotMat(0, 1)=C.x();
    rotMat(1, 1)=C.y();
    rotMat(2, 1)=C.z();

    rotMat(0, 2)=-B.x();
    rotMat(1, 2)=-B.y();
    rotMat(2, 2)=-B.z();


return rotMat;
}

float Unfold::findBias(Eigen::Vector4d vector){

    float theta=atan2f(vector.x(),abs(vector.y()));

    return theta;
}

void mouse_callback( int event, int x, int y, int flags, void* param){
    if(event == CV_EVENT_LBUTTONDOWN){
        cout << x << " " << y << endl;
    }
}


void Unfold::robustPlane3DFit(vector<Eigen::Vector3d> &x, Eigen::Vector3d  &c, Eigen::Vector3d &u)
{
    int i, k, N = x.size() ;
    Eigen::Vector3d u1, u2, u3 ;
    const int NITER = 1 ;

    double *weight = new double [N] ;
    double *res = new double [N] ;

    for( i=0 ; i<N ; i++ ) weight[i] = 1.0 ;

    double wsum = N ;

    for( k=0 ; k<NITER ; k++ )
    {

        c = Eigen::Vector3d::Zero() ;
        Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();

        for( i=0 ; i<N ; i++ )
        {
            const Eigen::Vector3d &P = x[i] ;
            double w = weight[i] ;

            c += w * P ;
        }

        c /= wsum ;

        for( i=0 ; i<N ; i++ )
        {
            const Eigen::Vector3d &P = x[i] ;
            double w = weight[i] ;

            cov += w *  (P - c) * (P - c).adjoint();

        }

        cov *= 1.0/wsum ;

        Eigen::Matrix3d U ;
        Eigen::Vector3d L ;

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver(cov);
        L = eigensolver.eigenvalues() ;
        U = eigensolver.eigenvectors() ;

        // Recompute weights

        u1 = U.col(0) ;
        u2 = U.col(1) ;
        u3 = U.col(2) ;

        for( i=0 ; i<N ; i++ )
        {
            const Eigen::Vector3d &P = x[i] ;

        //    double r = (P -c).dot(P-c) ;
        //    double ss = (P-c).dot(u1);

         //   r -= ss * ss ;

          //  r = sqrt(r) ;

            double r = fabs((P-c).dot(u1));

            weight[i] = r ;
            res[i] = r ;
        }

        // Estimate residual variance
        double sigma ;

        sort(weight, weight + N) ;

        sigma = weight[N/2]/0.6745 ;

        // Update weights using Hubers scheme

        wsum = 0.0 ;

        for( i=0 ; i<N ; i++ )
        {

            double r = fabs(res[i]) ;

            if ( r <= sigma )  weight[i] = 1.0 ;
            else if ( r > sigma && r <= 3.0*sigma ) weight[i] = sigma/r ;
            else weight[i] = 0.0 ;

            wsum += weight[i] ;
        }
    }

    u = u1 ;

    delete weight ;
    delete res ;
}

 Eigen::Vector3d Unfold::computeNormal(const pcl::PointCloud<pcl::PointXYZ> &pc, int x, int y)
{
    const int nrmMaskSize = 6 ;
    int w = pc.width, h = pc.height ;

    vector<Eigen::Vector3d> pts ;

    for(int i = y - nrmMaskSize ; i<= y + nrmMaskSize ; i++  )
        for(int j = x - nrmMaskSize ; j<= x + nrmMaskSize ; j++  )
        {
            if ( i < 0 || j < 0 || i > h-1 || j > w-1 ) continue ;

            pcl::PointXYZ val = pc.at(j, i) ;

            if ( !pcl_isfinite(val.z) ) continue ;

            pts.push_back(Eigen::Vector3d(val.x, val.y, val.z)) ;
        }

    if ( pts.size() < 3 ) return Eigen::Vector3d() ;

    Eigen::Vector3d u, c ;
    robustPlane3DFit(pts, c, u);

    if ( u(2) > 0 ) u = -u ;

    return u ;

}


 bool Unfold::pointInsideCone(const Eigen::Vector3d &x, const Eigen::Vector3d &apex, const Eigen::Vector3d &base, float aperture)
{
    // This is for our convenience
    float halfAperture = aperture/2.f;

    // Vector pointing to X point from apex
    Eigen::Vector3d apexToXVect = apex - x ;
    apexToXVect.normalize() ;

    // Vector pointing from apex to circle-center point.
    Eigen::Vector3d axisVect = apex - base;
    axisVect.normalize() ;

    // determine angle between apexToXVect and axis.

    double d = apexToXVect.dot(axisVect) ;

    bool isInInfiniteCone = fabs(d) > cos(halfAperture) ;

    if(!isInInfiniteCone) return false;

    // X is contained in cone only if projection of apexToXVect to axis
    // is shorter than axis.
    // We'll use dotProd() to figure projection length.

    bool isUnderRoundCap = apexToXVect.dot(axisVect)  < 1 ;

    return isUnderRoundCap;
}

void Unfold::findMeanShiftPoint(const pcl::PointCloud<pcl::PointXYZ> &depth, int x0, int y0, int &x1, int &y1, double radius, double variance, int maxIter)
{
   pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

    boost::shared_ptr <pcl::PointCloud<pcl::PointXYZ> > cloud(new pcl::PointCloud<pcl::PointXYZ>(depth)) ;

    kdtree.setInputCloud (cloud);

    int w = depth.width, h = depth.height ;

    pcl::PointXYZ p = depth.at(x0, y0) ;
    Eigen::Vector3d center(p.x, p.y, p.z) ;

    int iter = 0 ;
    double centerDist ;

    do
    {

        Eigen::Vector3d newCenter(0, 0, 0) ;

        vector<int> pointIdxRadiusSearch;
        vector<float> pointRadiusSquaredDistance;

        pcl::PointXYZ center_(center.x(), center.y(), center.z()) ;
        kdtree.radiusSearch (center_, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) ;

        int n = pointIdxRadiusSearch.size() ;

        double denom = 0.0 ;

        for(int i=0 ; i<n ; i++ )
        {
            pcl::PointXYZ p = depth.at(pointIdxRadiusSearch[i]) ;
            Eigen::Vector3d pc(p.x, p.y, p.z) ;

            double ep = exp(-pointRadiusSquaredDistance[i]/ (2.0 * variance));

            denom += ep ;

            newCenter += ep * pc ;
        }

        newCenter /= denom ;

        centerDist = (newCenter - center).norm() ;

        center = newCenter ;

        ++iter ;

        //cout << centerDist << endl ;

    } while ( centerDist > variance && iter < maxIter ) ;

    vector<int> pointIdxNNSearch;
    vector<float> pointNNSquaredDistance;

    pcl::PointXYZ center_(center.x(), center.y(), center.z()) ;
    if ( kdtree.nearestKSearch(center_, 1, pointIdxNNSearch, pointNNSquaredDistance) > 0 )
    {
        int idx = pointIdxNNSearch[0] ;

        y1 = idx / w ;
        x1 =  idx - y1 * w ;

    }
}


bool Unfold::findLowestPoint(const pcl::PointCloud<pcl::PointXYZ> &depth, const Eigen::Vector3d &orig, const Eigen::Vector3d &base, float apperture, Eigen::Vector3d &p, Eigen::Vector3d &n)
{

    int w = depth.width, h = depth.height ;

    int max_y = -1 ;
    int best_j = -1, best_i = -1 ;

    bool found = false ;

    double maxv = -DBL_MAX ;

    for(int j=0 ; j<w ; j++ )
    {

        for(int i=0 ; i<h ; i++)
        {
            pcl::PointXYZ val = depth.at(j, i) ;

            if ( !pcl_isfinite(val.z) ) continue ;

            Eigen::Vector3d p(val.x, val.y, val.z) ;

            // test whether the point lies within the cone

            if ( !pointInsideCone(p, orig, base, apperture) ) continue ;

            // project the point on the cone axis to determine how low it is.

            double s = (p - orig).dot(base - orig) ;

            if ( s > maxv )
            {
                maxv = s ;
                best_j = j ;
                best_i = i ;
                found = true ;
            }

        }
    }

    if ( !found ) return false ;

    pcl::PointXYZ p_ = depth.at(best_j, best_i) ;

    int ox, oy ;

    // find densest point cluster in a 3cm sphere around the detected point
    findMeanShiftPoint(depth, best_j, best_i, ox, oy, 0.05) ;

    // compute normal vector around this point
    n = computeNormal(depth, ox, oy) ;

    p = Eigen::Vector3d(p_.x, p_.y, p_.z) ;
}




//bool Unfold::findLowestPoint(const pcl::PointCloud<pcl::PointXYZ> &depth, const Eigen::Vector3d &orig, const Eigen::Vector3d &base, float apperture, Eigen::Vector3d &p, Eigen::Vector3d &n, cv::Mat depthMap){

//    int w = depth.width, h = depth.height ;

//    int best_i=-1, best_j=-1;
//    float best_x=-1, best_y=-1, best_z=-1;


//    bool found = false ;
//    float minx = 10 ;
//    for(int j=66 ; j<626 ; j++ )
//    {


//        for(int i=100 ; i<350 ; i++)
//        {
//            pcl::PointXYZ val = depth.at(j, i) ;

//            if ( val.z<1 || val.z>1.5 ) continue ;

//            if ( minx>val.x )
//            {
//                minx = val.x ;
//                best_j = j ;
//                best_i = i ;

//                found = true ;
//            }

//        }
//    }



//    for(int i=0; i<depthMap.rows; ++i)
//        for(int j=0; j<depthMap.cols; ++j)
//            if( (depthMap.at<unsigned short>(i, j) < 900) || (depthMap.at<unsigned short>(i, j) > 1400) )
//                depthMap.at<unsigned short>(i, j) = 0;

//    cv::TermCriteria criteria(1, 10, 0.1);
//    cv::Rect rect1(best_j-10, best_i-10, 20, 20);
//    cv::meanShift(depthMap, rect1, criteria);
//    best_j = rect1.x + rect1.width/2;
//    best_i = rect1.y + rect1.height/2;
////    cout<< "best2= (" << best_j <<" , "<< best_i <<")"<<endl;

//    pcl::PointXYZ p_ = depth.at(best_j, best_i ) ;
//    n = computeNormal(depth, best_j, best_i ) ;
//    p = Eigen::Vector3d(p_.x, p_.y, p_.z) ;

//    if ( !found ) return false ;

//}




void printPose(geometry_msgs::Pose p){

    cout << "pose = (" << p.position.x << ", "<< p.position.y << ", "<< p.position.z << " )" <<endl;

}


int Unfold::graspLowestPoint(bool lastMove){


    moveArms(movingArmPose(), holdingArmPose(), movingArm, holdingArm );

    setGripperStates(movingArm , true);
    ros::Duration(1.0).sleep();

    //starting position
    bool grasp = false;
    camera_helpers::OpenNICaptureAll grabber("xtion3") ;
    geometry_msgs::Pose desPose;
    vector <geometry_msgs::Pose> poses;
    Eigen::Vector4d targetP;
    Eigen::Matrix4d rotMat;
    grabber.connect() ;


    while(!grasp){

        cv::Mat rgb, depth;
        pcl::PointCloud<pcl::PointXYZ> pc;
        cv::Mat R = cv::Mat(4, 4, CV_32FC1, cv::Scalar::all(0));
        ros::Time ts(0);
        image_geometry::PinholeCameraModel cm;

        //grabbing image and making the fit of lowest point
        Eigen::Matrix4d calib = getTranformationMatrix("xtion3_rgb_optical_frame");

        tf::StampedTransform st;
        Eigen::Vector3d top , bottom, p, n;
        float angle=0.5;

        st= getTranformation(holdingArm + "_ee", "xtion3_rgb_optical_frame");

        top.x()=st.getOrigin().x();
        top.y()=st.getOrigin().y();
        top.z()=st.getOrigin().z();

        bottom = top;
        bottom.x()-=1;

        grabber.grab(rgb, depth, pc, ts, cm);
        if(findLowestPoint(pc, top, bottom, angle, p, n)== false)
            cout<< "Cant find lowest point"<< endl;

        publishLowestPointMarker(marker_pub,p ,n );

        Eigen::Vector4d tar(p.x(), p.y(), p.z(), 1);       
        targetP = calib * tar;

        Eigen::Vector4d norm (n.x(), n.y(), n.z(), 0);
        Eigen::Vector4d targetN;
        targetN = calib * norm.normalized();
        targetN.normalized();

        rotMat = findLowestPointOrientation(targetN);


        desPose.orientation = rotationMatrix4ToQuaternion(rotMat);
        desPose.position.x = targetP.x() + rotMat(0, 0) * 0.03 - rotMat(0, 2) * 0.07;
        desPose.position.y = targetP.y() + rotMat(1, 0) * 0.03 - rotMat(1, 2) * 0.07;
        desPose.position.z = targetP.z() + rotMat(2, 0) * 0.03 - rotMat(2, 2) * 0.07;
        poses.push_back(desPose);

        desPose.position.x = targetP.x() + rotMat(0, 2) * 0.045 + rotMat(0, 0) * 0.03;
        desPose.position.y = targetP.y() + rotMat(1, 2) * 0.045 + rotMat(1, 0) * 0.03;
        desPose.position.z = targetP.z() + rotMat(2, 2) * 0.045 + rotMat(2, 0) * 0.03;
        poses.push_back(desPose);

        st= getTranformation(holdingArm + "_ee");

        //grasp lowest point
       addConeToCollisionModel(holdingArm, st.getOrigin().z() - desPose.position.z - 0.2, 0.1 );

        if(moveArmThrough(poses, movingArm) == -1){
            resetCollisionModel();
            cout<< "BIAS : " <<findBias(targetN)<<endl;
            rotateGripper(-findBias(targetN), holdingArm);
            continue;
        }
        resetCollisionModel();
        break;

    }
    setGripperStates( movingArm, false);

    //Make a circle trajectory
    desPose.orientation = rotationMatrix3ToQuaternion(vertical());

    geometry_msgs::Pose desPos1, desPos2;
    float radious=getArmsDistance();

    desPos1 = getArmPose("r1");
    desPos2 = getArmPose("r2");

    if(holdingArm == "r1"){
        desPos2.position.x-=radious/2.0;
        desPos1.position.x-=radious/2.0;


    }
    else{
        desPos1.position.x+=radious/2.0;
        desPos2.position.x+=radious/2.0;

    }

    ros::Duration(0.5).sleep();
    moveArmsNoTearing(desPos1, desPos2);

    ros::Duration(0.5).sleep();
    moveArmBetweenSpheres(movingArm, true,  desPose);
    if ( lastMove == true )
        return -1;
    desPos1 = getArmPose("r1");
    desPos2 = getArmPose("r2");


    cout<< "holding arm is = " << holdingArm << endl;

    if(holdingArm == "r1"){
        desPos2.position.x-=radious/2.0f;
        desPos1.position.z-=0.866025404*radious;
    }
    else{
        desPos1.position.x+=radious/2.0f;
        desPos2.position.z-=0.866025404*radious;
    }

    ros::Duration(1.5).sleep();
    if ( moveArms(desPos1, desPos2) == -1){
        cout<<"ABORDING..."<<endl;
        return -1;
    }

    setGripperStates(holdingArm, true);

    switchArms();


return 0;

}

int Unfold::graspPoint(const  pcl::PointCloud<pcl::PointXYZ> &pc,  int x, int y , bool lastMove  ){

    Eigen::Vector3d n;
    pcl::PointXYZ val = pc.at(x, y) ;
    Eigen::Vector3d p(val.x, val.y, val.z);
    geometry_msgs::Pose desPose;
    vector <geometry_msgs::Pose> poses;
    Eigen::Matrix4d rotMat;
    Eigen::Vector4d targetP;

    if ( pcl_isfinite(val.z) || val.z > 2.0 || val.z < 0.5 )
        cout<< "Invalid Coords" << endl;

    n = computeNormal(pc, x, y) ;

    Eigen::Matrix4d calib = getTranformationMatrix("xtion3_rgb_optical_frame");

    publishLowestPointMarker(marker_pub,p ,n );

    Eigen::Vector4d tar(p.x(), p.y(), p.z(), 1);
    targetP = calib * tar;

    Eigen::Vector4d norm (n.x(), n.y(), n.z(), 0);
    Eigen::Vector4d targetN;
    targetN = calib * norm.normalized();
    targetN.normalized();

    rotMat = findGraspingPointOrientation(targetN);

    desPose.orientation = rotationMatrix4ToQuaternion(rotMat);

    desPose.position.x = targetP.x() + rotMat(0, 0) * 0.03 - rotMat(0, 2) * 0.07;
    desPose.position.y = targetP.y() + rotMat(1, 0) * 0.03 - rotMat(1, 2) * 0.07;
    desPose.position.z = targetP.z() + rotMat(2, 0) * 0.03 - rotMat(2, 2) * 0.07;
    poses.push_back(desPose);

    desPose.position.x = targetP.x() + rotMat(0, 2) * 0.045 + rotMat(0, 0) * 0.03;
    desPose.position.y = targetP.y() + rotMat(1, 2) * 0.045 + rotMat(1, 0) * 0.03;
    desPose.position.z = targetP.z() + rotMat(2, 2) * 0.045 + rotMat(2, 0) * 0.03;
    poses.push_back(desPose);

    if(moveArmThrough(poses, movingArm) == -1){
        cout<< "ABORDING..."<< endl;
        return 0;
    }
    desPose.orientation = rotationMatrix3ToQuaternion(vertical());

    moveArmBetweenSpheres(movingArm, true ,desPose);

    geometry_msgs::Pose desPos1, desPos2;
    float radious=getArmsDistance();

    desPos1 = getArmPose("r1");
    desPos2 = getArmPose("r2");

    if(holdingArm == "r1"){
        desPos2.position.x-=radious/2.0;
        desPos1.position.x-=radious/2.0;
    }
    else{
        desPos1.position.x+=radious/2.0;
        desPos2.position.x+=radious/2.0;
    }

    ros::Duration(0.5).sleep();
    moveArmsNoTearing(desPos1, desPos2);

    ros::Duration(0.5).sleep();
    moveArmBetweenSpheres(movingArm, true,  desPose);
    if ( lastMove == true )
        return -1;

    desPos1 = getArmPose("r1");
    desPos2 = getArmPose("r2");


    cout<< "holding arm is = " << holdingArm << endl;

    if(holdingArm == "r1"){
        desPos2.position.x-=radious/2.0f;
        desPos1.position.z-=0.866025404*radious;
    }
    else{
        desPos1.position.x+=radious/2.0f;
        desPos2.position.z-=0.866025404*radious;
    }

    ros::Duration(1.5).sleep();
    if ( moveArms(desPos1, desPos2) == -1){
        cout<<"ABORDING..."<<endl;
        return -1;
    }

    setGripperStates(holdingArm, true);

    switchArms();

}


bool Unfold::grabFromXtion(cv::Mat rgb, cv::Mat depth, pcl::PointCloud<pcl::PointXYZ> pc ){

    camera_helpers::OpenNICaptureAll grabber("xtion3") ;
    grabber.connect() ;
    ros::Duration(0.3).sleep();
    ros::Time ts(0);
    image_geometry::PinholeCameraModel cm;

    if(!grabber.grab(rgb, depth, pc, ts, cm))
        return false;
    return true;

}


}

