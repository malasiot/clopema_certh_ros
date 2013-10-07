#include "fold_detector/FoldDetector.h"
#include "RotateAndGrab.h"
#include <robot_helpers/Utils.h>
#include <robot_helpers/Planners.h>
#include "Unfold.h"
#include <tf_conversions/tf_eigen.h>
#include <certh_libs/cvHelpers.h>
#include <highgui.h>
#include <pcl/io/pcd_io.h>

using namespace cv ;
using namespace Eigen ;
using namespace robot_helpers ;
using namespace std ;
using namespace certh_libs ;


class FoldDetectorAction: public RotateAndGrab
{
public:

      FoldDetectorAction(const string &arm): RotateAndGrab("xtion3", arm) {
        counter = 0 ;
        found = false ;

    }

    void process(const pcl::PointCloud<pcl::PointXYZ> &pc ,const Mat &clr, const Mat &depth, const image_geometry::PinholeCameraModel &cm, const ros::Time &ts, Affine3d &tip_pose_in_camera_frame)
    {
        cmodel = cm ;

        cout << counter << endl ;

        ros::Duration(0.1).sleep() ;

        cv::imwrite(str(boost::format("/tmp/cap_rgb_%d.png") % counter), clr) ;
        cv::imwrite(str(boost::format("/tmp/cap_depth_%d.png") % counter), depth) ;

        pcl::io::savePCDFileBinary(str(boost::format("/tmp/cap_pc_%d.pcd") % counter), pc) ;

        Vector3d tip = tip_pose_in_camera_frame * Vector3d(0, 0, 0) ;
        cv::Point2d p = cm.project3dToPixel(cv::Point3d(tip.x(), tip.y(), tip.z())); ;

        cx = p.x ;
        orientations.push_back(tip_pose_in_camera_frame.matrix()) ;

        vector<double> grasp_cand_(3) ;
        bool detected = folds_.detect( clr, depth, counter, grasp_cand_, p.x , orientLeft);

        if ( detected ) {
            found = true ;
            grasp_candidate = grasp_cand_ ;
            isACorner=true;
        }

       counter ++ ;
    }

    bool selectGraspPoint(Affine3d &pose, Vector3d &pp, bool & orientLeft, int & index, int  &x , int &y)
    {
        if ( !found )
        {
            grasp_candidate.resize(3) ;

            if( folds_.select(found, grasp_candidate, orientations, cx, orientLeft))
                    isACorner = true ;
            else
            {
                 bool orientation = detectHorizontalEdge(grasp_candidate, cx, orientations.size()-1);
                 isACorner = false;
            }
        }


        int idx = grasp_candidate[0] ;
         x = grasp_candidate[1] ;
         y = grasp_candidate[2] ;

        cv::Mat imc = cv::imread(str(boost::format("/tmp/cap_rgb_%d.png") % idx), -1) ;
        cv::Mat imd = cv::imread(str(boost::format("/tmp/cap_depth_%d.png") % idx), -1) ;
        index = idx ;
        // find the coordinates of the point in 3D in the correct frame

        ushort z ;
        if ( !sampleNearestNonZeroDepth(imd, x, y, z) ) return false ;

        cv::Point3d p = cmodel.projectPixelTo3dRay(Point2d(x, y));
        p.x *= z/1000.0 ; p.y *= z/1000.0 ; p.z = z/1000.0 ;

        cv::rectangle(imc, cv::Rect(x-2, y-2, 5, 5), cv::Scalar(255, 0, 255), 2) ;

        cv::imwrite("/tmp/results/gsp.png", imc) ;

        Affine3d camera_frame ;
        tf::StampedTransform tr = robot_helpers::getTranformation(camera + "_rgb_optical_frame") ;
        tf::TransformTFToEigen(tr, camera_frame) ;

        pose = camera_frame * Affine3d(orientations[idx])  ;

        cout <<"p = "<< p <<endl;

        pp = camera_frame * Vector3d(p.x, p.y, p.z)  ;
        cout <<"pp = " << pp << endl;
        return true ;
    }

    int counter ;
    int found ;
    folds folds_ ;
    vector<Matrix4d> orientations ;
    bool orientLeft;
    vector<double> grasp_candidate ;
    Affine3d pose ;
    int cx ;
    image_geometry::PinholeCameraModel cmodel  ;
    bool isACorner;

};



void publishPointMarker(ros::Publisher &vis_pub, const Eigen::Vector3d &p)
{

    visualization_msgs::Marker marker;
        // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/base_link";
    marker.header.stamp = ros::Time::now();

    marker.ns = "grasp point";
    marker.id = 0;

    marker.type = visualization_msgs::Marker::CUBE ;

    marker.pose.position.x = p.x() ;
    marker.pose.position.y = p.y() ;
    marker.pose.position.z = p.z() ;

    marker.scale.x = 0.01;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;

    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    vis_pub.publish(marker);

}



int main(int argc, char **argv) {

    ros::init(argc, argv, "unfolding2");
    ros::NodeHandle nh;
    ros::Publisher marker_pub;

    ros::AsyncSpinner spinner(4) ;

    spinner.start() ;

    marker_pub = nh.advertise<visualization_msgs::Marker>("/visualization_marker", 0);

    FoldDetectorAction action("r1") ;

    action.init(Vector3d(-0.12, -0.83, 1.6)) ;

    action.rotate(-2*M_PI) ;


    Affine3d pose ;
    Vector3d pp ;

    int x = 0,  y= 0;
    Unfold uf("r1",marker_pub);

    int idx;
    if ( action.selectGraspPoint(pose, pp, action.orientLeft, idx, x, y) )
    {

        MoveRobot rb ;
        rb.setServoMode(false);
        moveGripper(rb, "r1", pose.translation(), Quaterniond(pose.rotation())) ;
        pcl::PointCloud<pcl::PointXYZ> pc;
        pcl::io::loadPCDFile(str(boost::format("/tmp/cap_pc_%d.pcd") % idx), pc);
        setGripperState("r2", true);
        uf.graspPoint(pc, x, y, false, !action.orientLeft ,true, true);

// sotiris //
//        publishPointMarker(marker_pub, pp);


//        Vector3d dir(0.001, 0.99, 0) ;
//        dir.normalize() ;


//        KinematicsModel kmodel ;
//        kmodel.init() ;

//        GraspHangingPlanner gsp(kmodel, "r1") ;

//        gsp.cone_aperture = M_PI/20 ;
//        gsp.cone_length = 0.1 ;
//        gsp.offset = 0 ;

//        trajectory_msgs::JointTrajectory traj ;
//        cout<< "ORIENTATION OF GSP = " << action.orientLeft << endl;

//        if ( gsp.plan(pp, dir, traj) )
//           rb.execTrajectory(traj) ;
// /sotiris//

        setServoPowerOff() ;
    }

   // ros::spin() ;

    return 0 ;
}
