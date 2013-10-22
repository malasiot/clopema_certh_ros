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

    rotateData data;
    image_geometry::PinholeCameraModel cmodel  ;
    vector<double> grasp_candidate ;
    int found ;

    folds folds_ ;
    bool orientLeft;
    Affine3d pose ;
    bool isACorner;
    Unfold unfold;

    FoldDetectorAction(const string &arm): RotateAndGrab("xtion3", arm) {
        data.dataCounter = 0 ;
        found = false ;
        unfold.setHoldingArm(arm);
        cmove.setServoMode(false);
    }

    void process(const pcl::PointCloud<pcl::PointXYZ> &pc ,const Mat &clr, const Mat &depth, const image_geometry::PinholeCameraModel &cm, const ros::Time &ts, Affine3d &tip_pose_in_camera_frame)
    {
        cmodel = cm ;

        cout << data.dataCounter << endl ;

        ros::Duration(0.1).sleep() ;

        data.cloud.push_back(pc) ;
        data.clr.push_back(clr) ;
        data.depth.push_back(depth) ;

//        cv::imwrite(str(boost::format("/tmp/cap_rgb_%d.png") % data.dataCounter), clr) ;
//        cv::imwrite(str(boost::format("/tmp/cap_depth_%d.png") % data.dataCounter), depth) ;
//        pcl::io::savePCDFileBinary(str(boost::format("/tmp/cap_pc_%d.pcd") % data.dataCounter), pc) ;

        Vector3d tip = tip_pose_in_camera_frame * Vector3d(0, 0, 0) ;
        cv::Point2d p = cm.project3dToPixel(cv::Point3d(tip.x(), tip.y(), tip.z())); ;
        data.cx = p.x ;

        data.orientations.push_back(tip_pose_in_camera_frame.matrix()) ;

        vector<double> grasp_cand_(3) ;
        bool detected = folds_.detect( clr, depth, data.dataCounter, grasp_cand_, p.x , orientLeft);

        if ( detected ) {
            found = true ;
            grasp_candidate = grasp_cand_ ;
            isACorner=true;
        }

       data.dataCounter ++ ;
    }




    bool selectGraspPoint(Affine3d &pose, Vector3d &pp, bool & orientLeft, int & index, int  &x , int &y)
    {
        if ( !found )
        {
            grasp_candidate.resize(3) ;

            if( folds_.select(found, grasp_candidate, data.orientations, data.cx, orientLeft))
                    isACorner = true ;
            else
            {
                 orientLeft = detectHorizontalEdge(grasp_candidate, data.cx, data.dataCounter-1,  data.depth[data.dataCounter-1], data.clr[data.dataCounter-1]);
                 isACorner = false;
            }
        }


        int idx = grasp_candidate[0] ;
         x = grasp_candidate[1] ;
         y = grasp_candidate[2] ;

         cv::Mat imc = data.clr[idx];   //cv::imread(str(boost::format("/tmp/cap_rgb_%d.png") % idx), -1) ;
         cv::Mat imd = data.depth[idx];     //cv::imread(str(boost::format("/tmp/cap_depth_%d.png") % idx), -1) ;
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

        pose = camera_frame * Affine3d(data.orientations[idx])  ;

        cout <<"p = "<< p <<endl;

        pp = camera_frame * Vector3d(p.x, p.y, p.z)  ;
        cout <<"pp = " << pp << endl;
        return true ;
    }


    bool failedGrasp(bool &orient){

        cv::Mat rgb ;
        cv::Mat depth ;
        pcl::PointCloud<pcl::PointXYZ> pc ;

        unfold.grabFromXtion(rgb, depth, pc) ;

        data.cloud.push_back(pc) ;
        data.clr.push_back(rgb) ;
        data.depth.push_back(depth) ;
        data.dataCounter++ ;

        orient = detectHorizontalEdge(grasp_candidate, data.cx ,data.dataCounter-1, data.depth[data.dataCounter-1], data.clr[data.dataCounter-1]);

    }
};



int main(int argc, char **argv) {

    ros::init(argc, argv, "unfolding2");
    ros::NodeHandle nh;
   // ros::Publisher marker_pub;

    //ros::AsyncSpinner spinner(4) ;

//    spinner.start() ;

  //  marker_pub = nh.advertise<visualization_msgs::Marker>("/visualization_marker", 0);

    FoldDetectorAction fd("r1") ;

    fd.unfold.parkArmsForGrasping();
    fd.init(Vector3d(-0.12, -0.83, 1.6)) ;
    fd.rotate(-2*M_PI) ;

    Affine3d pose ;
    Vector3d pp ;

    int x = 0,  y= 0;
    Unfold uf("r1");

    int idx;
    if ( fd.selectGraspPoint(pose, pp, fd.orientLeft, idx, x, y) )
    {

        MoveRobot rb ;
        rb.setServoMode(false);
        moveGripper(rb, "r1", pose.translation(), Quaterniond(pose.rotation())) ;


        setGripperState("r2", true);

        if (!uf.graspPoint(fd.data.cloud[idx], x, y, false, !fd.orientLeft ,true, true) ){

            bool done = false ;

            while(!done){

                fd.failedGrasp(fd.orientLeft);
                done = uf.graspPoint(fd.data.cloud[fd.data.dataCounter-1], fd.grasp_candidate[1], fd.grasp_candidate[2], false, !fd.orientLeft, true, true);

            }
        }

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
