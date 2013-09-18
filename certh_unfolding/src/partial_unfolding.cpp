#include "Unfold.h"
#include "fold_detector/FoldDetector.h"
#include "highgui.h"

using namespace cv ;


struct colors{

    int c0;
    int c1;
    int c2;

};

colors show_colors(int);
void grasp_point(bool  , vector<double>& , vector<Eigen::Matrix4d>&  , vector<vector<int> >&  , vector<vector<Point> >&  , vector<vector<bool> >&  , int );

int main(int argc, char **argv) {

    ros::init(argc, argv, "unfolding");
    ros::NodeHandle nh;
    ros::Publisher marker_pub;
    marker_pub = nh.advertise<visualization_msgs::Marker>("/visualization_marker", 0);



    Unfold uf("r2",marker_pub );
    folds f ;

    std::vector < Unfold::grabRGBD > images = uf.grabRGBD360() ;

    int pict_num=images.size();
    int start_pict=0;
    int step=1;
    int i_stop=-1;
    int last_i;
    bool detected=false;

    vector<int> tmp(1,0);
    Point p(0,0);
    vector<Point> P(1,p);
    vector<vector<int> > store(1,tmp);
    vector<vector<Point> > location(1,P);
    vector<bool> b(1,false);
    vector<vector<bool> > current_corner(1,b);
    vector<Eigen::Matrix4d> orientation;
    //int th = 5;
    vector<double> grasp_candidate(3);
    //read the txt with the coordinates of the gripper

    float gripperPositionX = images[0].gripperOrientation(0, 3 ) ;
    float gripperPositionZ = images[0].gripperOrientation(2, 3 ) ;

    int cx,focal=525;
    cx=int(focal*gripperPositionX/gripperPositionZ+316);


    //for pict_num instances
    for (int i=start_pict;i<pict_num;i=i+step){
      //for the case that max<=3 and horizontal edge detection is needed
      last_i=i;

      cv::Mat bgrImage = images[i].rgb ;
      cv::Mat depthMap = images[i].depth ;
      orientation.push_back(images[i].gripperOrientation);

      detected=f.fold_detector( bgrImage, depthMap, i, grasp_candidate, store, location,current_corner,cx );

      if (detected==true){
          i_stop=(i-start_pict)/step;
          break;
      }

    }

    grasp_point (detected , grasp_candidate, orientation ,  store , location , current_corner , cx);



    cout << "GRASP CANDITADE = " <<grasp_candidate.at(0) << " " <<  grasp_candidate.at(1) << " " <<grasp_candidate.at(2)<< endl;
    string rgbFileName = "/tmp/unfold/Grasping.png" ;
    cv::imwrite(rgbFileName, images[grasp_candidate.at(0)].rgb ) ;

    for (int i = -4; i<5 ; i++){
        for ( int j = -4 ; j<5 ; j++){

            images[grasp_candidate.at(0)].rgb.at<cv::Vec3b>(grasp_candidate.at(2)+i,grasp_candidate.at(1)+j)[0] = 1;
            images[grasp_candidate.at(0)].rgb.at<cv::Vec3b>(grasp_candidate.at(2)+i,grasp_candidate.at(1)+j)[1] = 1;
            images[grasp_candidate.at(0)].rgb.at<cv::Vec3b>(grasp_candidate.at(2)+i,grasp_candidate.at(1)+j)[2] = 200;

        }
    }
    Mat winnerPic = images[grasp_candidate.at(0)].rgb;
    Mat winnerPicd = images[grasp_candidate.at(0)].depth;


    //depict the edges
    ret_all r=f.call_main( winnerPic, winnerPicd);

    for (int i=0;i<r.detailed_edges.size();i=i+2){

        if (r.detailed_edges.at(i).size()>0 && r.detailed_edges.at(i).at(0)!=-90){
            for (int j=0;j<r.detailed_edges.at(i).size()-1;j++){
                colors rc=show_colors(i);

                line(winnerPic, Point(r.detailed_edges.at(i).at(j),r.detailed_edges.at(i+1).at(j)), Point(r.detailed_edges.at(i).at(j+1),r.detailed_edges.at(i+1).at(j+1)), Scalar(rc.c0,rc.c1,rc.c2), 1, CV_AA);
            }
        }
    }

    namedWindow("edges",0);
    imshow("edges",winnerPic);
    waitKey(0);
    cout<<"store.size "<<store.size()<<endl;
    for (int i=0;i<store.size();i++){
        cout<<endl;
        for (int j=0;j<store.at(i).size();j++){
            cout<<store.at(i).at(j)<<" ";
        }
    }

    rgbFileName = "/tmp/unfold/GraspingPaint.png" ;
    cv::imwrite(rgbFileName, images[grasp_candidate.at(0)].rgb ) ;

    geometry_msgs::Pose pose ;

    pose.position = getArmPose( uf.getHoldingArm(), "xtion3_rgb_optical_frame").position ;

    pose.orientation =   rotationMatrix4ToQuaternion( images[grasp_candidate.at(0)].gripperOrientation );

    cout<< images[grasp_candidate.at(0)].gripperOrientation << endl;

    printPose(pose);

    moveArm(pose, uf.getHoldingArm(), "xtion3_rgb_optical_frame");

    setServoPowerOff() ;

    return 0 ;
}
