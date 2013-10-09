#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "FoldDetector.h"
#include <fstream>
#include <iostream>
//#include <string>
#include <tf_conversions/tf_eigen.h>
using namespace cv;
using namespace std;

//struct a_corners{
//	vector<int> certain_c;
//	vector<int> str_l_c;
//	vector<int> distant_c;
//};
//
//struct ret_all{
//	vector<vector<int>> junctions;
//	vector<vector<int>> detailed_edges;
//	vector<int> disconnected;
//	vector<vector<int>> edges_of_junct;
//	vector<vector<int>> edges_t;
//	vector<vector<int>> table;
//	a_corners a_corn;
//};


struct colors{
	
	int c0;
	int c1;
	int c2;

};
colors show_colors(int);
ret_all call_main(Mat,Mat);
int choose_a_corner(a_corners, depths ,vector<vector<int> > ,int ,vector<vector<int> >& ,vector<vector<Point> >&,vector<vector<bool> >& ,vector<vector <bool> > & , vector<vector<float> >& ,int& ,ret_all, vector<vector< int> > & , vector<vector <Point> > & , Mat);

bool folds::fold_detector(Mat bgrImage, Mat depthMap, int th, vector<double>& grasp_candidate, vector<vector<int> >& store, vector<vector<Point> >& location, vector<vector<bool> >& current_corner,  vector<vector <bool> > & side, vector<vector<float> >& depthD, int cx, vector<vector< int> > & radius, vector<vector <Point> > & Points, bool &orientLeft  ){

    bool ret;
	int i_stop=-1,k_stop=-1;
    folds f;
	ret_all r=call_main( bgrImage, depthMap );
	
		
		//i_stop shows the image where more than 6 votes for a point are gathered
		//if there is no such a point yet, i_stop==-1
		
        i_stop=choose_a_corner(r.a_corn,r.d, r.junctions,th,store,location,current_corner, side, depthD,k_stop,r, radius, Points, bgrImage);

		cout<<"!";
        if (i_stop!=-1 && location.at(i_stop).at(k_stop).x<cx && current_corner.at(i_stop).at(k_stop)==true && depthD.at(i_stop).at(k_stop)>300 && radius.at(i_stop).at(k_stop)>2){//<---
        //if (i_stop!=-1 && location.at(i_stop).at(k_stop).x<cx && current_corner.at(i_stop).at(k_stop)==true && depthD.at(i_stop).at(k_stop)>3000){
			
            ret=true;
			grasp_candidate.at(0)=i_stop;
//			grasp_candidate.at(1)=location.at(i_stop).at(k_stop).x;
//			grasp_candidate.at(2)=location.at(i_stop).at(k_stop).y;
            grasp_candidate.at(1)=Points.at(i_stop).at(k_stop).x;
            grasp_candidate.at(2)=Points.at(i_stop).at(k_stop).y;
            orientLeft = side.at(i_stop).at(k_stop);
            //cout<<"COORDINATES "<<grasp_candidate.at(1)<<" "<<grasp_candidate.at(2)<<endl;
            //cout<<"i_stop "<<i_stop<<endl;
            //depict
            cv::Mat winnerPic = cv::imread(str(boost::format("/tmp/cap_rgb_%d.png") % grasp_candidate.at(0)), -1) ;
            cv::Mat winnerPicd = cv::imread(str(boost::format("/tmp/cap_depth_%d.png") % grasp_candidate.at(0)), -1) ;

            ret_all r=f.call_main( winnerPic, winnerPicd);

            for (int i=0;i<r.detailed_edges.size();i=i+2){

                if (r.detailed_edges.at(i).size()>0 && r.detailed_edges.at(i).at(0)!=-90){
                    for (int j=0;j<r.detailed_edges.at(i).size()-1;j++){
                        colors rc=show_colors(i);

                        line(winnerPic, Point(r.detailed_edges.at(i).at(j),r.detailed_edges.at(i+1).at(j)), Point(r.detailed_edges.at(i).at(j+1),r.detailed_edges.at(i+1).at(j+1)), Scalar(rc.c0,rc.c1,rc.c2), 1, CV_AA);
                    }
                }
            }

            imwrite("/tmp/results/cap_rgb_point_.png",winnerPic);
		}
		else{
			ret=false;
			
		}
		
		return ret;
}
