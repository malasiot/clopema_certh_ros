#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "FoldDetector.h"
#include <fstream>
#include <iostream>
//#include <string>

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

//ret_all call_main(Mat,Mat);
//int choose_a_corner(a_corners ,vector<vector<int>> ,int ,vector<vector<int>>& ,vector<vector<Point>>&,vector<vector<bool>>& ,int& );
//bool fold_detector(Mat, Mat, double, vector<double>& , vector<double>& ,vector<vector<int> > &, vector<vector<Point> >& , vector<vector<bool> >& ,int );
colors show_colors(int);
void grasp_point(bool , vector<double>& , vector<double>& , vector<vector<int> >& , vector<vector<Point> >& , vector<vector<bool> >&  ,vector<vector <bool> > & , vector<vector<float> >& , int);
int main(){
    folds f;
   // string str="D:/ipthl/rot/rot4";
    int pict_num=70;
    int start_pict=0;
    int step=5;
    int i_stop=-1,k_stop=-1;
    int last_i;
    bool detected=false;
    vector<int> tmp(1,0);
    Point p(0,0);
    vector<Point> P(1,p);
    vector<vector<int> > store(1,tmp);
    vector<vector<Point> > location(1,P);
    vector<bool> b(1,false);
    vector<vector<bool> > current_corner(1,b);
    vector<double> theta;
    double th;
    vector<double> grasp_candidate(3);
    vector<float> fl(1,0);
    vector<vector <float> > depthD(1,fl);
    vector<vector<bool> > side(1,b);
    //read the txt with the coordinates of the gripper
    vector<float> gripper(3);
    fstream file1("axis.txt");
    for (int j=0;j<3;j++){
        file1>>gripper.at(j);

    }
    int cx,cy,focal=525;
    cx=int(focal*gripper.at(0)/gripper.at(2)+316);
    cy=int(focal*gripper.at(1)/gripper.at(2)+237);
    cx=471;
    //cout<<"cx"<<cx<<endl;

    //for pict_num instances
    for (int i=start_pict;i<pict_num;i=i+step){
        //for the case that max<=3 and horizontal edge detection is needed
        last_i=i;
        stringstream ss,ssc;
        //ss<<str;
       // ssc<<str;
        ss<<"cap_depth_000";
        ssc<<"cap_rgb_000";
        if (i<10){
            ss<<"00";
            ssc<<"00";
        }
        else {
            if (i<100){
                ss<<"0";
                ssc<<"0";
            }
        }
        ss<<i;
        ssc<<i;
        ss<<".png";
        ssc<<".png";
        Mat bgrImage=imread(ssc.str());
        Mat depthMap=imread(ss.str(),-1);
        th=(i-start_pict)/step;
        detected=f.fold_detector( bgrImage, depthMap, th, grasp_candidate,store, location,current_corner, side, depthD, cx );

        if (detected==true){
            i_stop=(i-start_pict)/step;
            break;
        }
        for (int k=0;k<store.size();k++){
            for (int l=0;l<store.at(k).size();l++){
                cout<<store.at(k).at(l)<<" ";
            }
            cout<<endl;
        }
        cout<<"/// "<< i<<"///"<<endl;
        for (int k=0;k<depthD.size();k++){
            for (int l=0;l<depthD.at(k).size();l++){
                cout<<depthD.at(k).at(l)<<" ";
            }
            cout<<endl;
        }
    }

    grasp_point( detected , grasp_candidate, theta ,  store , location , current_corner , side, depthD, cx);

    return 0;
}
