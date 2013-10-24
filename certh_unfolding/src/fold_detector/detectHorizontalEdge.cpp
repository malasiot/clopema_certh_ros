#include <tf_conversions/tf_eigen.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "FoldDetector.h"
#include <iostream>

//#include <string>

using namespace cv;
using namespace std;

struct colors{

    int c0;
    int c1;
    int c2;

};

colors show_colors(int);


bool detectHorizontalEdge( vector<double>& grasp_candidate, int cx , int n, Mat winnerPicd ,Mat winnerPic,int hand, int lowl){
    folds f;
    bool side;
    cout<<"go for horizontal"<<endl;
//    cv::Mat winnerPic= cv::imread(str(boost::format("/tmp/cap_rgb_%d.png") % n), -1) ;
//    cv::Mat winnerPicd = cv::imread(str(boost::format("/tmp/cap_depth_%d.png") % n), -1) ;

    //detect the edges
    ret_all r=f.call_main( winnerPic, winnerPicd);
    //detect the most horizontal edge
    //(find the edge with tthe minimum inclination)
    double min_f=0.0;
    int i_min=-1;

	int dif=int(2*(cx-lowl)/3);
	int limit2=lowl+dif;

	dif=int((cx-lowl)/10);
	int limitL=lowl+dif;


    for (int i=0;i<r.edges_t.size();i++){
        if (r.edges_t.at(i).at(0)!=-90){
            double ar=r.edges_t.at(i).at(1)-r.edges_t.at(i).at(3);
            double par=r.edges_t.at(i).at(0)-r.edges_t.at(i).at(2);
            if (par!=0){
                double f=abs(ar/par);
				bool expr1=(hand==1 && r.edges_t.at(i).at(0)<cx && r.edges_t.at(i).at(2)<cx);//<------------------------------------
				bool expr2=(hand==2 && r.edges_t.at(i).at(0)<limit2 && r.edges_t.at(i).at(2)<limit2);//<--------------------------
				bool expr3=(r.edges_t.at(i).at(0)>limitL && r.edges_t.at(i).at(2)>limitL);
                if ((expr1==true || expr2==true) && expr3){//if it does not start from the grasping point
                    if (min_f<f){
                        cout<<"f= "<<f;
                        min_f=f;
                        i_min=i;
                        side=true;//<---------------------------------------------------------
                        if ((ar/par)<0){
                            side=false;
                        }//<----------------------
                    }
                }
            }//if par
        }//if at(0)!=-90
    }//for
    //detect the most horizontal subedge
    int i_min2=0;
    min_f=0;
    for (int i=0;i<r.detailed_edges.at(2*i_min).size()-1;i++){
        double ar=r.detailed_edges.at(2*i_min+1).at(i) - r.detailed_edges.at(2*i_min+1).at(i+1) ;
        double par= r.detailed_edges.at(2*i_min).at(i) - r.detailed_edges.at(2*i_min).at(i+1) ;
        if (par != 0){
            double f= abs( ar/par );
            if ( min_f < f ){
                min_f=f;
                i_min2=i;
            }
        }
    }

   // cout<<"imin "<<i_min<<" f= "<<double(min_f)<<" imin2 "<<i_min2;

    grasp_candidate.at(0)=n;
    grasp_candidate.at(1)=r.detailed_edges.at(2*i_min).at(i_min2);
    grasp_candidate.at(2)=r.detailed_edges.at(2*i_min+1).at(i_min2);

            /////depict the edges
            for (int i=0;i<r.detailed_edges.size();i=i+2){

                if (r.detailed_edges.at(i).size()>0){
                    for (int j=0;j<r.detailed_edges.at(i).size()-1;j++){
                        colors rc=show_colors(i);

                        line(winnerPic, Point(r.detailed_edges.at(i).at(j),r.detailed_edges.at(i+1).at(j)), Point(r.detailed_edges.at(i).at(j+1),r.detailed_edges.at(i+1).at(j+1)), Scalar(rc.c0,rc.c1,rc.c2), 1, CV_AA);
                    }
                }
            }


            for (int o1=-4;o1<5;o1++){
                for(int o2=-4;o2<5;o2++){
                    for (int c=0;c<2;c++){
                        winnerPic.at<Vec3b>(grasp_candidate.at(2)+o1,grasp_candidate.at(1)+o2)[c]=0;
                    }
                    winnerPic.at<Vec3b>(grasp_candidate.at(2)+o1,grasp_candidate.at(1)+o2)[2]=255;
                }
            }


            colors rc=show_colors(11);
            line(winnerPic, Point(r.edges_t.at(i_min).at(0),r.edges_t.at(i_min).at(1)), Point(r.edges_t.at(i_min).at(2),r.edges_t.at(i_min).at(3)), Scalar(rc.c0,rc.c1,rc.c2), 4, CV_AA);

            imwrite("/tmp/results/cap_rgb_edge_.png",winnerPic);

//            namedWindow("winner",0);
//            imshow("winner",winnerPic);
//            waitKey();



    return side;


}
