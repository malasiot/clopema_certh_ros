#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "FoldDetector.h"
#include <iostream>
# include <tf_conversions/tf_eigen.h>

using namespace cv;
using namespace std;

struct colors{
	
	int c0;
	int c1;
	int c2;

};

colors show_colors(int);

bool folds::grasp_point(bool detected , vector<double>& grasp_candidate, vector<Eigen::Matrix4d>& orientation, vector<vector<int> >& store , vector<vector<Point> >& location , vector<vector<bool> >& current_corner , vector<vector<bool> >& side, vector<vector<float> >& depthD, int cx, bool &orientLeft,vector<vector< int> > & radius, vector<vector <Point> > & Points,int hand,int lowl){

    folds f;
//	int start_pict=100;
//	int step=5;
//	int last_i=170;
	int dif=int(2*(cx-lowl)/3);
	int limit2=lowl+dif;

	dif=int((cx-lowl)/10);
	int limitL=lowl+dif;

	if (detected==false){
		int max=0,kmax=0,lmax=0;
		for (int k=0;k<store.size();k++){
			for (int l=0;l<store.at(k).size();l++){
				bool expr=(hand==1 && location.at(k).at(l).x<cx) || (hand==2 && location.at(k).at(l).x<limit2);//<--------------------------
                if (max<store.at(k).at(l) && expr==true && depthD.at(k).at(l)>35 && radius.at(k).at(l)>2 && location.at(k).at(l).x>limitL){//<-------------------------
                //if (max<store.at(k).at(l) && location.at(k).at(l).x<cx ){
					//make sure that the point is detected to this particular image
					if (current_corner.at(k).at(l)==true){
						max=store.at(k).at(l);
						kmax=k;
						lmax=l;
					}
				}
			}
		}
        if (max>3){
            cout<<endl<< " max>3 "<<endl;
            grasp_candidate.at(0)=kmax;
//            grasp_candidate.at(1)=location.at(kmax).at(lmax).x;
//            grasp_candidate.at(2)=location.at(kmax).at(lmax).y;
            grasp_candidate.at(1)=Points.at(kmax).at(lmax).x;
            grasp_candidate.at(2)=Points.at(kmax).at(lmax).y;
            cout<<" SIDE "<<side.at(kmax).at(lmax);
            orientLeft = side.at(kmax).at(lmax);

            ///////depict
//            cv::Mat winnerPic = cv::imread(str(boost::format("/tmp/cap_rgb_%d.png") % grasp_candidate.at(0)), -1) ;
//            cv::Mat winnerPicd = cv::imread(str(boost::format("/tmp/cap_depth_%d.png") % grasp_candidate.at(0)), -1) ;
            Mat winnerPic= clr[grasp_candidate.at(0)];
            Mat winnerPicd= depth[grasp_candidate.at(0)];

                ret_all r=f.call_main( winnerPic, winnerPicd);
                //cout<<"ind= "<<ind<<" max= "<<max;
                for (int i=0;i<r.detailed_edges.size();i=i+2){

                    if (r.detailed_edges.at(i).size()>0 && r.detailed_edges.at(i).at(0)!=-90){
                        for (int j=0;j<r.detailed_edges.at(i).size()-1;j++){
                            colors rc=show_colors(i);

                            line(winnerPic, Point(r.detailed_edges.at(i).at(j),r.detailed_edges.at(i+1).at(j)), Point(r.detailed_edges.at(i).at(j+1),r.detailed_edges.at(i+1).at(j+1)), Scalar(rc.c0,rc.c1,rc.c2), 1, CV_AA);
                        }
                    }
                }

//                imwrite(str(boost::format("/tmp/results/f_gsp_rgb%d.png") % hand), winnerPic);
//                imwrite(str(boost::format("/tmp/results/f_gsp_depth%d.png") % hand), winnerPicd);
            ////////////////////////
            cout<<endl<< "max"<<max<<endl;

            return true;
        }
        return false ;




		


    }

}
