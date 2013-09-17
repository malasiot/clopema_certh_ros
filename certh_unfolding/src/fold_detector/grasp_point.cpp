#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "FoldDetector.h"
#include <iostream>

using namespace cv;
using namespace std;

struct colors{
	
	int c0;
	int c1;
	int c2;

};

colors show_colors(int);

void grasp_point(bool detected , vector<double>& grasp_candidate, vector<Eigen::Matrix4d>& orientation, vector<vector<int> >& store , vector<vector<Point> >& location , vector<vector<bool> >& current_corner , int cx){


    folds f;
	int start_pict=100;
	int step=5;
	int last_i=170;
    cx=471;

	if (detected==false){
		int max=0,kmax=0,lmax=0;
		for (int k=0;k<store.size();k++){
			for (int l=0;l<store.at(k).size();l++){
				
				if (max<store.at(k).at(l) && location.at(k).at(l).x<cx ){
					//make sure that the point is detected to this particular image
					if (current_corner.at(k).at(l)==true){
						max=store.at(k).at(l);
						kmax=k;
						lmax=l;
					}
				}
			}
		}

        grasp_candidate.at(0)=kmax;
        grasp_candidate.at(1)=location.at(kmax).at(lmax).x;
        grasp_candidate.at(2)=location.at(kmax).at(lmax).y;

        cout<<"MAX "<< max<<endl;
//		if (max>3){
//			cout<<"MAX>3";
//			//the number of the picture with the selected corner
//			int ind=kmax*step+start_pict;
//			//depict the winner
//			stringstream ssc;
//			stringstream ss;
			
//        //	ssc<<str;
//        //	ss<<str;
//            ssc<<"cap_rgb_000";
//            ss<<"cap_depth_000";
//			if (ind<10){
//				ssc<<"00";
//				ss<<"00";
//			}
//			else {
//				if (ind<100){
//					ssc<<"0";
//					ss<<"0";
//				}
//			}
//			ssc<<ind;
//			ss<<ind;
//			ssc<<".png";
//			ss<<".png";
//            Mat winnerPic=imread(ssc.str());
//            Mat winnerPicd=imread(ss.str(),-1);
//			for (int o1=-4;o1<5;o1++){
//				for(int o2=-4;o2<5;o2++){
//					for (int c=0;c<2;c++){
//						winnerPic.at<Vec3b>(location.at(kmax).at(lmax).y+o1,location.at(kmax).at(lmax).x+o2)[c]=0;
//					}
//					winnerPic.at<Vec3b>(location.at(kmax).at(lmax).y+o1,location.at(kmax).at(lmax).x+o2)[2]=255;
//				}
//			}
//			namedWindow("winner",0);
//			imshow("winner",winnerPic);
//			waitKey();
//			//depict the edges
//			ret_all r=f.call_main( winnerPic, winnerPicd);
//			cout<<"ind= "<<ind<<" max= "<<max;
//			for (int i=0;i<r.detailed_edges.size();i=i+2){
		
//				if (r.detailed_edges.at(i).size()>0 && r.detailed_edges.at(i).at(0)!=-90){
//					for (int j=0;j<r.detailed_edges.at(i).size()-1;j++){
//						colors rc=show_colors(i);
			
//						line(winnerPic, Point(r.detailed_edges.at(i).at(j),r.detailed_edges.at(i+1).at(j)), Point(r.detailed_edges.at(i).at(j+1),r.detailed_edges.at(i+1).at(j+1)), Scalar(rc.c0,rc.c1,rc.c2), 1, CV_AA);
//					}
//				}
//			}


//			namedWindow("winner",0);
//			imshow("winner",winnerPic);
//			waitKey();
//            grasp_candidate.at(0)=kmax;
//			grasp_candidate.at(1)=location.at(kmax).at(lmax).x;
//			grasp_candidate.at(2)=location.at(kmax).at(lmax).y;

//		}
		//if max<=3 then the result is not considered valid enough,so an 
		//horizontal edge is detected
//        else{
//			cout<<"Edge";
//			//the number of the picture with the selected corner
//			int ind=last_i;
//			//depict the winner
//			stringstream ssc;
//			stringstream ss;
//            //ssc<<str;
//            //ss<<str;
//            ssc<<"cap_rgb_000";
//            ss<<"cap_depth_000";
//			if (ind<10){
//				ssc<<"00";
//				ss<<"00";
//			}
//			else {
//				if (ind<100){
//					ssc<<"0";
//					ss<<"0";
//				}
//			}
//			ssc<<ind;
//			ss<<ind;
//			ssc<<".png";
//			ss<<".png";
//			Mat winnerPic=imread(ssc.str());
//			Mat winnerPicd=imread(ss.str(),-1);
//			//detect the edges
//			ret_all r=f.call_main( winnerPic, winnerPicd);
//			//detect the most horizontal edge
//			//(find the edge with tthe minimum inclination)
//			double min_f=0.0;
//			int i_min=-1;
//			for (int i=0;i<r.edges_t.size();i++){
//				if (r.edges_t.at(i).at(0)!=-90){
//					double ar=r.edges_t.at(i).at(1)-r.edges_t.at(i).at(3);
//					double par=r.edges_t.at(i).at(0)-r.edges_t.at(i).at(2);
//					if (par!=0){
//						double f=abs(ar/par);
					
//						if (r.edges_t.at(i).at(0)<cx && r.edges_t.at(i).at(2)<cx){//if it does not start from the grasping point
//							if (min_f<f){
//								cout<<"f= "<<f;
//								min_f=f;
//								i_min=i;
//							}
//						}
//					}//if par
//				}//if at(0)!=-90
//			}//for
//			//detect the most horizontal subedge
//			int i_min2=0;
//			min_f=0;
//			for (int i=0;i<r.detailed_edges.at(2*i_min).size()-1;i++){
//				double ar=r.detailed_edges.at(2*i_min+1).at(i) - r.detailed_edges.at(2*i_min+1).at(i+1) ;
//				double par= r.detailed_edges.at(2*i_min).at(i) - r.detailed_edges.at(2*i_min).at(i+1) ;
//				if (par != 0){
//					double f= abs( ar/par );
//					if ( min_f < f ){
//						min_f=f;
//						i_min2=i;
//					}
//				}
//			}

//			cout<<"imin"<<i_min<<" f= "<<double(min_f);
//            grasp_candidate.at(0)=i_min;
//			grasp_candidate.at(1)=r.detailed_edges.at(2*i_min).at(i_min2);
//			grasp_candidate.at(2)=r.detailed_edges.at(2*i_min+1).at(i_min2);

			/////depict the edges
//			for (int i=0;i<r.detailed_edges.size();i=i+2){
		
//				if (r.detailed_edges.at(i).size()>0){
//					for (int j=0;j<r.detailed_edges.at(i).size()-1;j++){
//						colors rc=show_colors(i);
			
//						line(winnerPic, Point(r.detailed_edges.at(i).at(j),r.detailed_edges.at(i+1).at(j)), Point(r.detailed_edges.at(i).at(j+1),r.detailed_edges.at(i+1).at(j+1)), Scalar(rc.c0,rc.c1,rc.c2), 1, CV_AA);
//					}
//				}
//			}
			

//			for (int o1=-4;o1<5;o1++){
//				for(int o2=-4;o2<5;o2++){
//					for (int c=0;c<2;c++){
//						winnerPic.at<Vec3b>(grasp_candidate.at(2)+o1,grasp_candidate.at(1)+o2)[c]=0;
//					}
//					winnerPic.at<Vec3b>(grasp_candidate.at(2)+o1,grasp_candidate.at(1)+o2)[2]=255;
//				}
//			}

			
//			colors rc=show_colors(11);
//			line(winnerPic, Point(r.edges_t.at(i_min).at(0),r.edges_t.at(i_min).at(1)), Point(r.edges_t.at(i_min).at(2),r.edges_t.at(i_min).at(3)), Scalar(rc.c0,rc.c1,rc.c2), 4, CV_AA);
//			namedWindow("winner",0);
//			imshow("winner",winnerPic);
//			waitKey();
		

		
//		}

//	}
//	else{
//		cout<<"early stop";
//			int ind=grasp_candidate.at(0)*step+start_pict;
//			//depict the winner
//			stringstream ssc;
//			stringstream ss;
			
//            //ssc<<str;
//            //ss<<str;
//            ssc<<"cap_rgb_000";
//            ss<<"cap_depth_000";
//			if (ind<10){
//				ssc<<"00";
//				ss<<"00";
//			}
//			else {
//				if (ind<100){
//					ssc<<"0";
//					ss<<"0";
//				}
//			}
//			ssc<<ind;
//			ss<<ind;
//			ssc<<".png";
//			ss<<".png";
//			Mat winnerPic=imread(ssc.str());
//			Mat winnerPicd=imread(ss.str(),-1);
//			for (int o1=-4;o1<5;o1++){
//				for(int o2=-4;o2<5;o2++){
//					for (int c=0;c<2;c++){
//						winnerPic.at<Vec3b>(grasp_candidate.at(2)+o1,grasp_candidate.at(1)+o2)[c]=0;
//					}
//					winnerPic.at<Vec3b>(grasp_candidate.at(2)+o1,grasp_candidate.at(1)+o2)[2]=255;
//				}
//			}
//			namedWindow("winner",0);
//			imshow("winner",winnerPic);
//			waitKey();
//			cout<<"COORDINATES2 "<<grasp_candidate.at(1)<<" "<<grasp_candidate.at(2)<<endl;
			

//			//------------depict the edges------------------
//			ret_all r=f.call_main( winnerPic, winnerPicd);
			
//			for (int i=0;i<r.detailed_edges.size();i=i+2){
		
//				if (r.detailed_edges.at(i).size()>0 && r.detailed_edges.at(i).at(0)!=-90){
//					for (int j=0;j<r.detailed_edges.at(i).size()-1;j++){
//						colors rc=show_colors(i);
			
//						line(winnerPic, Point(r.detailed_edges.at(i).at(j),r.detailed_edges.at(i+1).at(j)), Point(r.detailed_edges.at(i).at(j+1),r.detailed_edges.at(i+1).at(j+1)), Scalar(rc.c0,rc.c1,rc.c2), 1, CV_AA);
//					}
//				}
//			}


//			namedWindow("winner",0);
//			imshow("winner",winnerPic);
//			waitKey();

    }

}
