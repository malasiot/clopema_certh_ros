#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "FoldDetector.h"
#include <iostream>

using namespace cv;
using namespace std;
typedef uint16_t char16_t;
//
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

struct true_pix{
    vector<vector<int> > junctions;
    vector<vector<int> > detailed_edges;
    vector<vector<int> > edges_t;
};


ret_all canny_image(int ,int ,int,int,Mat,Mat);
true_pix true_pixels(vector<vector<int> > ,vector<vector<int> > ,vector<vector<int> > ,int ,int, int );
colors show_colors(int);


ret_all folds :: call_main( Mat bgrImage, Mat depthMap )
{
		
		//change the orientation of the images
		

       /* Mat depthMap;
        Mat bgrImage;*/
        Mat grayImage;
		Mat gray_rgb;
		Mat gray;
		

		/*bgrImage=imread("imc0.png");
		depthMap=imread("im0.png",-1);*/
		
		int maxx=0,minx=60000,maxy=0,miny=6000;

		cvtColor(bgrImage,gray,CV_BGR2GRAY);
		//ftiakse eikona me tis diastaseis ths gray
		Mat bin=cv::Mat::zeros(gray.rows,gray.cols,CV_8UC1);
		Mat bin2=cv::Mat::zeros(gray.rows,gray.cols,CV_8UC1);
		//ftiakse mia eikona pou na deixnei gri alla na na einai rgb
		cvtColor(gray,gray_rgb,CV_GRAY2BGR);
						
		for (int k=0;k<depthMap.rows;k++){
			for (int l=0;l<depthMap.cols;l++){
				/////////////////////////////////////////////////////////////////////////////
				//gia na mh xtyphsei etsi kai uparxei antikeimeno sthn akrh
				if (k<12 || k>depthMap.rows-12 || l<12 || l>depthMap.cols-12){
					depthMap.at<char16_t>(k,l)=0;
				}
				//////////////////////////////////////////////////////////////////////////////
                if (depthMap.at<char16_t>(k,l)<1400 && depthMap.at<char16_t>(k,l)>900){
					//zwgrafizei thn perioxh se mia rgb eikona
					for (int c=0;c<2;c++){
						gray_rgb.at<Vec3b>(k,l)[c]=0;
					}
					gray_rgb.at<Vec3b>(k,l)[2]=255;
					bin.at<uchar>(k,l)=255;
				}
			}
		}
		//find contours of the areas in the limits
		vector< vector<Point> > contours;
		findContours(bin, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
		//an brethhke contour
		if(!contours.empty()){
			vector<double> areas(contours.size());
			for(int i = 0; i <(int) contours.size(); i++){
				areas[i] = contourArea(Mat(contours[i]));
			}
			double max;
			Point maxPosition;
			minMaxLoc(Mat(areas),0,&max,0,&maxPosition);
			//briskei poia einai h megalyterh epifaneia
			int c_indx=maxPosition.y;
											
			//krata ta shmeia ths epifaneias
			vector<Point> cntrs=contours[c_indx];
			//briskw megistes kai elaxistes diastaseis
											
			for (int g=0;g<cntrs.size();g++){
				Point P=cntrs.at(g);
				if(minx>P.x){
					minx=P.x;
				}
				if(maxx<P.x){
					maxx=P.x;
				}
				if (miny>P.y){
					miny=P.y;
				}
				if (maxy<P.y){
					maxy=P.y;
				}

			}
			maxx=maxx+10;
			maxy=maxy+10;
			minx=minx-10;
			miny=miny-10;

			line( gray_rgb,Point(minx, miny), Point(minx, maxy), Scalar(0,255,0), 3, CV_AA);
			line( gray_rgb,Point(maxx, miny), Point(maxx, maxy), Scalar(0,255,0), 3, CV_AA);
			line( gray_rgb,Point(minx, miny), Point(maxx, miny), Scalar(0,255,0), 3, CV_AA);
			line( gray_rgb,Point(minx, maxy), Point(maxx, maxy), Scalar(0,255,0), 3, CV_AA);
		}

	
					
	ret_all r=canny_image(maxx, minx,maxy,miny,bgrImage,depthMap);
	
	true_pix tr_p=true_pixels(r.junctions, r.detailed_edges, r.edges_t,minx,miny,maxx-minx);
	ret_all r_all;
	r_all.a_corn=r.a_corn;
	r_all.detailed_edges=tr_p.detailed_edges;
	r_all.disconnected=r.disconnected;
	r_all.edges_of_junct=r.edges_of_junct;
	r_all.edges_t=tr_p.edges_t;
	r_all.junctions=tr_p.junctions;
	r_all.table=r.table;
	
	Mat DisplayIm;
	bgrImage.copyTo(DisplayIm);
	Mat Display_a_c;
	bgrImage.copyTo(Display_a_c);
	//display junctions
	///*for (int i=0;i<r_all.junctions.size();i++){
	//	for (int o1=-3;o1<4;o1++){
	//		for (int o2=-3;o2<4;o2++){
	//			for (int c=0;c<3;c++){
	//				DisplayIm.at<Vec3b>(r_all.junctions.at(i).at(1)+o1,r_all.junctions.at(i).at(0)+o2)[c]=255;
	//			}
	//		}
	//	}
	//}*/
	
	
	/*for (int i=0;i<r_all.detailed_edges.size();i=i+2){
		
		if (r_all.detailed_edges.at(i).size()>0){
		for (int j=0;j<r_all.detailed_edges.at(i).size()-1;j++){
			colors rc=show_colors(i);
			
			line(Display_a_c, Point(r_all.detailed_edges.at(i).at(j),r_all.detailed_edges.at(i+1).at(j)), Point(r_all.detailed_edges.at(i).at(j+1),r_all.detailed_edges.at(i+1).at(j+1)), Scalar(rc.c0,rc.c1,rc.c2), 1, CV_AA);
		}
		}
	}*/

	/*for (int i=0;i<r_all.edges_t.size();i++){
		if (r_all.edges_t.at(i).at(0)!=-90){
			colors rc=show_colors(i);
			line(Display_a_c, Point(r_all.edges_t.at(i).at(0),r_all.edges_t.at(i).at(1)), Point(r_all.edges_t.at(i).at(2),r_all.edges_t.at(i).at(3)), Scalar(rc.c0,rc.c1,rc.c2), 1, CV_AA);
		}
	}

	namedWindow("edges_t",0);
	imshow("edges_t",Display_a_c);*/
	
	///*namedWindow("junctions",0);
	//imshow("junctions",DisplayIm);*/
	
	//display a-corners
	//int jun;
	//if (r_all.a_corn.certain_c.at(0)!=-1){
	//	for (int i=0;i<r_all.a_corn.certain_c.size();i++){
	//		jun=r_all.a_corn.certain_c.at(i);
	//		for (int o1=-4;o1<5;o1++){
	//			for (int o2=-4;o2<5;o2++){
	//				for (int c=0;c<2;c++){
	//					Display_a_c.at<Vec3b>(r_all.junctions.at(jun).at(1)+o1,r_all.junctions.at(jun).at(0)+o2)[c]=0;
	//				}
	//				Display_a_c.at<Vec3b>(r_all.junctions.at(jun).at(1)+o1,r_all.junctions.at(jun).at(0)+o2)[2]=255;
	//			}
	//		}
	//	}
	//}
	//
	//if (r_all.a_corn.distant_c.at(0)!=-1){
	//	for (int i=0;i<r_all.a_corn.distant_c.size();i++){
	//		jun=r_all.a_corn.distant_c.at(i);
	//		for (int o1=-4;o1<5;o1++){
	//			for (int o2=-4;o2<5;o2++){
	//				Display_a_c.at<Vec3b>(r_all.junctions.at(jun).at(1)+o1,r_all.junctions.at(jun).at(0)+o2)[0]=255;
	//				Display_a_c.at<Vec3b>(r_all.junctions.at(jun).at(1)+o1,r_all.junctions.at(jun).at(0)+o2)[1]=255;
	//				Display_a_c.at<Vec3b>(r_all.junctions.at(jun).at(1)+o1,r_all.junctions.at(jun).at(0)+o2)[2]=0;
	//			}
	//		}
	//	}
	//}
	//
	//if (r_all.a_corn.str_l_c.at(0)!=-1){
	//	for (int i=0;i<r_all.a_corn.str_l_c.size();i++){
	//		jun=r_all.a_corn.str_l_c.at(i);
	//		for (int o1=-4;o1<5;o1++){
	//			for (int o2=-4;o2<5;o2++){
	//				for (int c=1;c<3;c++){
	//					Display_a_c.at<Vec3b>(r_all.junctions.at(jun).at(1)+o1,r_all.junctions.at(jun).at(0)+o2)[c]=0;
	//				}
	//				Display_a_c.at<Vec3b>(r_all.junctions.at(jun).at(1)+o1,r_all.junctions.at(jun).at(0)+o2)[0]=255;
	//			}
	//		}
	//	}
	//}

	////cout<< "r_all.a_corn.certain_c.at(0)"<<r_all.a_corn.certain_c.at(0)<<" & "<<r_all.a_corn.distant_c.at(0)<<endl;
	////cout<<" r_all.a_corn.str_l_c.at(0) "<<r_all.a_corn.str_l_c.at(0);
	//namedWindow("folds",0);
	//imshow("folds",Display_a_c);

	//waitKey(0);

    return r_all;
}
