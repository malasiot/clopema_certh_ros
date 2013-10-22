#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>

using namespace cv;
using namespace std;

int garmentsLengthLimit(int cx, Mat depthMap,Mat bgrImage)
{

			int limUp=1400,limDown=	900;	
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
				if (depthMap.at<char16_t>(k,l)<limUp && depthMap.at<char16_t>(k,l)>limDown){
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
			

			//line( gray_rgb,Point(minx, miny), Point(minx, maxy), Scalar(0,255,0), 3, CV_AA);
			
		}



		//line( gray_rgb,Point(cx, miny), Point(cx, maxy), Scalar(0,255,0), 3, CV_AA);


		int dif=int(2*(cx-minx)/3);
		int lowl=minx+dif;
		/*line( gray_rgb,Point(lowl, miny), Point(lowl, maxy), Scalar(0,255,0), 3, CV_AA);
		namedWindow("oria",0);
		imshow("oria",gray_rgb);
		waitKey();*/
		
		return lowl;


}