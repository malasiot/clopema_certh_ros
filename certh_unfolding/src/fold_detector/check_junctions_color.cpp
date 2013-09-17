#include<fstream>
#include <stdlib.h>
#include <stdio.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <vector>
#include <math.h>


using namespace cv;
using namespace std;

void junctions_check_colors(vector<vector<int> > junctions,Mat im_d){
	/*Mat im=Mat::zeros(21,21,CV_8UC1);*/
	Mat im(21,21,CV_8UC3,Scalar(255,255,255));
	Mat imd,edges;

	im_d.copyTo(imd);
	
	for (int i=0;i<junctions.size();i++){
		
		for (int k=-2;k<3;k++){
			for (int k2=-2;k2<3;k2++){
				for (int c=0;c<3;c++){
					
					imd.at<Vec3b>(junctions.at(i).at(1)+k,junctions.at(i).at(0)+k2)[c]=0;
				}
			}
						
		}
		
		for (int k=-10;k<11;k++){
			for (int k2=-10;k2<11;k2++){
				for(int c=0;c<3;c++){
					im.at<Vec3b>(k+10,k2+10)[c]=imd.at<Vec3b>(junctions.at(i).at(1)+k,junctions.at(i).at(0)+k2)[c];	
				}
			}
						
		}
	/*	namedWindow("junctions_roi",0);
	imshow("junctions_roi",im);
	namedWindow("junction",0);
	imshow("junction",imd);*/
	/*Canny(im, edges, 60.0, 180.0, 3);
	namedWindow("canny_jun",0);
	imshow("canny_jun",edges);*/
//	waitKey(0);
	im_d.copyTo(imd);
	}	
	
}
