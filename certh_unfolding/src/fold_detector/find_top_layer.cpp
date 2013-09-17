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

void find_top_layer(Mat im, Mat imd){
	
	//find the first pixel!=255
	int sti,stj;
	bool flag=false;
	for (int i=0;i<im.rows && flag==false;i++ ){
		for (int j=0;j<im.cols;j++){
			if (im.at<uchar>(i,j)!=255){
				sti=i;
				stj=j;
				flag=true;
				break;
			}
		}
	}
	while(flag==true){
	int count=0,sum=0,right_stopper=im.cols-1;
	bool stop=false;
	for(int i=sti;i<im.rows;i++){
		
		//to prwto pixel ths epomenhs seiras
		//an pesei panw se akmh
		if (im.at<uchar>(i,stj)==255){
		
			while(im.at<uchar>(i,stj)==255 && stj<im.cols-1 && (stj<right_stopper || right_stopper==im.cols-1)){
				stj++;
				
			}
		/*	if (stj==im.cols-1){
				break;
			}*/
			if (stj>=right_stopper && right_stopper!=im.cols-1){
				
				stop=true;
				break;
			}
		}
		else{
			
		//an prepei na brw apo poio pixel na ksekinhsw
			//phgainw pros ta pisw gia na brw grammh 
			int in_stj=stj;
			//elegxw mexri 3 pshfia pisw
			bool f=false;
			while (im.at<uchar>(i,stj)!=255 && stj>0 && in_stj-4<stj){
				stj--;
				if (im.at<uchar>(i,stj)==255){
					stj++;
					f=true;
					break;

				}
			}
			
			if (f==false){
				//stj=in_stj;
				stop=true;
			}
			if (im.at<uchar>(i,stj)!=255 && stj==0){
				stop=false;
			}
		}
		//an synexizoume me to idio layer
		
		if (stop==false){
			
			for (int j=stj;j<im.cols;j++){
				if (im.at<uchar>(i,j)!=255){
					count++;
					sum=sum+imd.at<uchar>(i,j);
					im.at<uchar>(i,j)=255;
					right_stopper=j;
				}
				else{
					break;
				}
			}
		}

	}


	/*namedWindow("im",0);
	imshow("im",im);
	namedWindow("imd",0);
	imshow("imd",imd);
	waitKey(0);*/
	////search for new layer
	flag=false;
	for (int i=0;i<im.rows && flag==false;i++ ){
		for (int j=0;j<im.cols;j++){
			if (im.at<uchar>(i,j)!=255){
				sti=i;
				stj=j;
				flag=true;
				break;
			}
		}
	}


	}
}