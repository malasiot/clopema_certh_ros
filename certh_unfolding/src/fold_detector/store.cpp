/*#include<fstream>
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

struct new_points{
	vector<int> pntx;
	vector<int> pnty;
};

float normal_dist(Point,Point,Point);

new_points simplified_line(vector<int>,vector<int>,int);

void following_edges(Mat edges){
	Mat im;
	edges.copyTo(im);
	int ii,jj;
	vector<int> x_pixels(600);
	vector<int> y_pixels(600);
	int dim=0,count_times;
	//bres to pshlotero shmeio
	bool start=false;
	for (int i=5;i<edges.rows-5 && start==false;i++){
		for(int j=5 ;j<edges.cols-5;j++){
			if (edges.at<uchar>(i,j)==255){
				ii=i;
				jj=j;
				start=true;
				break;
			}
		}
	}
	
	for (int k=ii-1;k<ii+2;k++){
		for (int l=jj-1;l<jj+2;l++){
			edges.at<uchar>(k,l)=0;
		}
	}
	x_pixels.at(0)=jj;
	y_pixels.at(0)=ii;
	bool stop=true;
	
	while(stop==true){//oso briskei epomeno pixel
		
	count_times=1;
	stop=false;
	//follow edge
	while (count_times<4 && stop==false){
		count_times++;
		for (int i=ii-count_times;i<ii+count_times+1 && stop==false;i++){
			for (int j=jj-count_times;j<jj+count_times+1;j++){
				if (edges.at<uchar>(i,j)==255){
					//krataw syntatagmenes
					dim++;
					x_pixels.at(dim)=j;
					y_pixels.at(dim)=i;
					ii=i;
					jj=j;
					cout<<" "<<j;
					//diagrafw ta pixel sthn perioxh gia na mhn epistrepsei se ayta
					for (int k=ii-1;k<ii+2;k++){
						for (int l=jj-1;l<jj+2;l++){
							edges.at<uchar>(k,l)=0;
						}
					}
			
					stop=true;
					break;
				}
			}
		}
	}//while
	imshow("finding edges",edges);
	waitKey(0);
	}//while stop ==true
	new_points jun=simplified_line(x_pixels,y_pixels, dim);
	dim=jun.pntx.size();
	cout<<"dim"<<dim;
	for (int i=0;i<dim;i++){
		for (int k=-2;k<3;k++){
			for (int l=-2;l<3;l++){
				im.at<uchar>(jun.pnty.at(i)+k,jun.pntx.at(i)+l)=255;
			}
		}
	}
	imshow("junctions",im);
	waitKey(0);
}*/