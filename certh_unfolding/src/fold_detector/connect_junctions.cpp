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

struct colors{
	
	int c0;
	int c1;
	int c2;

};


colors show_colors(int);

void connect_junctions(vector<vector<int> > junctions,vector<vector<int> > detailed_edges,vector<int> disconnected,vector<vector<int> > edges_of_junct,Mat im){
	int s,count;
	//ftiaxnw eikona me tis akmes me xrwmata
	Mat im3=Mat::ones(im.rows,im.cols,CV_8UC1)*255;
	Mat ed_im;
	cvtColor(im3,ed_im,CV_GRAY2BGR);
	int c;
	for (int i=0;i<detailed_edges.size()/2;i++){
		c=0;
		colors r=show_colors(i);
		while (detailed_edges.at(2*i).at(c+1)!=0){
			line(ed_im, Point(detailed_edges.at(2*i).at(c),detailed_edges.at(2*i+1).at(c)), Point(detailed_edges.at(2*i).at(c+1),detailed_edges.at(2*i+1).at(c+1)), Scalar(r.c0,r.c1,r.c2), 1 , CV_AA);
			c++;
		}
	}
//	imshow("imcol",ed_im);
	///
	Mat ims=Mat::zeros(21,21,CV_8UC1);
	
	Mat tem;
	im.copyTo(tem);
	//briskw ta junction pou einai disconnected
	for (int i=0;i<disconnected.size();i++){
		//sbhnw thn pleyra me thn opoia syndeetai
		s=edges_of_junct.at(disconnected.at(i)).at(0);//h pleyra
		count=0;
		while (detailed_edges.at(2*s).at(count)!=0){
			line(im, Point(detailed_edges.at(2*s).at(count),detailed_edges.at(2*s+1).at(count)), Point(detailed_edges.at(2*s).at(count+1),detailed_edges.at(2*s+1).at(count+1)), 0, 3, CV_AA);
			count++;
		}
		//zwgrafise to junction
		for (int k=-1;k<2;k++){
			for (int l=-1;l<2;l++){
				im.at<uchar>(junctions.at(disconnected.at(i)).at(1)+k,junctions.at(disconnected.at(i)).at(0)+l)=255;
			}
		}
		for (int k=-10;k<11;k++){
			for (int k2=-10;k2<11;k2++){
				ims.at<uchar>(k+10,k2+10)=im.at<uchar>(junctions.at(disconnected.at(i)).at(1)+k,junctions.at(disconnected.at(i)).at(0)+k2);	
			}
						
		}
	/*	namedWindow("im",0);
		imshow("im",ims);*/
		for (int k=-10;k<11;k++){
			for (int k2=-10;k2<11;k2++){
				ims.at<uchar>(k+10,k2+10)=tem.at<uchar>(junctions.at(disconnected.at(i)).at(1)+k,junctions.at(disconnected.at(i)).at(0)+k2);	
			}
						
		}
	/*	namedWindow("im2",0);
		imshow("im2",ims);*/
		
		tem.copyTo(im);
		//waitKey(0);
	}
}
