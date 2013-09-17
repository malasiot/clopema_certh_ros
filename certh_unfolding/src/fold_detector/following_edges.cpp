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

struct new_points{
	vector<int> pntx;
	vector<int> pnty;
};
struct ret_fol_edges{
	new_points points;
	bool found;
	Mat image;
    vector<vector<int> > npd;
    vector<vector<int> > table;
};

struct ret_edges_detailed{
    vector<vector<int> > d_edges;
    vector<vector<int> > table;
};

float normal_dist(Point,Point,Point);

new_points simplified_line(vector<int>,vector<int>,int,double);
ret_edges_detailed edges_in_detail(vector<int> ,vector<int> ,new_points,Mat);


ret_fol_edges following_edges(Mat edges){
	ret_fol_edges ret;
	Mat im;

	

	edges.copyTo(im);
	int ii,jj;
	vector<int> x_pixels(1000);
	vector<int> y_pixels(1000);
	int dim=0,count_times;
	bool start=true;
while (start==true && dim<3){//etsi wste an briskei 1-2 pixel na ksanapsaxnei
	dim=0;
	//bres to pshlotero shmeio
	start=false;
	
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

	/*//bres to pshlotero shmeio
	for (int i=-2;i<3;i++){
		for (int j=-2;j<3;j++){
			edges.at<uchar>(ii+i,jj+j)=255;
		}
	}
	imshow("finding edges",edges);*/
	

	if (start==true){//an exei brethei pixel
	
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
	while (count_times<3 && stop==false){
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
				//	cout<<" "<<j;
					
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
	
	/*imshow("finding edges",edges);
	waitKey(0);*/
	}//while stop ==true
	if (dim>2){
		int dim2;
	new_points jun=simplified_line(x_pixels,y_pixels, dim,10);
	dim2=jun.pntx.size();
	//std::cout<<"dim"<<dim;
	for (int i=0;i<dim2;i++){
		for (int k=-2;k<3;k++){
			for (int l=-2;l<3;l++){
				im.at<uchar>(jun.pnty.at(i)+k,jun.pntx.at(i)+l)=255;
			}
		}
	}
	Mat im2;
	edges.copyTo(im2);
	
	ret_edges_detailed d_ret=edges_in_detail(x_pixels,y_pixels,jun,im2);
    vector<vector<int> > npd=d_ret.d_edges;
	/*imshow("junctions",im);
	waitKey(0);*/
	
	ret.found=true;
	ret.points.pntx=jun.pntx;
	ret.points.pnty=jun.pnty;
	ret.image=edges;
	ret.npd=npd;
	ret.table=d_ret.table;
	
	}//if dim>2
	}//if start==true
}//while start== true && dim<3
	if (start==false){
	
		vector<int> zer(2,0);
		ret.found=false;
		ret.points.pntx=zer;
		ret.points.pnty=zer;
		ret.image=edges;
		ret.npd=ret.npd;
		vector<int> tems(2,0);
        vector<vector<int> > temb(2,tems);
		ret.table=temb;
	}
	return ret;
}
