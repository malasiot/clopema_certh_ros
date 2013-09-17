#include<fstream>
#include <stdlib.h>
#include <stdio.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <vector>
#include <math.h>
//briskei tis akmes me leptomeries

using namespace cv;
using namespace std;

struct new_points{
	vector<int> pntx;
	vector<int> pnty;
};
struct colors{
	
	int c0;
	int c1;
	int c2;

};

struct ret_edges_detailed{
    vector<vector<int> > d_edges;
    vector<vector<int> > table;
};

colors show_colors(int );
new_points simplified_line(vector<int>,vector<int>,int,double);




ret_edges_detailed edges_in_detail(vector<int> x_pixels,vector<int> y_pixels,new_points np,Mat im)
{
	vector<int> tem1(im.cols,-2);
    vector<vector<int> > table(im.rows,tem1);

	int ned=np.pntx.size()-1;//o arithmos tvn akmwn einai ned-1
    vector<vector<int> > vx(ned);// analytika oi akmes
    vector<vector<int> > vy(ned);// analytika oi akmes
	int k=0;//deixnei se poia akmh eimaste
	vector<int> tem(300,0),temx(300,0),temy(300,0);
	//cout<<"ned= "<<ned<<endl;
	int count=0;
	temx.at(count)=x_pixels.at(0);
	temy.at(count)=y_pixels.at(0);

	for (int i=1;i<x_pixels.size();i++){
		/////
		for (int i1=-1;i1<2;i1++){
			for (int i2=-1;i2<2;i2++){
				table.at(y_pixels.at(i)+i1).at(x_pixels.at(i)+i2)=k;
			}
		}
////
		count++;
		temx.at(count)=x_pixels.at(i);
		temy.at(count)=y_pixels.at(i);
		//cout<<"count "<<count<<endl;
		if ((x_pixels.at(i)==np.pntx.at(k+1)) && (y_pixels.at(i)==np.pnty.at(k+1)) ){
			//cout<<"k= "<<k;
			vx.at(k)=temx;
			vy.at(k)=temy;
			if (k<ned-1){
				
				count=0;
				k++;
				temx=tem;
				temy=tem;
				temx.at(count)=x_pixels.at(i);
				temy.at(count)=y_pixels.at(i);
				
			}
			else{
				break;
			}

		}
	}

	//Mat depict(im.rows,im.cols, CV_8UC3, Scalar(255,255,255));
	////depict the edge
	//int c;
	//colors color;
	//	for (int j=0;j<ned;j++){
	//		c=0;
	//		color=show_colors(j);
	//		while(vx.at(j).at(c)!=0){
	//			cout<<"vx.at(j).at(c) "<<vx.at(j).at(c)<<endl;
	//			depict.at<Vec3b>(vy.at(j).at(c),vx.at(j).at(c))[0]=color.c0;
	//			depict.at<Vec3b>(vy.at(j).at(c),vx.at(j).at(c))[1]=color.c1;
	//			depict.at<Vec3b>(vy.at(j).at(c),vx.at(j).at(c))[2]=color.c2;
	//			c++;
	////			cout<<"c= "<<c;
	//		}
	//		
	//	}
	//	imshow("depict",depict);


	//depict with douglas-peucker
	Mat depict(im.rows,im.cols, CV_8UC3, Scalar(255,255,255));
		int c1;
		new_points nps;
		colors r;
		vector<vector<int> > d_edges(2*vx.size());//oi akmes me leptomeries
		d_edges.at(0)=tem;//gia tis diastaseis
		
		for (int j=0;j<vx.size();j++){
			c1=0;
			r=show_colors(j);
			while(vx.at(j).at(c1)!=0){
				c1++;
			}
		
			nps=simplified_line(vx.at(j),vy.at(j),c1,2);
		
			for (int k=0;k<nps.pntx.size()-1;k++){
				line(depict, Point(nps.pntx.at(k),nps.pnty.at(k)), Point(nps.pntx.at(k+1),nps.pnty.at(k+1)), Scalar(r.c0,r.c1,r.c2), 1, CV_AA);
			}
			
			//apothhkeyse tis akmes me leptomeries
			temx=tem;
			temy=tem;
			for (int k=0;k<nps.pntx.size();k++){
				
				temx.at(k)=nps.pntx.at(k);
				temy.at(k)=nps.pnty.at(k);
			}
			d_edges.at(2*j)=temx;	
			d_edges.at(2*j+1)=temy;
			
		}
		//imshow("depict",depict);
		/*for (int i1=0;i1<table.size();i1++){
			for (int i2=0;i2<table.at(0).size();i2++){
				cout<<" "<<table.at(i1).at(i2);
			}
			cout<<endl;
		}*/
		ret_edges_detailed ret;
		ret.d_edges=d_edges;
		ret.table=table;
		return ret;
}
