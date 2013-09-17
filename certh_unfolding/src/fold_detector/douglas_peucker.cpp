#include<fstream>
#include <stdlib.h>
#include <stdio.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <vector>
#include <math.h>
#include <list>
#include <stack>


using namespace cv;
using namespace std;


struct new_points{
	vector<int> pntx;
	vector<int> pnty;
};

float normal_dist(Point,Point,Point);




new_points simplified_line(vector<int> x_pixels,vector<int> y_pixels,int dim,double tolerance){
	new_points np;

	//////Douglas_Peucker///////////
	//double tolerance=10;
	list<int> list_pts;
	stack<int> s;
	int indxfirst=0;   //index of first point
	//vector<int> first(2);
	//vector<int> last(2);
	int indxlast;
	Point first(x_pixels.at(0),y_pixels.at(0));//coordinates of first point
	
	indxlast=dim-1;//index of last point
	Point last(x_pixels.at(indxlast),y_pixels.at(indxlast));    //coordinates of last point
	
	s.push(indxlast);
	list_pts.push_back(indxfirst);

	//cout<<dim<<endl;
	//cout<<s.top()<<endl<<list_pts.front();
	int i,indxdist;
	float dmax,d;
	while(!s.empty()){//terminate the iterations when stack is empty
		dmax=0;//initialise maximum distance as zero
		i=indxfirst+1; //start iterations from the next point of first index
		while(i<indxlast){//this loop computes shortest distance of a point from a line segment
			Point P(x_pixels.at(i),y_pixels.at(i)); //coordinates of point with index 'i'
			d=normal_dist(first,last,P);//and compares it with the maximum distance value in previous iterations
			if (d>dmax){
				dmax=d;
				indxdist=i;
			}//if
			i++;
		}//while(i<indxlast)
	     // compare the maximum distance with tolerance and then the index is either pushed onto stack or list of output points
		if (dmax<=tolerance){
			list_pts.push_back(indxlast);
			indxfirst=s.top();
			s.pop();
			first.x=x_pixels.at(indxfirst);
			first.y=y_pixels.at(indxfirst);
			if (!s.empty()){
				indxlast=s.top();
				last.x=x_pixels.at(indxlast);
				last.y=y_pixels.at(indxlast);

			}//if (!s.empty())
		}//if (dmax<=tolerance)
		else{
			indxlast=indxdist;
			last.x=x_pixels.at(indxlast);
			last.y=y_pixels.at(indxlast);
			s.push(indxdist);
		}

		
		
	}//while(!s.empty())

	//assign the coordinates of the points from the list of indices of point_list to output points.

	int slist=list_pts.size();
	vector<int> pntsx(slist);
	vector<int> pntsy(slist);
//	cout<<"slist"<<slist<<" k "<<list_pts.back()<<endl;

	for (int j=0;j<slist;j++){
		pntsx.at(j)=x_pixels.at(list_pts.front());
		pntsy.at(j)=y_pixels.at(list_pts.front());
		list_pts.pop_front();
	//	cout<<pntsx.at(j)<<" k "<<pntsy.at(j)<<endl;
	}
	np.pntx=pntsx;
	np.pnty=pntsy;
	
return np;
}