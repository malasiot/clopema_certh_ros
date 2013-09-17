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

//this function computes the distance(d) of a point 'P'(x0,y0) from a line passing through
//two known points P1(x1,y1) and P2(x2,y2). it uses the formula
//d=asb((P2(1)-P1(1))*(P1(2)-P(2))-(P1(1)-P(1))*(P2(2)-P1(2)))/sqrt((P2(1)-P1(1))^2+(P2(2)-P1(2))^2)
//that is d=|(x2-x1)(y1-y0)-(x1-x0)(y2-y1)|/sqrt((x2-x1)^2+(y2-y1)^2).



float normal_dist(Point P1,Point P2,Point P){
	float dist,arith,paran;
	arith=(float)((P2.x-P1.x)*(P1.y-P.y)-(P1.x-P.x)*(P2.y-P1.y));
	paran=sqrt(pow((double)(P2.x-P1.x),2)+pow((double)(P2.y-P1.y),2));
	dist=abs(arith/paran);
	
	return dist;
}