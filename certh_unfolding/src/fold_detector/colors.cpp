#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>

#include<fstream>
#include <stdlib.h>
#include <stdio.h>
#include <opencv2/core/core.hpp>
#include <vector>
#include <math.h>
using namespace cv;
using namespace std;

struct colors{
	
	int c0;
	int c1;
	int c2;

};

colors show_colors(int count_folds){
	int c0,c1,c2,n=9;
	colors ret_color;
	if(count_folds%n==0){
		ret_color.c0=100;
		ret_color.c1=0;
		ret_color.c2=0;
	}
	if(count_folds%n==1){
		ret_color.c0=0;
		ret_color.c1=100;
		ret_color.c2=0;
	}
	if(count_folds%n==2){
		ret_color.c0=0;
		ret_color.c1=0;
		ret_color.c2=100;
	}
	if(count_folds%n==3){
		ret_color.c0=200;
		ret_color.c1=100;
		ret_color.c2=0;
	}
	if(count_folds%n==4){
		ret_color.c0=0;
		ret_color.c1=200;
		ret_color.c2=100;
	}
	if(count_folds%n==5){
		ret_color.c0=100;
		ret_color.c1=0;
		ret_color.c2=200;
	}
	if(count_folds%n==6){
		ret_color.c0=200;
		ret_color.c1=100;
		ret_color.c2=100;
	}
	if(count_folds%n==7){
		ret_color.c0=100;
		ret_color.c1=200;
		ret_color.c2=100;
	}
	if(count_folds%n==8){
		ret_color.c0=100;
		ret_color.c1=100;
		ret_color.c2=200;
	}
	return ret_color;
}