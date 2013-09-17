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

int find_layer(vector<int> edges_clockwise,vector<vector<int> > edges_t,Mat iml1,int ind1,int ind2,int c_u,vector<int> jun){
	//edges_clockwise:oi akmes gyrw apo to junction topothethmenes me th fora tou rologiou
	//edges_t:oloklhres oi akmes, ind1,ind2:o arithmos pou antistoixei sthn 1h kai 2h akmh pou eksetazoume
	//c_u: poses akmes briskontai panw apo to junction

	//bres ta a1 kai b1 ths prwths akmhs
	int par=edges_t.at(edges_clockwise.at(ind1)).at(0)-edges_t.at(edges_clockwise.at(ind1)).at(2);
	int ar=edges_t.at(edges_clockwise.at(ind1)).at(1)-edges_t.at(edges_clockwise.at(ind1)).at(3);
	float a1,b1;
	if (par!=0){
			a1=(float)ar/par;
			b1=(float)(-a1*edges_t.at(edges_clockwise.at(ind1)).at(0)+edges_t.at(edges_clockwise.at(ind1)).at(1));
			
		}
	

	//bres ta a2 b2 ths 2hs akmhs
	par=edges_t.at(edges_clockwise.at(ind2)).at(0)-edges_t.at(edges_clockwise.at(ind2)).at(2);
	ar=edges_t.at(edges_clockwise.at(ind2)).at(1)-edges_t.at(edges_clockwise.at(ind2)).at(3);
	float a2,b2;
	
	if (par!=0){
			a2=(float)ar/par;
			b2=(float)(-a2*edges_t.at(edges_clockwise.at(ind2)).at(0)+edges_t.at(edges_clockwise.at(ind2)).at(1));
			
		}
	
	//oria gia thn 1h akmh 
	int y11,y12;
	y11=edges_t.at(edges_clockwise.at(ind1)).at(0);
	y12=edges_t.at(edges_clockwise.at(ind1)).at(2);
	if (edges_t.at(edges_clockwise.at(ind1)).at(0)>edges_t.at(edges_clockwise.at(ind1)).at(2)){
		int t1=y11;
		y11=y12;
		y12=t1;
	}

//oria gia thn 2h akmh 
	int y21,y22;
	y21=edges_t.at(edges_clockwise.at(ind2)).at(0);
	y22=edges_t.at(edges_clockwise.at(ind2)).at(2);
	if (edges_t.at(edges_clockwise.at(ind2)).at(0)>edges_t.at(edges_clockwise.at(ind2)).at(2)){
		int t1=y21;
		y21=y22;
		y22=t1;
	}

	////sxhmatizw mia mikrh eikona gyrv apo to junction
	//Mat ims=Mat::zeros(21,21,CV_8UC1);
	//for (int k=-10;k<11;k++){
	//		for (int k2=-10;k2<11;k2++){
	//			ims.at<uchar>(k+10,k2+10)=iml1.at<uchar>(jun.at(1)+k,jun.at(0)+k2);	
	//			
	//		}			
	//	}
	//	for (int k=-2;k<3;k++){
	//		for (int k2=-2;k2<3;k2++){
	//			ims.at<uchar>(jun.at(1)+k,jun.at(0)+k2)=255;	
	//		}			
	//	}



	//briskw to hmiepipedo pou sxhmatizetai apo thn 1h akmh
	// kai mas afora

	//bres ta oria gia thn 1h grammh
		int minyj,maxyj;
		minyj=jun.at(0)-10;
		maxyj=jun.at(0)+10;
		//cout<<"minyj= "<<minyj<<" maxyj= "<<maxyj<<" y11= "<<y11<<" y12= "<<y12;
		if (y11<minyj){
			y11=minyj;
		}
		if (y12>maxyj){
			y12=maxyj;
		}

		//for (int k=-2;k<3;k++){
		//	for (int k2=-2;k2<3;k2++){
		//		iml1.at<uchar>(jun.at(1)+k,jun.at(0)+k2)=255;	
		//		iml1.at<uchar>(edges_t.at(edges_clockwise.at(ind2)).at(1)+k,edges_t.at(edges_clockwise.at(ind2)).at(0)+k2)=255;
		//		iml1.at<uchar>(edges_t.at(edges_clockwise.at(ind2)).at(1)+k,edges_t.at(edges_clockwise.at(ind2)).at(0)+10+k2)=255;
		//	}			
		//}

		//cout<<" final y11 y22 "<<y11<<"  "<<y12;
	//dokimh
	for (int k=y11;k<y12;k++){
		for (int k1=-3;k1<4;k1++){
			for (int k2=-3;k2<4;k2++){
				
			//	iml1.at<uchar>((int)(a1*k+b1+k2),k+k1)=255;
				iml1.at<uchar>((int)(a1*k+b1),k)=255;
			}
		}
	}
		int bg1,f1,s1;
		for(int k=y11;k<y12;k++){
			
			//an h grammh einai panw apo to junction
			if (ind1<c_u){
				 bg1=(int)(a1*k+b1+0.5);
				 f1=jun.at(1)+10;
			}
			else{
				bg1=jun.at(1)-10;
				f1=(int)(a1*k+b1+0.5);
			}
			for (int l=bg1;l<f1;l++){
				if (l<jun.at(1)+10 && l>jun.at(1)-10){
					iml1.at<uchar>(l,k)=255;
				}
			}
		}

	//	
	////bres ta oria gia th 2h akmh
	//	minyj,maxyj;
	//	minyj=jun.at(0)-10;
	//	maxyj=jun.at(0)+10;
	//
	//	if (y21<minyj){
	//		y21=minyj;
	//	}
	//	if (y22>maxyj){
	//		y22=maxyj;
	//	}
	//dokimh

	/*for (int k=y21;k<y22;k++){
		for (int k1=-3;k1<4;k1++){
			for (int k2=-3;k2<4;k2++){
				iml1.at<uchar>((int)(a2*k+b2+k2),k+k1)=255;
			}
		}
	}
	*/

	return 0;
}