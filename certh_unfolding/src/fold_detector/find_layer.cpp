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


int calc_closest_edge_side(vector<int>,vector<int> );

int find_layer(vector<int> edges_clockwise,vector<vector<int> > edges_t,Mat iml1,int ind1,int ind2,int c_u,vector<int> jun, vector<vector<int> > d_edges){
	//edges_clockwise:oi akmes gyrw apo to junction topothethmenes me th fora tou rologiou
	//edges_t:oloklhres oi akmes, ind1,ind2:o arithmos pou antistoixei sthn 1h kai 2h akmh pou eksetazoume
	//c_u: poses akmes briskontai panw apo to junction,d_edges: edges in detail

	//briskw poia pleyra ths 1hs akmhs einai konta sto junction
	int cl1=calc_closest_edge_side(edges_t.at(ind1),jun);
	int st1,f1,step1,orio=10;
	if (cl1==0){
		
		st1=0;
		int c=0;
		
		while (d_edges.at(2*edges_clockwise.at(ind1)).at(c)!=0 && d_edges.at(2*edges_clockwise.at(ind1)).size()>c){
			c++;
			//check if it is not in the area you want
			if (d_edges.at(2*edges_clockwise.at(ind1)).at(c)>=jun.at(0)+orio || d_edges.at(2*edges_clockwise.at(ind1)).at(c)<=jun.at(0)-orio){
				if (d_edges.at(2*edges_clockwise.at(ind1)+1).at(c)>=jun.at(1)+orio || d_edges.at(2*edges_clockwise.at(ind1)+1).at(c)-10<=jun.at(1)-orio){ 
					break;
				}
			}
		}
		
		f1=c;
		//step1=1;
	}
	else{
		//cout<<"case2";
		int c=0;
		while (d_edges.at(2*edges_clockwise.at(ind1)).at(c)!=0 && d_edges.at(2*edges_clockwise.at(ind1)).size()>c){
			c++;
			//check if it has reached the area you want
			if (d_edges.at(2*edges_clockwise.at(ind1)).at(c)<=jun.at(0)+orio || d_edges.at(2*edges_clockwise.at(ind1)).at(c)>=jun.at(0)-orio){
				if (d_edges.at(2*edges_clockwise.at(ind1)+1).at(c)<=jun.at(1)+orio || d_edges.at(2*edges_clockwise.at(ind1)+1).at(c)-10>=jun.at(1)-orio){ 
					break;
					c--;
				}
			}
		}
		int c1=0;
		while (d_edges.at(2*edges_clockwise.at(ind1)).at(c1)!=0 &&  d_edges.at(2*edges_clockwise.at(ind1)).size()>c){
			c1++;
		}

		st1=c;
		f1=c1--;
		
		
	}
	
	//gia kathe upoakmh
	for (int k=st1;k<f1-1;k++){
		for (int o1=-3;o1<4;o1++){
			for (int o2=-3;o2<4;o2++){

				iml1.at<uchar>(d_edges.at(2*edges_clockwise.at(ind1)+1).at(k)+o1,d_edges.at(2*edges_clockwise.at(ind1)).at(k)+o2)=255;
				iml1.at<uchar>(d_edges.at(2*edges_clockwise.at(ind1)+1).at(k+1)+o1,d_edges.at(2*edges_clockwise.at(ind1)).at(k+1)+o2)=255;
			}
		}
		/*namedWindow("iiml1",0);
		imshow("iiml1",iml1);
		waitKey(0);*/

		//bres thn eksiswsh ths eytheias
		int par=d_edges.at(2*edges_clockwise.at(ind1)).at(k)-d_edges.at(2*edges_clockwise.at(ind1)).at(k+1);
		int ar=d_edges.at(2*edges_clockwise.at(ind1)+1).at(k)-d_edges.at(2*edges_clockwise.at(ind1)+1).at(k+1);
		float a1,b1;
		if (par!=0){
			a1=(float)ar/par;
			b1=(float)(-a1*d_edges.at(2*edges_clockwise.at(ind1)).at(k)+d_edges.at(2*edges_clockwise.at(ind1)+1).at(k));
		}
		
		//cout <<endl<<" eutheia x "<< d_edges.at(2*edges_clockwise.at(ind1)).at(k+1)<<" y "<<d_edges.at(2*edges_clockwise.at(ind1)+1).at(k+1)<<endl;
		//cout<<" eutheia2 y "<<a1*d_edges.at(2*edges_clockwise.at(ind1)).at(k+1)+b1;
	//	cout<<" edw "; 

		//skanarw ta pixel sto parathyro pou me endiaferei
		for (int i=jun.at(0)-orio;i<jun.at(0)+orio;i++){
			
			for (int j=jun.at(1)-orio;j<jun.at(1)+orio;j++){
				
				//an h akmh einai panw apo to junction
				if (par!=0){
			
				//	cout<< " y "<<(i-b1)/a1;
					if ( ind1<c_u && (j-b1)/a1<i && (j-b1)/a1>0){//ind1<c_u
						iml1.at<uchar>(j,i)=255;
					}
					else{
						if (ind1>=c_u && (j-b1)/a1>i && (j-b1)/a1<jun.at(0)+orio){
							iml1.at<uchar>(j,i)=255;
						}
					}
				}
				else{
					
					if ( ind1<c_u && d_edges.at(2*edges_clockwise.at(ind1)).at(k)<i && d_edges.at(2*edges_clockwise.at(ind1)+1).at(k)>0){//
						iml1.at<uchar>(j,i)=255;
					}
					else{
						if( ind1>=c_u && d_edges.at(2*edges_clockwise.at(ind1)).at(k)>i && d_edges.at(2*edges_clockwise.at(ind1)+1).at(k)<jun.at(0)+orio){
							iml1.at<uchar>(j,i)=255;
						}
					}
				}
				//cout<<" 0 ";
			}
		}
	}
	Mat ims=Mat::zeros(21,21,CV_8UC1);
	for (int k=-10;k<11;k++){
			for (int k2=-10;k2<11;k2++){
				ims.at<uchar>(k+10,k2+10)=iml1.at<uchar>(jun.at(1)+k,jun.at(0)+k2);	
			}
						
		}
	/*namedWindow("imsl1",0);
	imshow("imsl1",ims);*/


	return 0;
}