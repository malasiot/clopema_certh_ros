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

struct corner_depths{
	vector<float> average;
	vector<float> averagex;
	vector<float> averagey;
	bool layers3;
};


bool check_depths(float side,corner_depths r){


//elegxw pws einai moirasmena ta bathh
			int lu1=0,ru1=0,rd1=0,ld1=0;
			int lu2=0,ru2=0,rd2=0,ld2=0;
			if (side<0){//an to hmiepipedo me tis akmes einai deksia
				//apo ayto pou prepei na einai pio konta
				//pros to pio makrino
				lu1=3;
				ld1=3;
				ru1=1;
				rd1=2;
			}
			else{
				ru1=3;
				rd1=3;
				ld1=2;
				lu1=1;
			}
			//briskw th seira symfwna me thn epistrofh apo ta corner_depths
			
			vector<float> xav=r.averagex,yav=r.averagey;
		//	cout<<"averages"<<endl;
		/*	for (int io=0;io<3;io++){
				cout<<" i "<<io<<" "<<r.average.at(io)<<endl<<" xav "<<xav.at(io)<<" yav "<<yav.at(io)<<endl;
			}*/
			
			if (side<0){//
				
				
				//find left
				int ll=0;
				for (int ii=1;ii<3;ii++){
					if (xav.at(ll)>xav.at(ii)){
						ll=ii;
					}
					if (xav.at(ll)==xav.at(ii)){
						if (r.average.at(ll)<r.average.at(ii)){
							ll=ii;
						}
					}
				}
				//einai me ayksousa seira opote deixnei to i deixnei kai poso bathia einai se sxesh me ta alla
				lu2=ll+1;
				ld2=ll+1;
				//bres ta 2 pou den taksinomhthhkan
				vector<int> tem(2);
				int ct=0;
				for (int ii=0;ii<3;ii++){
					if (ll!=ii){
						tem.at(ct)=ii;
						ct++;
					}
				}
				//	///////////
			/*	Mat test;
				ims.copyTo(test);
				int keept;
				
				for (int kk1=-1;kk1<2;kk1++){

					for (int kk2=-1;kk2<2;kk2++){

						test.at<uchar>((int)(yav.at(ll))+kk1,(int)(xav.at(ll))+kk2)=255;
					}
				}
				for (int kk1=0;kk1<3;kk1++){
					for (int kk2=-2;kk2<3;kk2++){
					test.at<uchar>((int)(yav.at(kk1)),(int)(xav.at(kk1)+kk2))=255;
					}
				}
				namedWindow("test",0);
				imshow("test",test);*/
				/////////////
				
				//bres poio einai pio panw kai poio pio katw
				if(yav.at(tem.at(0))<yav.at(tem.at(1))){
					ru2=tem.at(0)+1;
					rd2=tem.at(1)+1;
				}
				else{
					ru2=tem.at(1)+1;
					rd2=tem.at(0)+1;
				}
			
			}
			else{
			
					//find right
				int ll=0;
				for (int ii=1;ii<3;ii++){
					if (xav.at(ll)<xav.at(ii)){
						ll=ii;
					}
					if (xav.at(ll)==xav.at(ii)){
						if (r.average.at(ll)<r.average.at(ii)){
							ll=ii;
						}
					}
				}
		//	///////////
			/*	Mat test;
				ims.copyTo(test);
				int keept;
				
				for (int kk1=-1;kk1<2;kk1++){

					for (int kk2=-1;kk2<2;kk2++){

						test.at<uchar>((int)(yav.at(ll))+kk1,(int)(xav.at(ll))+kk2)=255;
					}
				}
				for (int kk1=0;kk1<3;kk1++){
					for (int kk2=-2;kk2<3;kk2++){
					test.at<uchar>((int)(yav.at(kk1)),(int)(xav.at(kk1)+kk2))=255;
					}
				}
				namedWindow("test",0);
				imshow("test",test);*/
				/////////////
				//einai me ayksousa seira opote deixnei to i deixnei kai poso bathia einai se sxesh me ta alla
				ru2=ll+1;
				rd2=ll+1;
				//bres ta 2 pou den taksinomhthhkan
				vector<int> tem(2);
				int ct=0;
				for (int ii=0;ii<3;ii++){
					if (ll!=ii){
						
						tem.at(ct)=ii;
						ct++;
					}
				}
				//	cout<<"tem"<<tem.at(0)<<" "<<tem.at(1)<<endl;
				//bres poio einai pio panw kai poio pio katw
				if(yav.at(tem.at(0))<yav.at(tem.at(1))){
					lu2=tem.at(0)+1;
					ld2=tem.at(1)+1;
				}
				else{
					lu2=tem.at(1)+1;
					ld2=tem.at(0)+1;
				}
			}
			bool ok;
			if (ru2==ru1 && rd2==rd1 && lu2==lu1 && ld2==ld1){
				ok=true;
				/*cout<<"OK!!!!!!!!!!"<<endl;
				cout<<" 1 "<<ru1<<" "<<rd1<<" "<<lu1<<" "<<ld1<<endl;
				cout<<" 2 "<<ru2<<" "<<rd2<<" "<<lu2<<" "<<ld2<<endl;*/
			}
			else{
				ok=false;
				/*cout<<"Not ok!!!!!!!"<<endl;
				cout<<" 1 "<<ru1<<" "<<rd1<<" "<<lu1<<" "<<ld1<<endl;
				cout<<" 2 "<<ru2<<" "<<rd2<<" "<<lu2<<" "<<ld2<<endl;*/
			 }
			return ok;
	}