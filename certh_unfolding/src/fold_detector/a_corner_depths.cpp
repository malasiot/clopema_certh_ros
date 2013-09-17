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

corner_depths a_corner_depth(Mat iml,Mat imdl,vector<int> jun){

	corner_depths ret;
	vector<float> averages;//krata tous mesous orous twn epipedwn
	//vector<vector<Point>> store_coor;//krata tis syntetagmenes tou kathe epipedou
	vector<float> averagesx,averagesy;//mesoi oroi tvn syntetagmenwn x kai y kathe epipedou
	vector<int> lsize;//o arithmos twn pixel kathe epipedou
	Mat im=Mat::zeros(19,19,CV_8UC1);
	Mat imd=Mat::zeros(19,19,CV_8UC1);
	for (int k=-9;k<10;k++){
			for (int k2=-9;k2<10;k2++){
				im.at<uchar>(k+9,k2+9)=iml.at<uchar>(jun.at(1)+k,jun.at(0)+k2);	
				imd.at<uchar>(k+9,k2+9)=imdl.at<uchar>(jun.at(1)+k,jun.at(0)+k2);	
			}
						
		}

	/*namedWindow("ims",0);
	imshow("ims",im);*/
	// bres an kapoio pixel den enwnetai kapou
	int countn;
	vector<Point> disconnected;
	for (int i=1;i<im.rows-1;i++){
		for (int j=1;j<im.cols-1;j++){
			if (im.at<uchar>(i,j)==255){
				countn=0;
				for (int in=i-1;in<i+2;in++){
					for (int jn=j-1;jn<j+2;jn++){
						if (im.at<uchar>(in,jn)==255){
						countn++;
						
						}
					}
				}
				if (countn<3){
					disconnected.push_back(Point(i,j));
				}
			}
		}
	}

//trabaw mia orizontia grammh mexri to kontynotero leuko pixel
	int f1=500,f2=500,count_times=0;
	
	for (int i=0;i<disconnected.size();i++){
		f1=500;f2=500;
		int x=disconnected.at(i).x,y=disconnected.at(i).y;
		//epanalambanw merikes fores se periptwsh pou de ginei h enwsh amesws
		//gia periptwseis einai ligo metatopismeno to asyndeto pixel px im 309
		while (f1==500 && f2==500 && count_times<4){
		count_times++;
		
		if (x>1 && x<im.rows-1 && y<im.cols-1){
		if (im.at<uchar>(x,y+1)!=255 && im.at<uchar>(x+1,y+1)!=255 && im.at<uchar>(x-1,y+1)!=255){
			
			for (int k=disconnected.at(i).y+1;k<im.cols;k++){
				if (im.at<uchar>(disconnected.at(i).x,k)==255){
					f1=k;
				}
			}
		}
		}
		if (x>1 && x<im.rows-1 && y>1){
		if (im.at<uchar>(x,y-1)!=255 && im.at<uchar>(x+1,y-1)!=255 && im.at<uchar>(x-1,y-1)!=255){
			for (int k=disconnected.at(i).y-1;k>=0;k--){
				if (im.at<uchar>(disconnected.at(i).x,k)==255){
					f2=k;
				}
			}
		}
		}
		//An exei pros kai tis 2 kateythynseis
		if (!(f1==500 && f2==500)){
			if (f1<f2){
				for (int k=disconnected.at(i).y;k<=f1;k++){
					im.at<uchar>(disconnected.at(i).x,k)=255;
				}
			}
			else{
				for (int k=disconnected.at(i).y;k>=f2;k--){
					im.at<uchar>(disconnected.at(i).x,k)=255;
				}
			}
		}
		
		if (f1==500 && f2==500){
			if(x>1 && x<im.rows-1){
				if ((y<im.cols-1) && (im.at<uchar>(x,y+1)==255 || im.at<uchar>(x+1,y+1)==255 || im.at<uchar>(x-1,y+1)==255)){
					y=y+1;
					
				}
				else{
					if((y>1) && (im.at<uchar>(x,y-1)==255 || im.at<uchar>(x+1,y-1)==255 || im.at<uchar>(x-1,y-1)==255)){
						y=y-1;
						
					}
				}
			}
		}

		
		}//while
		
		//dokimazw kai katheta
		if (f1==500 && f2==500){
			x=disconnected.at(i).x,y=disconnected.at(i).y;
			if(x<im.rows-1 && y<im.cols-1 && y>1){
				if (im.at<uchar>(x+1,y)!=255 && im.at<uchar>(x+1,y+1)!=255 && im.at<uchar>(x+1,y-1)!=255){
					for (int k=disconnected.at(i).x+1;k<im.rows;k++){
						if (im.at<uchar>(k,disconnected.at(i).y)==255){
							f1=k;
						}
					}
				}
			}
			if (x>1 && y<im.cols-1 && y>1){
				if (im.at<uchar>(x-1,y)!=255 && im.at<uchar>(x-1,y+1)!=255 && im.at<uchar>(x-1,y-1)!=255){
					for (int k=disconnected.at(i).x-1;k>=0;k--){
						if (im.at<uchar>(k,disconnected.at(i).y)==255){
							f2=k;
						}
					}
				}
			}
			//An exei pros kai tis 2 kateythynseis
		if (!(f1==500 && f2==500)){
			if (f1<f2){
				for (int k=disconnected.at(i).x;k<=f1;k++){
					im.at<uchar>(k,disconnected.at(i).y)=255;
				}
			}
			else{
				for (int k=disconnected.at(i).x;k>=f2;k--){
					im.at<uchar>(k,disconnected.at(i).y)=255;
				}
			}
		}

		}

	}

	///////////////////////////////////////////////////////////////////////////////////
	if (!(f1==500 && f2==500 && disconnected.size()>0)){
		//find the first pixel!=255
	int sti,stj;
	bool flag=false;
	for (int i=0;i<im.rows && flag==false;i++ ){
		for (int j=0;j<im.cols;j++){
			if (im.at<uchar>(i,j)!=255){
				sti=i;
				stj=j;
				flag=true;
				break;
			}
		}
	}
	
	while(flag==true){
	vector<Point> coor;
	int count=0,sum=0,sumx=0,sumy=0,right_stopper=im.cols;
	bool stop=false;
	for(int i=sti;i<im.rows;i++){
		
		//to prwto pixel ths epomenhs seiras
		//an pesei panw se akmh
		if (im.at<uchar>(i,stj)==255){
			while(im.at<uchar>(i,stj)==255 && stj<im.cols-1 && stj<right_stopper){
				stj++;
			}
			if (stj==im.cols-1){
				break;
			}
			if (stj>=right_stopper){
				stop=true;
				break;
			}
		}
		else{
		//an prepei na brw apo poio pixel na ksekinhsw
			//phgainw pros ta pisw gia na brw grammh 
			int in_stj=stj;
			//elegxw mexri pisw// 3 pshfia to palio
			bool f=false;
			//while (im.at<uchar>(i,stj)!=255 && stj>0 && in_stj-4<stj){
			while (im.at<uchar>(i,stj)!=255 && stj>0 ){
				stj--;
				if (im.at<uchar>(i,stj)==255){
					
					stj++;
					f=true;
					break;

				}
			}
			
			if (f==false){
				stop=true;
			}
			if (im.at<uchar>(i,stj)!=255 && stj==0){
				stop=false;
			}
		}
		//an synexizoume me to idio layer
		
		if (stop==false){
			
			for (int j=stj;j<im.cols;j++){
				if (im.at<uchar>(i,j)!=255){
					count++;
					sum=sum+imd.at<uchar>(i,j);
					im.at<uchar>(i,j)=255;
					right_stopper=j;
					
					sumy=sumy+i;
					sumx=sumx+j;
					
					
				}
				else{
					break;
				}
			}
		}
		

	}


	////search for new layer
	flag=false;
	for (int i=0;i<im.rows && flag==false;i++ ){
		for (int j=0;j<im.cols;j++){
			if (im.at<uchar>(i,j)!=255){
				sti=i;
				stj=j;
				flag=true;
				break;
			}
		}
	}
	//store the average depth
	averages.push_back(sum/count);
	averagesx.push_back(sumx/count);
	averagesy.push_back(sumy/count);
	lsize.push_back(count);
	right_stopper=im.cols;
	
	}


	//keep the 3 bigger layers(in case there are more than 3)
	
	//gia thn periptwsh mono pou ta epipeda einai 3 kai panw
	if (averages.size()>2){

	vector <int> lsize2=lsize;
	std::sort(lsize.rbegin(),lsize.rend());

	vector<float> av(3),avx(3),avy(3);
	
	for (int i=0;i<3;i++){
		for (int j=0;j<lsize.size();j++){
			if (lsize.at(i)==lsize2.at(j)){
				av.at(i)=averages.at(j);
				avx.at(i)=averagesx.at(j);
				avy.at(i)=averagesy.at(j);
			}
		}
	}
	/*for (int i=0;i<3;i++){
		cout<<" i= "<<i<<" length= "<<layers_lengths.at(i)<<" av= "<<av.at(i)<<endl;
	}*/

	//opou average~0 kane to 5000 wste na fainetai oti einai pio bathia
	for (int i=0;i<3;i++){
		if (av.at(i)<9){
			av.at(i)=5000;
		}
	}
	//taksinomish symfwna me to meso bathos twn epipedwn
	vector<float> avf=av,avxf(3),avyf(3);
	
	std::sort(avf.begin(),avf.end());
	
		for (int j=0;j<3;j++){
			for (int i=0;i<av.size();i++){
				if (avf.at(j)==av.at(i)){
					avxf.at(j)=avx.at(i);
					avyf.at(j)=avy.at(i);
					break;
				}			
			}
		}
	

	
	ret.average=avf;
	ret.averagex=avxf;
	ret.averagey=avyf;
	ret.layers3=true;
	}
	else{
		vector<float> t(3,0);
		ret.average=t;
		ret.averagex=t;
		ret.averagey=t;
		ret.layers3=false;
	}
	
	}
	else{
		vector<float> t(3,0);
		ret.average=t;
		ret.averagex=t;
		ret.averagey=t;
		ret.layers3=false;
	}
	
	
	return ret;
}