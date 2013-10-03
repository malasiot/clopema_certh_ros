#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "FoldDetector.h"
#include <fstream>
#include <iostream>
//#include <string>

using namespace cv;
using namespace std;

int calc_closest_edge_side(vector<int> ,vector<int>);

float angleMeasurement(vector<int> jun, vector<int> v1, vector <int> v2){
	//calculate the corner formed by the vector v1 and v2
	//we consider that they start from the same point
	int c1=calc_closest_edge_side(v1 , jun);
	int o1=0;
	if (c1==0){o1=2;}

	int c2=calc_closest_edge_side(v2 , jun);
	int o2=0;
	if (c2==0){o2=2;}



	float v12=0;
	for (int i=0;i<2;i++){ 
		v12=v12+v1.at(o1+i)*v2.at(o2+i);
	}
	float norm1=v1.at(o1)*v1.at(o1)+v1.at(o1+1)*v1.at(o1+1);
	norm1=sqrt(norm1);
	float norm2=v2.at(o2)*v2.at(o2)+v2.at(o2+1)*v2.at(o2+1);
	norm2=sqrt(norm2);
	float nrm=norm1*norm2;
	float cth=v12/nrm;
	//cout<<"cth= "<<cth;
	float th=acos(cth);
	//cout<<" th= "<<th;
	return th;


}


Point search_area(int corner, ret_all r, bool side, Mat bin, vector<int> vec, vector<int> o, int & ri){
	Mat edges;
	bin.copyTo(edges);
	for (int i=0;i<r.detailed_edges.size();i=i+2){
		
		if (r.detailed_edges.at(i).size()>0){
			for (int j=0;j<r.detailed_edges.at(i).size()-1;j++){
				line(edges, Point(r.detailed_edges.at(i).at(j),r.detailed_edges.at(i+1).at(j)), Point(r.detailed_edges.at(i).at(j+1),r.detailed_edges.at(i+1).at(j+1)), 255, 1, CV_AA);
			}
		}
	}
	/*namedWindow("edges",0);
	imshow("edges",edges);
	waitKey();*/
	/////////////////////
	//draw the vectors
	
	for (int i=0;i<2;i++){
		int ind = r.edges_of_junct.at(corner).at(vec.at(i));
		for (int j = 0; j < r.detailed_edges.at(2*ind).size()-1 ; j++){
			
			line(bin, Point(r.detailed_edges.at(2*ind).at(j),r.detailed_edges.at(2*ind+1).at(j)), Point(r.detailed_edges.at(2*ind).at(j+1),r.detailed_edges.at(2*ind+1).at(j+1)), 255, 2, CV_AA);
		}
	}
	

	////junction
	//for (int o1=-4;o1<5;o1++){
	//	for (int o2=-4;o2<5;o2++){
	//		//bin.at<uchar>(r.junctions.at(corner).at(1)+100+o1,r.junctions.at(corner).at(0)+o2)=255;
	//		bin.at<uchar>(r.junctions.at(corner).at(1)+o1,r.junctions.at(corner).at(0)+o2)=255;
	//	}
	//}


	/*namedWindow("bin",0);
	imshow("bin",bin);
	waitKey();*/
	//////////////
	//detect the area for the normal extraction



	
	//cout<<"Voted =" <<voted<<" vote u "<<voteu;
	int istarty,istopy,istartx,istopx;
	
	if (side==false){
		istarty=r.junctions.at(corner).at(1);
		istopy=istarty+20;
	}
	else{
		istarty=r.junctions.at(corner).at(1)-20;
		istopy=r.junctions.at(corner).at(1);
	}

	istartx=r.junctions.at(corner).at(0)-20;
	istopx=r.junctions.at(corner).at(0)+20;

	//////////////////draw///
	
	for (int o1=istartx;o1<istopx;o1++){

		for (int o2=-1;o2<1;o2++){
		bin.at<uchar>(istarty+o2,o1+o2)=255;
		}
	}

	for (int o1=istarty;o1<istopy;o1++){

		for (int o2=-1;o2<1;o2++){
		bin.at<uchar>(o1+o2,istartx+o2)=255;
		}
	}


	//namedWindow("bin",0);
	//imshow("bin",bin);
	//waitKey();
	Mat bin2;
	bin.copyTo(bin2);

	int xgr,ygr;
	bool found=false;
	while (ri>0 && found==false){
		ri--;
        //cout<<ri<<" ";
		for (int xx=istartx+ri; xx<istopx-ri && found==false;xx++){
			for (int yy=istarty; yy<istopy-ri;yy++){
				//bin.at<uchar>(yy,xx)=255;
				bin.copyTo(bin2);
				bool wrong_square=false;
				for (int xi=xx-ri; xi<xx+ri && wrong_square==false;xi++){
					for (int yi=yy-ri; yi<yy+ri; yi++){

						bin2.at<uchar>(yi,xi)=255;
						
						vector<float> dif(2);
						for (int ii=0; ii<2; ii++){

							int ind = r.edges_of_junct.at(corner).at(vec.at(ii)); 
							bool expr3=(yi>r.edges_t.at(ind).at(o.at(vec.at(ii))+1) && side==false);
							bool expr4=(yi<r.edges_t.at(ind).at(o.at(vec.at(ii))+1) && side==true);
							if (expr3==true || expr4==true){

								dif.at(ii)=xi-r.edges_t.at(ind).at(o.at(vec.at(ii)));
								
							}
							else{
								for (int k=0;k<r.detailed_edges.at(2*ind).size()-1 && r.detailed_edges.at(2*ind).at(k+1)>0; k++){
									bool expr1=(yi>=r.detailed_edges.at(2*ind+1).at(k) && yi<=r.detailed_edges.at(2*ind+1).at(k+1));
									bool expr2=(yi<=r.detailed_edges.at(2*ind+1).at(k) && yi>=r.detailed_edges.at(2*ind+1).at(k+1));
									if (expr1==true || expr2==true){
										
										float par= r.detailed_edges.at(2*ind+1).at(k)-r.detailed_edges.at(2*ind+1).at(k+1);
										float ar= r.detailed_edges.at(2*ind).at(k)-r.detailed_edges.at(2*ind).at(k+1);
										if (par!=0){
											float a=ar/par;
											float b=-a*r.detailed_edges.at(2*ind+1).at(k)+r.detailed_edges.at(2*ind).at(k);
											float xd= a*yi+b;
												dif.at(ii)=xi-xd;
										}
										else{
											dif.at(ii)=xi-r.detailed_edges.at(2*ind).at(k+1);
										}
									
									}
									
								}
							
								
							}
							if (edges.at<uchar>(yi,xi)>200){//
								dif.at(ii)=0;
							}
							/*for (int o1=-2;o1<3;o1++){
								for (int o2=-2;o2<3;o2++){
									edges.at<uchar>(yi+o1,xi+o2)=255;
								}
							}
							imshow("edges",edges);*/
							//waitKey();
						}

						//cout<<"                "<<dif.at(0)<<" "<<dif.at(1)<<endl;
						if (dif.at(0)*dif.at(1)>=0){
							wrong_square=true;
							break;
						}

					}
				}
				if (wrong_square==false){
					xgr=xx;
					ygr=yy;

					found=true;
					break;
				
				}
				/*namedWindow("bin2",0);
				imshow("bin2",bin2);*/
				//waitKey();
			
			}
		}
	}

	Point P;

	if (found==true){
		for (int o1=-ri;o1<ri;o1++){
			for (int o2=-ri; o2<ri ;o2++){
				bin2.at<uchar>(ygr+o1,xgr+o2)=255;
			}
		}
		P.x=xgr;P.y=ygr;

			/*namedWindow("winner",0);
			imshow("winner",bin2);
			waitKey();*/
	}
	else{
		P.x=0;P.y=0;
	}
	

	return P;

}



int normalPoint(int corner, ret_all r, bool side, Mat imc, Point & P){
	int tr=9;
	int ri=tr,r0;
	
	Mat bin=cv::Mat::zeros(imc.rows,imc.cols,CV_8UC1);

	////////////////////////////////////////////////////////"T-junction"
	if (r.edges_of_junct.at(corner).at(1)==r.edges_of_junct.at(corner).at(2)){
		int ind=r.edges_of_junct.at(corner).at(1);
		vector<int> vec(2);
		vector<int> o(2);
		int upper_edge;
		int c1=calc_closest_edge_side(r.edges_t.at(r.edges_of_junct.at(corner).at(0)), r.junctions.at(corner));
		o.at(0)=0;
		if (c1==0){o.at(0)=2;}
		o.at(1)=2;
		if (r.edges_t.at(ind).at(o.at(0))>r.edges_t.at(ind).at(o.at(1))){
			upper_edge=0;
		}
		else{
			upper_edge=1;
		}
		//fake edgest 
		vector<int> edgest1(4), edgest2(4);
		edgest1.at(0)=r.junctions.at(corner).at(0);
		edgest1.at(1)=r.junctions.at(corner).at(1);
		edgest1.at(2)=r.edges_t.at(r.edges_of_junct.at(corner).at(1)).at(0);
		edgest1.at(3)=r.edges_t.at(r.edges_of_junct.at(corner).at(1)).at(1);
		edgest2.at(0)=r.junctions.at(corner).at(0);
		edgest2.at(1)=r.junctions.at(corner).at(1);
		edgest2.at(2)=r.edges_t.at(r.edges_of_junct.at(corner).at(1)).at(2);
		edgest2.at(3)=r.edges_t.at(r.edges_of_junct.at(corner).at(1)).at(3);

		//find the point on the edge that the junction corresponds to
		int xs=r.junctions.at(corner).at(0),ys,keepi;
		if (r.detailed_edges.at(2*ind+1).size()>0){//<-------------------------------------------------------------------------------
		for (int i=0; i<r.detailed_edges.at(2*ind).size()-1 && r.detailed_edges.at(2*ind).at(i+1)>0; i++){
			bool expr1=(r.junctions.at(corner).at(0)>=r.detailed_edges.at(2*ind).at(i) && r.junctions.at(corner).at(0)<=r.detailed_edges.at(2*ind).at(i+1));
			bool expr2=(r.junctions.at(corner).at(0)<=r.detailed_edges.at(2*ind).at(i) && r.junctions.at(corner).at(0)>=r.detailed_edges.at(2*ind).at(i+1));
			if ((expr1==true) || (expr2==true)){
				float ar= r.detailed_edges.at(2*ind+1).at(i)-r.detailed_edges.at(2*ind+1).at(i+1);
				float par =r.detailed_edges.at(2*ind).at(i)-r.detailed_edges.at(2*ind).at(i+1);
				if (par!=0){
					float a=ar/par;
					float b=-a*r.detailed_edges.at(2*ind).at(i)+r.detailed_edges.at(2*ind+1).at(i);
					ys=int(a*r.junctions.at(corner).at(0)+b);
				}
				else{
					ys=r.detailed_edges.at(2*ind+1).at(i);
				}
				keepi=i;//the point is found
			}

		}
		//fake detailed edges
		//
		vector<vector<int> > det1(2),det2(2);
		vector <int> temp(2);
		/*temp.at(0)=edgest1.at(2);
		temp.at(1)=edgest1.at(3);*/
		temp.at(0)=r.detailed_edges.at(2*ind).at(0);
		temp.at(1)=r.detailed_edges.at(2*ind+1).at(0);
		int cl=calc_closest_edge_side(r.edges_t.at(r.edges_of_junct.at(corner).at(1)), temp);
		//if (cl==0)
		
			det1.at(0).resize(keepi+2);
			det1.at(1).resize(keepi+2);
			
			for (int i=0;i<=keepi;i++){
				det1.at(0).at(i)=r.detailed_edges.at(2*ind).at(i);
				det1.at(1).at(i)=r.detailed_edges.at(2*ind+1).at(i);
			}
			
			det1.at(0).at(keepi+1)=xs;
			det1.at(1).at(keepi+1)=ys;
			
			det2.at(0).resize(1);
			det2.at(1).resize(1);
			det2.at(0).at(0)=xs;
			det2.at(1).at(0)=ys;
			
			/*cout<<keepi+1<< " "<<r.detailed_edges.at(2*ind).size();
			cout<<xs<<" "<<ys<<endl;*/
			for (int i=keepi+1;i<r.detailed_edges.at(2*ind).size() && r.detailed_edges.at(2*ind).at(i)>0;i++){
				
				det2.at(0).resize(i-keepi);
				det2.at(1).resize(i-keepi);
				//cout<<" "<<r.detailed_edges.at(2*ind).at(i)<<" "<<r.detailed_edges.at(2*ind+1).at(i)<<endl;
				det2.at(0).push_back(r.detailed_edges.at(2*ind).at(i));
				det2.at(1).push_back(r.detailed_edges.at(2*ind+1).at(i));

			}
			/*cout<<endl;
			for (int i=0;i<det2.at(0).size();i++){
				cout<<det2.at(0).at(i)<<" "<<det2.at(1).at(i)<<endl;
			}*/


			if (cl==2){
				vector<vector<int> > tempv=det2;
				det2=det1;
				det1=det2;
			}
			///////////////////////////////////drawings////////////////////////////////////////
		/*	for (int i=0;i<det1.at(0).size()-1;i++){
				line(bin, Point(det1.at(0).at(i),det1.at(1).at(i)), Point(det1.at(0).at(i+1),det1.at(1).at(i+1)), 255, 2, CV_AA);
			}
			
		for (int o1=-4;o1<5;o1++){
			for (int o2=-4;o2<5;o2++){
				bin.at<uchar>(ys+o1,xs+o2)=255;
				bin.at<uchar>(r.junctions.at(corner).at(1)+o1,r.junctions.at(corner).at(0)+o2)=255;
			}
		}*/
		/*for (int j=0;j<r.detailed_edges.at(2*ind).size()-1 && r.detailed_edges.at(2*ind).at(j+1)>0;j++){
			line(bin, Point(r.detailed_edges.at(2*ind).at(j),r.detailed_edges.at(2*ind+1).at(j)), Point(r.detailed_edges.at(2*ind).at(j+1),r.detailed_edges.at(2*ind+1).at(j+1)), 255, 2, CV_AA);
		}*/
		/*namedWindow("Point",0);
		imshow("Point",bin);
		waitKey();*/
		/*for (int i=0;i<det2.at(0).size()-1 && det2.at(0).at(i+1)>0;i++){
				line(bin, Point(det2.at(0).at(i),det2.at(1).at(i)), Point(det2.at(0).at(i+1),det2.at(1).at(i+1)), 255, 2, CV_AA);
			}
		namedWindow("Point",0);
		imshow("Point",bin);
		waitKey();*/
		//detect the lowest area of interest
		vec.at(0)=0;vec.at(1)=1;
		if (upper_edge==1){
			r.detailed_edges.at(2*ind)=det1.at(0);
			r.detailed_edges.at(2*ind+1)=det1.at(1);
			r.edges_t.at(ind)=edgest1;
			P= search_area( corner,  r, side,  bin, vec, o,ri);
			r0=ri;
		}
		else
		{
			//ri=tr;
			////
			r.detailed_edges.at(2*ind)=det2.at(0);
			r.detailed_edges.at(2*ind+1)=det2.at(1);
			r.edges_t.at(ind)=edgest2;
			P=search_area( corner,  r, side,  bin, vec, o,ri);
			r0=ri;
		}
			}////<---------------------------------------------------------------------
		else{
			P.x=0;
			P.y=0;
			r0=0;
		}//<---------------------------------------------------
	}
	else{////////////////////////////////////////arrow junction////////////////////////////////
		vector<int> tv(2);
		vector<vector<int> > vec(2,tv);
		vector<int> o(3);
		vector<vector<int> > v(3);

	//the 3 vectors of the corner
	
	for (int i=0;i<3;i++){
		v.at(i)=r.edges_t.at(r.edges_of_junct.at(corner).at(i));
	}


	vector<double> f(3),fb(3);
	f.at(0)=angleMeasurement(r.junctions.at(corner), v.at(0) ,v.at(1));
	f.at(1)=angleMeasurement(r.junctions.at(corner), v.at(0) ,v.at(2));
	f.at(2)=angleMeasurement(r.junctions.at(corner), v.at(1) ,v.at(2));
	fb=f;
	//increasing order
	std::sort(fb.begin(),fb.end());
	//the two smallest angles
	
	int count=0;
	for (int i=0;i<3;i++){
		if ((f.at(i)==fb.at(0) || f.at(i)==fb.at(1)) && count<2){
			switch (i)
			{
			case 0:
				vec.at(count).at(0)=0;
				vec.at(count).at(1)=1;
				break;
			case 1:
				vec.at(count).at(0)=0;
				vec.at(count).at(1)=2;
				break;
			case 2:
				vec.at(count).at(0)=1;
				vec.at(count).at(1)=2;
				break;

			}
			count++;
		}
	}
	
	for (int i=0;i<3;i++){
		int c1=calc_closest_edge_side(v.at(i) , r.junctions.at(corner));
		o.at(i)=0;
		if (c1==0){o.at(i)=2;}
	}

	//find the common edge
	int common;
	for (int i=0;i<2;i++){
		for (int j=0;j<2;j++){
			if (vec.at(0).at(i)==vec.at(1).at(j)){
				common=vec.at(0).at(i);
			}
		}
	}
	//find the 2 not common edges
	vector<int > not_common(2);
	for (int i=0;i<2;i++){
		for (int j=0;j<2;j++){
			if (vec.at(i).at(j)!=common){
				not_common.at(i)=vec.at(i).at(j);
			}
		}
	}
	//detect the lowest area of interest
	int part1=r.edges_t.at(r.edges_of_junct.at(corner).at(not_common.at(0))).at(o.at(not_common.at(0)));
	int part2=r.edges_t.at(r.edges_of_junct.at(corner).at(not_common.at(1))).at(o.at(not_common.at(1)));
	if (part1<part2){
		ri=tr;
		P=search_area( corner,  r, side,  bin, vec.at(0), o, ri);
		r0=ri;
	}
	else{
		ri=tr;
		P=search_area( corner,  r, side,  bin, vec.at(1), o, ri);
		r0=ri;
	}
	}
	
	
	

	if (P.x==0){
		r0=0;
		P.x=r.junctions.at(corner).at(0);
		P.y=r.junctions.at(corner).at(1);
	}
	
	

	for (int o1=-r0;o1<r0;o1++){
		for (int o2=-r0;o2<r0;o2++){
			bin.at<uchar>(P.y+o1,P.x+o2)=255;
		}
	}
//	cout<<"side "<<side;
//	cout<<" R= "<<r0;
//	namedWindow("final winner",0);
//	imshow("final winner", bin);
//	waitKey();
	return r0;

}
