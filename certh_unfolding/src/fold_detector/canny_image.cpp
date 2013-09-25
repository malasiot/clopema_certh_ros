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
struct colors{
	
	int c0;
	int c1;
	int c2;

};

struct ret_j_ed{
    vector<vector<int> > junctions;
    vector<vector<int> > detailed_edges;
	vector<int> disconnected;
    vector<vector<int> > edges_of_junct;
    vector<vector<int> > edges_t;
    vector<vector<int> > table;
};
struct a_corners{
	vector<int> certain_c;
	vector<int> str_l_c;
	vector<int> distant_c;
};

struct depths{
	vector <float> difd_c;
	vector <float> difd_s;
	vector <float> difd_d;
	vector<bool> side_c;
	vector<bool> side_s;
	vector<bool> side_d;

};


struct ret_all{
    vector<vector<int> > junctions;
    vector<vector<int> > detailed_edges;
	vector<int> disconnected;
    vector<vector<int> > edges_of_junct;
    vector<vector<int> > edges_t;
    vector<vector<int> > table;
	a_corners a_corn;
	depths d;
};
typedef uint16_t char16_t;


colors show_colors(int );
void thinningGuoHall(Mat& );
ret_fol_edges following_edges(Mat );
//void detailed_edges(vector<vector<int>>,vector<vector<int>>,Mat);
void junctions_check(vector<vector<int> >,Mat);
void junctions_check_colors(vector<vector<int> >,Mat);
void connect_junctions(vector<vector<int> > ,vector<vector<int> > ,vector<int> ,vector<vector<int> >,Mat );
ret_j_ed connect2_junctions(vector<vector<int> > ,vector<vector<int> > ,vector<int> ,vector<vector<int> > ,Mat,vector<vector<int> >,vector<vector<int> >);
int calc_closest_edge_side(vector<int>,vector<int>);
void histo( Mat);
int find_layer(vector<int>,vector<vector<int> > ,Mat,int, int,int,vector<int>,vector<vector<int> >);
void find_top_layer(Mat,Mat);
a_corners a_corner_check(vector<vector<int> > ,vector<vector<int> >,vector<vector<int> >& ,Mat,Mat,vector<vector<int> >,depths&);

void negative(Mat& im){
	for (int i=0;i<im.rows;i++){
		for (int j=0;j<im.cols;j++){
			if (im.at<uchar>(i,j)==0){
				im.at<uchar>(i,j)=255;
			}
			else{
				im.at<uchar>(i,j)=0;
			}

		}
	}
	
}


ret_all canny_image(int maxx,int minx,int maxy,int miny,Mat imagec,Mat imaged)
{	//briskw edges=h depth eikona meta to canny,roi_dep2=depth image se 8bit,roi_dep se 16,
	//roi_im h rgb
	///
    int orio=950;
	int ci=0,cj=0;
int minel,maxel;
//Mat imagec, imaged, gray;
Mat gray;
int difx=maxx-minx+1;
int dify=maxy-miny+1;
Mat roi_im1=Mat::zeros(dify,difx,CV_8UC1);
Mat roi_dep1=Mat::zeros(dify,difx,CV_16UC1);//png-16bit
//inverse roi_im1,roi_dep1
Mat roi_im=Mat::zeros(difx,dify,CV_8UC1);
Mat roi_dep=Mat::zeros(difx,dify,CV_16UC1);//png-16bit

Mat roi_dep2;


	
	GaussianBlur( imaged, imaged, Size(3,3), 0, 0, BORDER_DEFAULT );
	////////////
	//namedWindow("im_depth",0);
	//imshow("im_depth",imaged);
	//h gray image ths rgb
	cvtColor(imagec, gray, CV_BGR2GRAY);
	

	//apomonwse thn perioxh endiaferontos 
	//periorise thn perioxh timwn bathous oso ginetai
	for (int i=miny;i<maxy-1;i++){
		ci++;
		cj=0;
		
		for(int j=minx;j<maxx-1;j++){
			
			cj++;
			
			
			//gray.at<uchar>(i,j)=0;
			
			roi_im1.at<uchar>(ci,cj)=gray.at<uchar>(i,j);
			
			roi_dep1.at<char16_t>(ci,cj)=imaged.at<char16_t>(i,j);
			//cout<<roi_dep.at<char16_t>(ci,cj)<<" ";
			//an to bathos einai mikrotero apo ena orio
			//tote mhdenise to
			
			if (roi_dep1.at<char16_t>(ci,cj)<orio){
				roi_dep1.at<char16_t>(ci,cj)=0;
				roi_im1.at<uchar>(ci,cj)=0;
			}//if
			else
			{   //an h timh einai panw apo ena orio (apo gyrw gyrw perioxes)
				//dwse ths mia mikroterh timh pou exeis apofasisei
                if (roi_dep1.at<char16_t>(ci,cj)>1450){
					//roi_dep.at<char16_t>(ci,cj)=500;
					roi_dep1.at<char16_t>(ci,cj)=0;//otidhpote panw apo 2000 einai thorybos
					roi_im1.at<uchar>(ci,cj)=0;
				}//if
				else{
					//an h timh einai mesa sta apodekta oria tote apla afairese 
					//thn timh tou oriou apo to trexon bathos
					roi_dep1.at<char16_t>(ci,cj)=roi_dep1.at<char16_t>(ci,cj)-orio;

				}//else
			
			}//else
		
		//	cout<<" "<<roi_dep.at<char16_t>(ci,cj);
			
		}
		
	}
	
	///////from horizontal to vetical position
	
	for (int i=0;i<roi_dep.rows;i++){
		for (int j=0;j<roi_dep.cols;j++){
			roi_dep.at<char16_t>(i,j)=roi_dep1.at<char16_t>(j,roi_dep1.cols-i-1);
			roi_im.at<uchar>(i,j)=roi_im1.at<uchar>(j,roi_dep1.cols-i-1);
			
		}
	}
	/*imshow("im1",roi_im);
	waitKey();*/
	Mat imc_depdif;
	roi_im.copyTo(imc_depdif);
	cvtColor(imc_depdif,imc_depdif,CV_GRAY2BGR);

	//mayro perigramma 10 pixel gyrw gyrw
	//gia na mh xtypaei
	for (int i=0;i<11;i++){
		for (int j=0;j<roi_dep.cols;j++){
			roi_dep.at<char16_t>(i,j)=0;
		}
	}
	for (int j=0;j<11;j++){
		for (int i=0;i<roi_dep.rows;i++){
			roi_dep.at<char16_t>(i,j)=0;
		}
	}
	for (int i=roi_dep.rows-10;i<roi_dep.rows;i++){
		for (int j=0;j<roi_dep.cols;j++){
			roi_dep.at<char16_t>(i,j)=0;
		}
	}
	for (int j=roi_dep.cols-10;j<roi_dep.cols;j++){
		for (int i=0;i<roi_dep.rows;i++){
			roi_dep.at<char16_t>(i,j)=0;
		}
	}

	
	//apomakrynw oti exei meinei apo to perigramma
	//pou dhmioyrgeitai apo ta kena bathous
	int countp0;
	for (int i=1;i<roi_dep.rows-1;i++){
		for (int j=1;j<roi_dep.cols-1;j++){
			if (roi_dep.at<char16_t>(i,j)!=0){
				countp0=0;
				for (int ii=i-1;ii<=i+1;ii++){
					for (int jj=j-1;jj<=j+1;jj++){
						if (roi_dep.at<char16_t>(ii,jj)==0){
							countp0++;
						}//if
					
					}//for.
				}//for
				if (countp0>=5){
					roi_dep.at<char16_t>(i,j)=0;
					roi_im.at<uchar>(i,j)=0;
				}
			}
		}
	}

	//imwrite("se16bit.png",roi_dep);
	
	

	Mat displayim;
	cvtColor(roi_im,displayim,CV_GRAY2BGR);

	double anal=255.0/(500.0);//alliws anal=255/(maxel-minel);
	
	//metatroph ths 16-bit eikonas se 8bit giati alliws de mporw na
	//efarmosw canny
	roi_dep.convertTo(roi_dep2,CV_8UC1,anal);
	//imwrite("se8bit.png",roi_dep2);
	
	//imwrite("image.png",roi_dep2);
	
	//find the edges
	Mat edges;
	Canny(roi_dep2, edges, 30.0, 80.0, 3);
	/*namedWindow("edgesall",0);
	imshow("edgesall",edges);*/
	
	thinningGuoHall(edges);
	/*namedWindow("final",0);
	imshow("final",edges);
	waitKey(0);*/
	//////////////////////////////////////////////////////////////////////////////////////////////////
	Mat im;
	vector<int> tem1(im.cols,-2);
    vector<vector<int> > table(im.rows,tem1);

	
    vector<vector<int> > edges_t(10);
    vector<vector<int> > junctions;
    vector<vector<int> > detailed_edges(20);
	//junctions conneted to only one edge
	//contains the number of each junctions
	vector<int> disconnected(2);
	//for each junction shows the edges it is connected with
    vector<vector<int> > edges_of_junct(10);

	vector<int> temp(10);
	vector<int> zero(10,-90);
	vector<int> temp2(2);
	vector<int> tempj(4);
	vector<int> zeroj(4,-90);
	edges.copyTo(im);
	
	ret_fol_edges ret=following_edges(im);
	
	if (ret.found==true){
		edges_t.resize(ret.points.pntx.size()-1);
		junctions.resize(ret.points.pntx.size());
		detailed_edges.resize(2*(ret.points.pntx.size()-1));
		temp2.at(0)=ret.points.pntx.at(0);
		temp2.at(1)=ret.points.pnty.at(0);
		junctions.at(0)=temp2;
		for (int i=0;i<ret.points.pntx.size()-1;i++){
			//edges
			tempj.at(0)=ret.points.pntx.at(i);
			tempj.at(1)=ret.points.pnty.at(i);
			tempj.at(2)=ret.points.pntx.at(i+1);
			tempj.at(3)=ret.points.pnty.at(i+1);
			edges_t.at(i)=tempj;
			//junctions
			temp2.at(0)=ret.points.pntx.at(i+1);
			temp2.at(1)=ret.points.pnty.at(i+1);
			junctions.at(i+1)=temp2;
			
		}
		//detailed edges
		detailed_edges=ret.npd;

		//se kathe junction antistoixw tis akmes ths
		//to prwto kai teleutaio junction sxetizetai me mia akmh
		edges_of_junct.resize(junctions.size());
		temp=zero;
		temp.at(0)=0;
		edges_of_junct.at(0)=temp;
		temp.at(0)=edges_t.size()-1;
		edges_of_junct.at(junctions.size()-1)=temp;
		//ta ypoloipa junction se prwth fash me 2 akmes
		for (int j=1;j<junctions.size()-1;j++){
			temp.at(0)=j-1;
			temp.at(1)=j;
			edges_of_junct.at(j)=temp;
		}
		//to prwto kai teleutaio junction tha einai disconnected
		disconnected.at(0)=0;
		disconnected.at(1)=junctions.size()-1;

		//table
		table=ret.table;
	}
	while (ret.found==true){
		
		ret=following_edges(im);
		
		if (ret.found==true){
			int ned=edges_t.size();
			int n=edges_t.size()+ret.points.pntx.size()-1;
			edges_t.resize(n);
			detailed_edges.resize(2*n);

			for (int i=ned;i<n;i++){
				
				//edges
				temp.at(0)=ret.points.pntx.at(i-ned);
				temp.at(1)=ret.points.pnty.at(i-ned);
				temp.at(2)=ret.points.pntx.at(i+1-ned);
				temp.at(3)=ret.points.pnty.at(i+1-ned);
				edges_t.at(i)=temp;
				
			}

			int nj=junctions.size();
			int n2=junctions.size()+ret.points.pntx.size();
			junctions.resize(junctions.size()+ret.points.pntx.size());

			for (int i=nj;i<n2;i++){
				//junctions
				temp2.at(0)=ret.points.pntx.at(i-nj);
				temp2.at(1)=ret.points.pnty.at(i-nj);
				junctions.at(i)=temp2;
			}
			for (int i=2*ned;i<2*n;i++){
				//detailed junctions
				detailed_edges.at(i)=ret.npd.at(i-2*ned);
			}

			//enhmerwsh gia edges_of_junctions
			edges_of_junct.resize(junctions.size());
			temp=zero;
			temp.at(0)=ned;
			
			edges_of_junct.at(nj)=temp;
			temp.at(0)=n-1;
			edges_of_junct.at(n2-1)=temp;
			//ta ypoloipa junction se prwth fash me 2 akmes
			int nt=ned;
			for (int j=nj+1;j<n2-1;j++){
				temp.at(0)=nt;
				nt++;
				temp.at(1)=nt;
				edges_of_junct.at(j)=temp;
			}

			//enhmerwsh gia to poia junctions einai disconnetcted
		disconnected.resize(disconnected.size()+2);
		disconnected.at(disconnected.size()-2)=nj;
		disconnected.at(disconnected.size()-1)=n2-1;

		//table
		for (int i1=0;i1<ret.table.size();i1++){
			for (int i2=0;i2<ret.table.at(0).size();i2++){
				if (ret.table.at(i1).at(i2)!=-2){
					table.at(i1).at(i2)=ret.table.at(i1).at(i2)+ned;

				}
			}
		}

		}//if (ret.found==true)

	}//while (ret.found==true)
	
	//depict edges and junctions
	Mat depict;
	
	cvtColor(roi_im,depict,CV_GRAY2BGR);
	for (int k=0;k<edges_t.size();k++){
		colors r=show_colors(k);

		line(depict, Point(edges_t.at(k).at(0),edges_t.at(k).at(1)), Point(edges_t.at(k).at(2),edges_t.at(k).at(3)), Scalar(r.c0,r.c1,r.c2), 1, CV_AA);
	}
	for (int k=0;k<junctions.size();k++){
		for (int i=-2;i<3;i++){
			for (int j=-2;j<3;j++){
				for (int c=0;c<3;c++){
					depict.at<Vec3b>(junctions.at(k).at(1)+i,junctions.at(k).at(0)+j)[c]=255;
				}
			}
		}
	}
	//namedWindow("results",0);
	//imshow("results",depict);
	Mat im2;
	edges.copyTo(im2);
	
	//depict edges in detail
	Mat details_im,im1;
	/*cvtColor(roi_im,details_im,CV_GRAY2BGR);*/
	edges.copyTo(im1);
	negative(im1);
	cvtColor(im1,details_im,CV_GRAY2BGR);
	int cc;
	for (int i=0;i<detailed_edges.size();i=i+2){
		colors r=show_colors(i);
		cc=0;
		while (detailed_edges.at(i).at(cc+1)!=0){
			line(details_im, Point(detailed_edges.at(i).at(cc),detailed_edges.at(i+1).at(cc)), Point(detailed_edges.at(i).at(cc+1),detailed_edges.at(i+1).at(cc+1)), Scalar(r.c0,r.c1,r.c2), 1, CV_AA);
			cc++;
		}
	}
	/*namedWindow("detailed_edges",0);
	imshow("detailed_edges",details_im);
	waitKey(0);*/
	

	Mat im3;
	edges.copyTo(im3);

	ret_j_ed r_e_j=connect2_junctions( junctions,detailed_edges,disconnected,edges_of_junct,im3,table,edges_t );
	
	Mat iml1;
	edges.copyTo(iml1);
	depths d;
	a_corners r_a_c=a_corner_check(r_e_j.junctions,r_e_j.edges_t, r_e_j.edges_of_junct, iml1,roi_dep2,r_e_j.table,d);

	

	/*	namedWindow("iml1",0);
		imshow("iml1",iml1);
		waitKey(0);*/
//	}
	//waitKey(0);

	Mat disc_junctions;
	edges.copyTo(disc_junctions);
	//cout<<" junctions num "<<r_e_j.junctions.size()<<" disconnected"<<endl;
	for (int i=0;i<r_e_j.edges_of_junct.size();i++){
		if (r_e_j.edges_of_junct.at(i).at(1)==-90){
		for (int o1=-3;o1<4;o1++){
			for (int o2=-3;o2<4;o2++){
				disc_junctions.at<uchar>(r_e_j.junctions.at(i).at(1)+o1,r_e_j.junctions.at(i).at(0)+o2)=255;
			}
		}
		}
	}
	//namedWindow("disconnected junctions",0);
	//imshow("disconnected junctions",disc_junctions);
	//waitKey(0);


	

	////////////////////////////
	ret_all r;
	r.a_corn=r_a_c;
	r.detailed_edges=r_e_j.detailed_edges;
	r.disconnected=r_e_j.disconnected;
	r.edges_of_junct=r_e_j.edges_of_junct;
	r.edges_t=r_e_j.edges_t;
	r.junctions=r_e_j.junctions;
	r.table=r_e_j.table;
	r.d=d;
	
	return r;

}
