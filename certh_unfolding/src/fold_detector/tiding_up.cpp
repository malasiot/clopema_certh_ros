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
void negative(Mat&);
colors show_colors(int );



int calc_closest_edge_side(vector<int> edge,vector<int> junction){
	//bres poia akrh ths akmhw ntistoixei sto koino junction
	//epistrefei 0 h 2 analogws me to poia akrh einai plhsiestera
			double dist01,dist23,tem1,tem2;
			tem1=edge.at(0)-junction.at(0);
			tem1=tem1*tem1;
			tem2=edge.at(1)-junction.at(1);
			tem2=tem2*tem2;
			dist01=sqrt(tem1+tem2);
			tem1=edge.at(2)-junction.at(0);
			tem1=tem1*tem1;
			tem2=edge.at(3)-junction.at(1);
			tem2=tem2*tem2;
			dist23=sqrt(tem1+tem2);
			if (dist01<dist23){
				return 0;
			}
			else{
				return 2;
			}
}


ret_j_ed tiding_up(vector<vector<int> > junctions,vector<vector<int> > detailed_edges,vector<int> disconnected,vector<vector<int> > edges_of_junct,vector<vector<int> > edges_t,vector<int> connect,Mat im,vector<vector<int> > table){
	//diwxnw osa pleon den einai disconnected apo ton pinaka
	int count=0;
	for (int i=0;i<disconnected.size();i++){
		if (disconnected.at(i)==-1){
			count++;
		}
		else{
			disconnected.at(i-count)=disconnected.at(i);
		}
	}
	disconnected.resize(disconnected.size()-count);
	////
	//enwnw akmes kai sbhnw ta antistoixa junction
	for (int i=0;i<junctions.size();i++){
		if (connect.at(i)==1){
			int e1=edges_of_junct.at(i).at(0);
			int e2=edges_of_junct.at(i).at(1);
			int cl1=calc_closest_edge_side(edges_t.at(e1),junctions.at(i));
			int o1=0;
			if (cl1==0){
				o1=2;
			}
			int cl2=calc_closest_edge_side(edges_t.at(e2),junctions.at(i));
			int o2=0;
			if (cl2==0){
				o2=2;
			}
			
			//enhmerwsh
			edges_of_junct.at(i).at(0)=-90;
			for (int k=0;k<edges_of_junct.size();k++){
				for (int l=0;l<edges_of_junct.at(0).size();l++){
					if (edges_of_junct.at(k).at(l)==e2){
						edges_of_junct.at(k).at(l)=e1;
					}
				}
			}
			//enhmerwsh table
			for (int ii=0;ii<table.size();ii++){
				for (int jj=0;jj<table.at(0).size();jj++){
					if (table.at(ii).at(jj)==e2){
						table.at(ii).at(jj)==e1;
					}
				}
			}


			//
		
			edges_t.at(e1).at(cl1)=edges_t.at(e2).at(o2);
			edges_t.at(e1).at(cl1+1)=edges_t.at(e2).at(o2+1);
			
			if ((cl1==2) && (cl2==0)){
				//metra posa stoixeia exei ontws
				int cc=0;
				while (detailed_edges.at(2*e1).at(cc)!=0){
					cc++;
				}
				//prostese ta stoixeia tou allou edge
				int cc2=0;
				while (detailed_edges.at(2*e2).at(cc2)!=0){
					detailed_edges.at(2*e1).at(cc+cc2)=detailed_edges.at(2*e2).at(cc2);
					detailed_edges.at(2*e1+1).at(cc+cc2)=detailed_edges.at(2*e2+1).at(cc2);
					cc2++;
				}
			}
			
			if ((cl1==2) && (cl2==2)){
				//metra posa stoixeia exei ontws
				int cc=0;
				while (detailed_edges.at(2*e1).at(cc)!=0){
					cc++;
				}
				//antesrepse th seira twn stoixeiwn ston pinaka ths 2hs akmhs
				vector<int> det(detailed_edges.at(2*e2).size(),0);
				vector<int> det2(detailed_edges.at(2*e2).size(),0);
				int cc2=0;
				for (int c=detailed_edges.at(2*e2).size()-1;c>=0;c--){
					if (detailed_edges.at(2*e2).at(c)!=0){
						det.at(cc2)=detailed_edges.at(2*e2).at(c);
						det2.at(cc2)=detailed_edges.at(2*e2+1).at(c);
						cc2++;
					}
				}
				//prosthese ta stoixeia ths 2hs akmhs
				for (int c=0;c<cc2;c++){
					detailed_edges.at(2*e1).at(cc+c)=det.at(c);
					detailed_edges.at(2*e1+1).at(cc+c)=det2.at(c);
				}
		
			}

			if ((cl1==0) && (cl2==0)){
				//antestrepse th seira twn stoixeiwn ston pinaka ths 2hs akmhs
				vector<int> det(detailed_edges.at(2*e2).size(),0);
				vector<int> det2(detailed_edges.at(2*e2).size(),0);
				int cc2=0;
				for (int c=detailed_edges.at(2*e2).size()-1;c>=0;c--){
					if (detailed_edges.at(2*e2).at(c)!=0){
						det.at(cc2)=detailed_edges.at(2*e2).at(c);
						det2.at(cc2)=detailed_edges.at(2*e2+1).at(c);
						cc2++;
					}
				}
				//topothethse ta stoixeia tou edge1 sto det
				int cc=0;
				while (detailed_edges.at(2*e1).at(cc)!=0){
					det.at(cc2+cc)=detailed_edges.at(2*e1).at(cc);
					det2.at(cc2+cc)=detailed_edges.at(2*e1+1).at(cc);
					cc++;
				}

				detailed_edges.at(2*e1)=det;
				detailed_edges.at(2*e1+1)=det2;
			}
			if ((cl1==0) && (cl2==2)){
				int cc=0;
				vector<int> det(detailed_edges.at(2*e1).size(),0);
				vector<int> det2(detailed_edges.at(2*e1).size(),0);
				while(detailed_edges.at(2*e2).at(cc)!=0){
					det.at(cc)=detailed_edges.at(2*e2).at(cc);
					det2.at(cc)=detailed_edges.at(2*e2+1).at(cc);
					cc++;
				}
				int cc2=0;
				while(detailed_edges.at(2*e1).at(cc2)!=0){
					det.at(cc+cc2)=detailed_edges.at(2*e1).at(cc2);
					det.at(cc+cc2)=detailed_edges.at(2*e1+1).at(cc2);
					cc2++;
				}
			}
			edges_t.at(e2).at(0)=-90;
			detailed_edges.at(2*e2).at(0)=-90;
			detailed_edges.at(2*e2+1).at(0)=-90;
		}
		
	}

	//edges_of_junct
	int c_j=0;
	for (int i=0;i<edges_of_junct.size();i++){
		if (edges_of_junct.at(i).at(0)!=-90){
			edges_of_junct.at(c_j)=edges_of_junct.at(i);
			junctions.at(c_j)=junctions.at(i);
			c_j++;
		}
	}
	junctions.resize(c_j);
	edges_of_junct.resize(c_j);
	//////////////////

	for (int i=0;i<junctions.size();i++){
		for (int k=-2;k<3;k++){
			for (int l=-2;l<3;l++){
				im.at<uchar>(junctions.at(i).at(1)+k,junctions.at(i).at(0)+l)=255;
			}
		}
	}
	//imshow("im6",im);

	negative(im);
	Mat img,img2;
	cvtColor(im, img, CV_GRAY2BGR);
	img.copyTo(img2);
	for (int i=0;i<edges_t.size();i++){
		if (edges_t.at(i).at(0)!=-90){
			colors r=show_colors(i);
			line(img, Point(edges_t.at(i).at(0),edges_t.at(i).at(1)), Point(edges_t.at(i).at(2),edges_t.at(i).at(3)), Scalar(r.c0,r.c1,r.c2), 1, CV_AA);
		}
	}
	
	//imshow("final2",img);


	for (int i=0;i<detailed_edges.size();i=i+2){
		if (detailed_edges.at(i).at(0)!=-90){
			colors r=show_colors(i);
			int ce=1;
		//	cout<<ce<<" "<<detailed_edges.at(i).at(ce);
			while ((detailed_edges.at(i).at(ce)!=0) && (ce<detailed_edges.at(0).size()-1)){
				line(img2, Point(detailed_edges.at(i).at(ce-1),detailed_edges.at(i+1).at(ce-1)), Point(detailed_edges.at(i).at(ce),detailed_edges.at(i+1).at(ce)), Scalar(r.c0,r.c1,r.c2), 1, CV_AA);
				
				//line(img2, Point(detailed_edges.at(i).at(ce-1),detailed_edges.at(i+1).at(ce-1)), Point(detailed_edges.at(i).at(ce),detailed_edges.at(i+1).at(ce)), Scalar(r.c0,r.c1,r.c2), 1, CV_AA);
				
				ce++;
				//cout<<ce<<" ";
			}
			//cout<<"ok";
		}
	}
	ret_j_ed ret;
	ret.detailed_edges=detailed_edges;
	ret.disconnected=disconnected;
	ret.edges_of_junct=edges_of_junct;
	ret.edges_t=edges_t;
	ret.junctions=junctions;
	ret.table=table;
	/*imshow("detailed_edges_final",img2);
	waitKey(0);*/
	return ret;
}
