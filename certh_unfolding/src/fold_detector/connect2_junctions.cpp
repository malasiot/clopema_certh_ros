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
struct ret_fol{
	Mat im;
	bool line;
	int winner;
};

struct ret_un_jun{
	vector<int> disconnected;
    vector<vector<int> > edges_of_junct;
    vector<vector<int> > junctions;
};
struct ret_j_ed{
    vector<vector<int> > junctions;
    vector<vector<int> > detailed_edges;
	vector<int> disconnected;
    vector<vector<int> > edges_of_junct;
    vector<vector<int> > edges_t;
    vector<vector<int> > table;
};

ret_fol follow(vector<vector<int> > , int ,Mat, vector<vector<int> > );
ret_un_jun unify_junctions(int ,int ,int ,vector<vector<int> > ,vector<int> ,vector<vector<int> > ,vector<vector<int> > );
ret_j_ed tiding_up(vector<vector<int> > ,vector<vector<int> > ,vector<int> ,vector<vector<int> > ,vector<vector<int> > ,vector<int>,Mat,vector<vector<int> >);
int calc_closest_edge_side(vector<int>,vector<int> );


ret_j_ed connect2_junctions(vector<vector<int> > junctions,vector<vector<int> > detailed_edges,vector<int> disconnected,vector<vector<int> > edges_of_junct,Mat im,vector<vector<int> > table,vector<vector<int> > edges_t){
	Mat im1,im2,im3;
	im.copyTo(im1);
	im.copyTo(im2);
	im.copyTo(im3);
	double tem1,tem2,mind=600;
	int ind_min=-1;
	ret_fol ret1,ret2;
	for (int i=0;i<disconnected.size();i++){
		
		if (disconnected.at(i)>=0){//exei brethei syndesh apo prohgoumeno disconnected
			
	
		//h pleyra me thn opoia syndeetai to junction "sbhnetai"
		int s=edges_of_junct.at(disconnected.at(i)).at(0);
		int count=0;
		while (detailed_edges.at(2*s).at(count)!=0){
			line(im1, Point(detailed_edges.at(2*s).at(count),detailed_edges.at(2*s+1).at(count)), Point(detailed_edges.at(2*s).at(count+1),detailed_edges.at(2*s+1).at(count+1)), 0, 3, CV_AA);
			count++;
		}
		//cout<<endl<<"i= "<<i<<endl;
		//////////////////////////////////////
		ret1=follow(junctions , disconnected.at(i), im2,table );
		
		ret2=follow(junctions , disconnected.at(i), ret1.im,table );
		
	/*	if (ret1.line==true && ret2.line==true){
			cout<<"true "<<endl;
		}
		else{
			cout<<"false"<<endl;
		}*/
		/*cout<<endl<<"winner1 "<<ret1.winner<<" winner2 "<<ret2.winner<<endl;
		waitKey(0);*/
		//////////////////////////
		Mat ims=Mat::zeros(19,19,CV_8UC1);
		Mat tem,im5;
		im.copyTo(tem);
		im.copyTo(im5);
		
		s=edges_of_junct.at(disconnected.at(i)).at(0);//h pleyra
		
		count=0;

		
		
		im.copyTo(im1);
		im.copyTo(im2);
		
		if (ret1.line==true && ret2.line==true){
			
			ret_un_jun r=unify_junctions(i,ret1.winner,ret2.winner,junctions,disconnected,edges_of_junct,edges_t);
		
			disconnected=r.disconnected;
			edges_of_junct=r.edges_of_junct;
			junctions=r.junctions;
		}
		
		}
		
	}
	
	for (int k=0;k<junctions.size();k++){
		for (int k1=-2;k1<3;k1++){
			for (int k2=-2;k2<3;k2++){
				im3.at<uchar>(junctions.at(k).at(1)+k1,junctions.at(k).at(0)+k2)=255;
			}
		}
	}
	
	//imshow("new results",im3);
	/////////////////////////////////
	//blepw an prepei na enwsw 2 akmes se 1
	//osa junction prepei na fygoyn kai na enwthoun oi grammes pou syndeoun
	//tha apothkeuontai me 1 sthn antistoixh thesh
	vector<int> connect(junctions.size(),0);

	//gia na tsekarei an enwnetai se 2 akmes (de ginetai
	//an einai disconnected h an syndeontai panw apo 2 akmes)
	bool check;
	for (int i=0;i<junctions.size();i++){
		
		//einai disconnected?
		check=true;
		for (int ii=0;ii<disconnected.size();ii++){
			if (disconnected.at(ii)==i){
				check=false;
			}
		}
		//sundeei 3 akmes?
		int c9=0;
		
		while ( (c9<edges_of_junct.at(0).size()) && (edges_of_junct.at(i).at(c9)!=-90) ){
			c9++;
			
		}
		
		if (c9!=2){
			check=false;
		}
		
		//syndeetai to katwtero shmeio ths mias akmhs me to
		//anwtero ths allhs?
		if (check==true){
			int e1,e2,o1,o2,cl1,cl2;
			e1=edges_of_junct.at(i).at(0);
			e2=edges_of_junct.at(i).at(1);
			cl1=calc_closest_edge_side(edges_t.at(e1),junctions.at(i));
			o1=0;
			if (cl1==0){
				o1=2;
			}
			//ta l1 kai l2 einai -1 otan to koino shmeio einai to mikrotero
			// kai 1 otan einai to megalytero apo ta 2 akra ths 1 akmhs
			int l1=1;
			if (edges_t.at(e1).at(cl1+1)<=edges_t.at(e1).at(o1+1)){
				l1=-1;
			}
			cl2=calc_closest_edge_side(edges_t.at(e2),junctions.at(i));
			o2=0;
			if (cl2==0){
				o2=2;
			}
			int l2=1;
			if (edges_t.at(e2).at(cl2+1)<edges_t.at(e2).at(o2+1)){
				l2=-1;
			}
			//gia na syndethoun prepei na enwthei ena katwtero me ena anwtero
			//akro ara prepei l1*l2=-1
			if (l1*l2==1){
				check=false;
			}
		}


		//an plhrountai oi proupotheseis gia enwsh akmwn
		
		if (check==true){
			
			int e1,e2;
			e1=edges_of_junct.at(i).at(0);
			e2=edges_of_junct.at(i).at(1);
				
			int par1,par2;
			double kl1,kl2;
			
			par1=edges_t.at(e1).at(1)-edges_t.at(e1).at(3);
			par2=edges_t.at(e2).at(1)-edges_t.at(e2).at(3);
		
			if ((par1!=0) && (par2!=0)){
				kl1=edges_t.at(e1).at(0)-edges_t.at(e1).at(2);
				kl1=kl1/par1;
				kl2=edges_t.at(e2).at(0)-edges_t.at(e2).at(2);
				kl2=kl2/par2;
			}
			else{
				par1=edges_t.at(e1).at(0)-edges_t.at(e1).at(2);
				par2=edges_t.at(e2).at(0)-edges_t.at(e2).at(2);
				if((par1!=0) && (par2!=0)){
					kl1=edges_t.at(e1).at(1)-edges_t.at(e1).at(3);
					kl1=kl1/par1;
					kl2=edges_t.at(e2).at(1)-edges_t.at(e2).at(3);
					kl2=kl2/par2;
				}
				else{
					check=false;
				}
			}//eyresh klisewn
			if (check==true){
				if (kl1*kl2>0){
				if (abs(kl1-kl2)<0.35){
				connect.at(i)=1;
				/*	Mat im4;
					im.copyTo(im4);
					cout<<"CONNECT THEM";
					for (int k=-2;k<3;k++){
						for (int l=-2;l<3;l++){
							im4.at<uchar>(junctions.at(i).at(1)+k,junctions.at(i).at(0)+l)=255;
						}
					}
					imshow("connection",im4);
					waitKey(0);*/
				}
			}
			}
			if (check==true && kl1*kl2<=0){
			//	cout<<"edww kl1";
				if (abs(kl1)+abs(kl2)<0.2){
			//		cout<<"edw kl2";
					connect.at(i)=1;
				}
			}
		}
		
	}
	//
	Mat im6,im7;
	im.copyTo(im6);

	//im.copyTo(im7);
	//cout<<"connect: "<<endl;
	//for (int k=0;k<connect.size();k++){
	//	cout<<connect.at(k)<<" ";
	//	if (connect.at(k)==1){
	//		for (int k1=-2;k1<3;k1++){
	//			for (int k2=-2;k2<3;k2++){
	//				im7.at<uchar>(junctions.at(k).at(1)+k1,junctions.at(k).at(0)+k2)=255;
	//			}
	//		}
	//	}
	//}
	//imshow("im7",im7);
	ret_j_ed ret;
	ret=tiding_up(junctions, detailed_edges, disconnected, edges_of_junct, edges_t,connect,im6,table);
	return ret;
}
