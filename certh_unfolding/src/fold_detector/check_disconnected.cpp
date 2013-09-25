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

struct depths{
	vector <float> difd_c;
	vector <float> difd_s;
	vector <float> difd_d;
	vector<bool> side_c;
	vector<bool> side_s;
	vector<bool> side_d;

};

int calc_closest_edge_side(vector<int> ,vector<int>);
corner_depths a_corner_depth(Mat ,Mat ,vector<int> );
bool check_depths(float,corner_depths);

vector<int> check_disconnected(vector<vector<int> > junctions,vector<vector<int> > edges,vector<vector<int> >& edges_of_junct,Mat im,Mat imd,vector<vector<int> > table, depths& d){
	Mat im1;
	im.copyTo(im1);
	vector<int> connected;
	bool found1=false;//oti brhke estw 1 a-corner
	//bres ta disconnected
	
	for (int i=0;i<edges_of_junct.size();i++){
		if (edges_of_junct.at(i).at(1)==-90 && edges_of_junct.at(i).at(0)>=0){

			vector<int> keep(2);
			int count=0;

			//des an uparxei junction konta
			for (int l=0;l<junctions.size();l++){
				if (edges_of_junct.at(l).at(1)!=-90 && edges_of_junct.at(l).at(2)==-90){
					if (junctions.at(i).at(0)-10<junctions.at(l).at(0) && junctions.at(i).at(0)+10>junctions.at(l).at(0)){
						if (junctions.at(i).at(1)-10<junctions.at(l).at(1) && junctions.at(i).at(1)+10>junctions.at(l).at(1)){
							//elegxw an exei h mia akmh einai panw kai h allh katw apo to junction
							int c1=calc_closest_edge_side(edges.at(edges_of_junct.at(l).at(0)),junctions.at(l));
							int o1=0;
							if (c1==0){
								o1=2;
							}
							int c2=calc_closest_edge_side(edges.at(edges_of_junct.at(l).at(1)),junctions.at(l));
							int o2=0;
							if (c2==0){
								o2=2;
							}
							int t1,t2,t3=junctions.at(l).at(1);
							t1=edges.at(edges_of_junct.at(l).at(0)).at(o1+1);
							t2=edges.at(edges_of_junct.at(l).at(1)).at(o2+1);
							//elegxw an h mia akmh apo to kontino junction einai panw kai h allh
							//katw apo to junction
							if ((t1>t3 && t2<t3) || (t1<t3 && t2>t3)){
							//	cout<<"candidate";
								bool same_semiplane=false;

								for (int o1=-3;o1<4;o1++){
									for(int o2=-3;o2<4;o2++){
										im1.at<uchar>(junctions.at(l).at(1)+o1,junctions.at(l).at(0)+o2)=255;
									}
								}
							/*	namedWindow("possible connections",0);
								imshow("possible connections",im1);
								waitKey(0);*/
							
								//bres poia apo tis 2 paei pio pshla
								int up_ed,d_ed,up_o,d_o;
								if (t1<t2){
									up_o=o1;
									up_ed=edges_of_junct.at(l).at(0);
									d_o=o2;
									d_ed=edges_of_junct.at(l).at(1);
								}
								else{
									up_o=o2;
									up_ed=edges_of_junct.at(l).at(1);
									d_o=o1;
									d_ed=edges_of_junct.at(l).at(0);
								}

							
								

							//elegxw an briskontai sto idio hmiepipedo kai oi 3 akmes
							//briskw thn eksiswsh ths eytheias ths "anwterhs" akmhs
							int par=junctions.at(l).at(0)-edges.at(up_ed).at(up_o);
							int ar=junctions.at(l).at(1)-edges.at(up_ed).at(up_o+1);
							float av,bv;
							if (par!=0){
								av=(float)ar/par;
								bv=(float)(-av*edges.at(up_ed).at(up_o)+edges.at(up_ed).at(up_o+1));
							}

									//an einai entelws katheth
							vector<int> side(2);
							int i_cl=calc_closest_edge_side(edges.at(edges_of_junct.at(i).at(0)),junctions.at(i));
							int i_o=0;
							if(i_cl==0){
								i_o=2;
							}
							if (par==0){
								//an ta x einai apo thn idia pleura tote 
								//anhkoun sto idio hmiepipedo
								
								side.at(0)=(float)(junctions.at(l).at(0)-edges.at(d_ed).at(d_o));
								side.at(1)=(float)(junctions.at(l).at(0)-edges.at(edges_of_junct.at(l).at(0)).at(i_o));	
							
								//idio hmiepipedo
			
								if (side.at(0)*side.at(1)>0){
									same_semiplane=true;
									//cout<<"same semiplane";
								}
								else{
									//cout<<"different semiplane";
								}
							}
							
							else{
								 //briskw to x pou antistoixei sthn pio "katheth" eytheia gia to y 
								//ths akrhs pou den antistoixei sto junction ths allhs eutheias 
				
								  int tt=edges.at(d_ed).at(d_o+1);
								 side.at(0)=(float)((tt-bv)/av)-(float)(edges.at(d_ed).at(d_o));

								 tt=edges.at(edges_of_junct.at(i).at(0)).at(i_o+1);
								 side.at(1)=(float)((tt-bv)/av)-(float)(edges.at(edges_of_junct.at(i).at(0)).at(i_o));

								 if (side.at(0)*side.at(1)>0){
									 same_semiplane=true;
									// cout<<"same semiplane";
								 }
								 else{
									//cout<<"different semiplane";
								}
							}
							if (same_semiplane==true){
								corner_depths r=a_corner_depth(im,imd,junctions.at(l));
								//elegxw pws einai moirasmena ta barh
								bool ok;
								//elegxw an epistrefei <3 epipeda h an yparxei asyndeth akmh
								if (r.layers3==false){
									ok=false;
								}
								else{
									ok=check_depths(side.at(0),r);
								}
								if (ok==true){
									////////////////////////<-----------------------------------------------------------
								/*	cout<<"???????"<<endl;
									cout<<"EDW2"<<endl;
									edges_of_junct.at(i).at(1)=up_ed;
									edges_of_junct.at(i).at(2)=d_ed;
									cout<<"EDGES OF JUNCT "<<edges_of_junct.at(i).at(0)<<" "<<edges_of_junct.at(i).at(1)<<" "<<edges_of_junct.at(i).at(2)<<endl;*/
									
									/////////////////////////////////
									found1=true;
									connected.push_back(l);
									d.difd_d.push_back(abs(r.average.at(2)-r.average.at(0)));
									if (side.at(0)>0){
										d.side_d.push_back(true);
									}
									else{
										d.side_d.push_back(false);
									}

									//cout<<"a corner"<<endl;
									edges_of_junct.at(l).at(2)=edges_of_junct.at(i).at(0);
									//edges_of_junct.at(i).at(1)=edges_of_junct.at(l).at(0);
									//edges_of_junct.at(i).at(2)=edges_of_junct.at(l).at(1);
								}
							}
							}//if candidates

						}
					}
				}
			}
	



			
			////an yparxoun2 akmes des an enwnontai se junction
			//if (count==2){

			//}
		}
	}
////////////////elegxw gia a-corner 

	for (int i=0;i<edges_of_junct.size();i++){
		if (edges_of_junct.at(i).at(1)==-90 && edges_of_junct.at(i).at(0)>=0){
			//elegxw an h akmh phgainei apo to junction pros ta panw
			int cll=calc_closest_edge_side(edges.at(edges_of_junct.at(i).at(0)),junctions.at(i));
			int ol=0;
			if (cll==0){
				ol=2;
			}
			if (edges.at(edges_of_junct.at(i).at(0)).at(ol+1)<junctions.at(i).at(1)){
			//elegxw an uparxei allh grammh sthn perioxh ektos apo thn disconnected
			int count=0,other_l=-1;
			bool stop=false;
				for (int k=junctions.at(i).at(0)-9;k<junctions.at(i).at(0)+9 && stop==false;k++){
					for (int l=junctions.at(i).at(1)-9;l<junctions.at(i).at(1)+9;l++){
						if (table.at(l).at(k)!=-2 && table.at(l).at(k)!=edges_of_junct.at(i).at(0)){
							
							if (other_l==-1){
								other_l=table.at(l).at(k);
							}
							else{
								if (other_l==table.at(l).at(k)){
									count++;
								}
								else{
									stop=true;
									break;
								}
							}
						}
					}
				}
			//	cout<<"count= "<<count<<" other_l= "<<other_l<<endl;
				//an yparxei mono 1 akoma akmh 
				if (other_l!=-1 && count>14 && stop==false){
					
					//elegjkse thn klish ths eytheias  
						//upologizw thn klish ths eutheias
					float f1;
					int t1=edges.at(other_l).at(2)-edges.at(other_l).at(0);
					int t2=edges.at(other_l).at(3)-edges.at(other_l).at(1);
					if (t1==0){
						f1=500;
					}
					else{
						f1=abs(atan(float(t2/t1)));
					}
					if (f1>3.14/4){
							for (int o1=-3;o1<4;o1++){
									for(int o2=-3;o2<4;o2++){
									im1.at<uchar>(junctions.at(i).at(1)+o1,junctions.at(i).at(0)+o2)=255;
								}
							}
					/*	namedWindow("possible connections",0);
						imshow("possible connections",im1);
						waitKey(0);*/


						//upologise se poio hmiepipedo brisketai h mesaia akmh ths gwnias
						//bres thn eksiswsh ths "anwterhs" eytheias (h eytheia pou apoteleitai apo tis alles 2)
						float sidet;
						int par=edges.at(other_l).at(0)-edges.at(other_l).at(2);
						int ar=edges.at(other_l).at(1)-edges.at(other_l).at(3);
						float av,bv;
						if (par!=0){
							av=(float)ar/par;
							bv=(float)(-av*edges.at(other_l).at(2)+edges.at(other_l).at(3));
						}
						//an einai entelws katheth
						if (par==0){
							//an ta x einai apo thn idia pleura tote 
							//anhkoun sto idio hmiepipedo
							int c=0;
								sidet=(float)(junctions.at(i).at(0)-edges.at(edges_of_junct.at(i).at(0)).at(ol));
					
							}
						else{
							 //briskw to x pou antistoixei sthn pio "katheth" eytheia gia to y 
							 //ths akrhs pou den antistoixei sto junction ths allhs eutheias 
							 int tt=edges.at(edges_of_junct.at(i).at(0)).at(ol+1);
							 sidet=(float)((tt-bv)/av)-(float)(edges.at(edges_of_junct.at(i).at(0)).at(ol));
						}
						//////
						//elegxw to bathos
						//metakinw ligo to kentro ths eikonas apo thn opoia pairnw
						//ta bathh giati epeidh se aythn thn periptwsh yparxei apostash 
						//ta epipeda alliws de tha fainontai kai ta 3 kala
						vector<int> d_center(2);
						d_center.at(1)=junctions.at(i).at(1);
						if (sidet<0){
							d_center.at(0)=junctions.at(i).at(0)-2;
						}
						else{
							d_center.at(0)=junctions.at(i).at(0)+2;
						}
						
						corner_depths rt=a_corner_depth(im,imd,d_center);
						//elegxw pws einai moirasmena ta bathh
						bool ok;
						if (rt.layers3==false){
							ok=false;
						}
						else{
							ok=check_depths(sidet,rt);
						}
						
						if (ok==true){
							found1=true;
							connected.push_back(i);
							/////
							d.difd_d.push_back(abs(rt.average.at(2)-rt.average.at(0)));
							if (sidet>0){
								d.side_d.push_back(true);
							}
							else{
								d.side_d.push_back(false);
							}
							////////////////////////<-----------------------------------------------------------
							edges_of_junct.at(i).at(1)=other_l;
							edges_of_junct.at(i).at(2)=other_l;
							//cout<<" EDGES OF JUNCT 1 "<<edges_of_junct.at(i).at(1)<< " "<<edges_of_junct.at(i).at(2)<<endl;
							/////////////////////////////////
							for (int o1=-3;o1<4;o1++){
									for(int o2=-3;o2<4;o2++){
									im1.at<uchar>(junctions.at(i).at(1)+o1,junctions.at(i).at(0)+o2)=255;
								}
							}
						}
					}
				}
			/*	namedWindow("possible connections",0);
				imshow("possible connections",im1);
				waitKey(0);*/
			}
		}
	}
	
	if (found1==false){
		connected.push_back(-1);
	}
	return connected;
}
