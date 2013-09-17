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
struct ret_un_jun{
	vector<int> disconnected;
    vector<vector<int> > edges_of_junct;
    vector<vector<int> > junctions;
};

ret_un_jun unify_junctions(int jun,int win1,int win2,vector<vector<int> > junctions,vector<int> disconnected,vector<vector<int> > edges_of_junct,vector<vector<int> > edges_t){
	//h akmh tou junction pou theloume na sundesoume
	bool flag=false;//an yparxei h oxi syndesh
	
	int c_edge=edges_of_junct.at(disconnected.at(jun)).at(0);
	
	int o_edge;//the other edge found to be connected to the junction
	if ((c_edge!=win1) && (win1!=-2)){
		o_edge=win1;
		flag=true;
	}
	else{
		if ((c_edge!=win2)  && (win2!=-2)){
			o_edge=win2;
			flag=true;
		}//else tha parameinei asyndeto ws exei

	}
	
	
	//an uparxei syndesh
	//bres to plhsiestero junction ths allhs akmhs sto asundeto junction
	if (flag==true){
		
	//bres ta 2 akra ths allhs akmhs me thn opoia syndeetai
	vector<int> tem(2,-100);//apothhkeuw ta 2 akra ths allhs akmhs
	int count=0;
	for (int i=0;i<edges_of_junct.size();i++){
		for (int j=0;j<edges_of_junct.at(0).size();j++){
			if (edges_of_junct.at(i).at(j)==o_edge){
				//gia thn periptvsh pou exei enwthei ena junction sthn mesh ths akmhs
				bool leave_it=false;
				if ((j==1) && (edges_of_junct.at(i).at(j)==edges_of_junct.at(i).at(j+1))){
					leave_it=true;
					j=j+1;
				}
				if (leave_it==false){
					tem.at(count)=i;
					count++;
				}
			}
		}
	}
	
	//upologise thn apostash apo to kontinotero junction
	double tem1,tem2,dist;
	int o_junction;
	if (count!=0){
		o_junction=tem.at(0);//the other junction
	tem1=(junctions.at(tem.at(0)).at(0)-junctions.at(disconnected.at(jun)).at(0));
	tem1=tem1*tem1;
	tem2=(junctions.at(tem.at(0)).at(1)-junctions.at(disconnected.at(jun)).at(1));
	tem2=tem2*tem2;
	dist=sqrt(tem1+tem2);
	
	if(count==2){
		tem1=(junctions.at(tem.at(1)).at(0)-junctions.at(disconnected.at(jun)).at(0));
		tem1=tem1*tem1;
		tem2=(junctions.at(tem.at(1)).at(1)-junctions.at(disconnected.at(jun)).at(1));
		tem2=tem2*tem2;
		tem1=sqrt(tem1+tem2);
		if (tem1<dist){
			o_junction=tem.at(1);
			dist=tem1;
		}
	}
	
	//cout<<"the c_edge is: "<<c_edge<<endl;
	
		//cout<<"connect "<<c_edge<<" with "<<o_edge<<endl;
		//cout<<endl<<"dist= "<<dist<<endl;
		//enhmerwsh tvn disconnected kai edges_of_junct
		if (dist<10){
			
		edges_of_junct.at(disconnected.at(jun)).at(0)=-90;
		int c=0;
		
		while ((c<edges_of_junct.at(o_junction).size()) && (edges_of_junct.at(o_junction).at(c)!=-90)){
			c++;
		}
		
		edges_of_junct.at(o_junction).at(c)=c_edge;
		//bazw to junction na brisketai sto shmeio tou allou junction (o_junction)
		junctions.at(o_junction)=junctions.at(disconnected.at(jun));
		//an to junction sundethhke bale sth thesh tou -1
		disconnected.at(jun)=-1;
		//des an kai to allo junction einai asyndeto
		for (int i=0;i<disconnected.size();i++){
			if (disconnected.at(i)==o_junction){
				disconnected.at(i)=-1;
			}
		}
	
		}
		else{
			//sthn periptwsh pou to disconnected edge syndeetai alla
			//oxi me junction alla panw sthn akmh bazw sto edges_of_junct
			//2 fores th syndesh me thn allh akmh gia na deiksw oti ksekinan 
			//3 akmes
			edges_of_junct.at(disconnected.at(jun)).at(1)=o_edge;
			edges_of_junct.at(disconnected.at(jun)).at(2)=o_edge;
			disconnected.at(jun)=-1;
		}
	}//if (count!=0)
	}//if (flag==true),an yparxei syndesh
	
	ret_un_jun ret;
	ret.disconnected=disconnected;
	ret.edges_of_junct=edges_of_junct;
	ret.junctions=junctions;
	return ret;
}
