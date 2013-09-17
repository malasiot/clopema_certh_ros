#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <iostream>
//#include <string>

using namespace cv;
using namespace std;

struct a_corners{
	vector<int> certain_c;
	vector<int> str_l_c;
	vector<int> distant_c;
};

//store:dinei tvn arithmo twn a-corners pou entopisthkan se sygkekrimena shmeia gia kathe configuration
//location:dinei thn topothesia tvn a-corners, dhladh tis syntetagmenes tou junction sto opoio antistoixoun
bool group_a_corners(vector<int> a_corner,vector<vector<int> > junctions,vector<vector<int> >& store,vector<vector<Point> >& location,vector<vector<bool> >& current_corner,int i,int ind,int& k_stop){
		bool stop=false;
		
		//gia oles tis trexouses a-corners
		for (int j=0;j<a_corner.size() && a_corner.at(0)!=-1;j++){
			bool exists=false;
			//gia oles tis gwnies pou exoun brethei gia ayto to i (image) 
			
			for (int k=0;k<store.at(i).size();k++){
				
				if (store.at(i).at(0)==0){
					
					store.at(i).at(0)=1;
					Point p;
					p.x=junctions.at(a_corner.at(j)).at(0);
					p.y=junctions.at(a_corner.at(j)).at(1);
					location.at(i).at(0)=p;
					exists=true;
					if (i==ind){
						current_corner.at(i).at(0)=true;
					}
					else{
						current_corner.at(i).at(0)=false;
					}
				}
				else{
				
				double dist,temp1,temp2;
				temp1=(location.at(i).at(k).x-junctions.at(a_corner.at(j)).at(0));
				temp1=temp1*temp1;
				temp2=location.at(i).at(k).y-junctions.at(a_corner.at(j)).at(1);
				temp2=temp2*temp2;
				dist=sqrt(temp1+temp2);
				
					if (dist<30 && i!=ind){//20
						
						store.at(i).at(k)=store.at(i).at(k)+1;
						exists=true;
						//if there are more than 6 votes for a particular point
						//then the search can stop
						if (store.at(i).at(k)>6){
							k_stop=k;
							stop=true;
							break;
						}
						
					
					}
			
				}
				
			}
			
			if (exists==false){
				
				store.at(i).resize(store.at(i).size()+1);
				store.at(i).at(store.at(i).size()-1)=1;
				location.at(i).resize(location.at(i).size()+1);
				location.at(i).at(location.at(i).size()-1).x=junctions.at(a_corner.at(j)).at(0);
				location.at(i).at(location.at(i).size()-1).y=junctions.at(a_corner.at(j)).at(1);
				current_corner.at(i).resize(current_corner.at(i).size()+1);
				if (i==ind){
					current_corner.at(i).at(current_corner.at(i).size()-1)=true;
				}
				else{
					current_corner.at(i).at(current_corner.at(i).size()-1)=false;
				}
			}
			
		}
		return stop;
}





int choose_a_corner(a_corners a_corn,vector<vector<int> > junctions,int ind,vector<vector<int> >& store,vector<vector<Point> >& location,vector<vector<bool> >& current_corner,int& k_stop){
	int start_point;
	if (ind<10){start_point=0;}
	else{start_point=ind-10;}
	if (ind>0){
		store.resize(store.size()+1);
		vector<int> temp(1,0);
		Point p(0,0);
		vector<Point> P(1,p);
		store.at(store.size()-1)=temp;
		location.resize(location.size()+1);
		location.at(location.size()-1)=P;
		current_corner.resize(current_corner.size()+1);
		vector<bool> b(1,false);
		current_corner.at(current_corner.size()-1)=b;
	}
	
	//i_stop shows the image where more than 6 votes for a point are gathered
	//if there is no such a point yet, i_stop==-1
	int i_stop=-1;
	//cout<<"start point"<<start_point<<" ind "<<ind<<endl;
	for (int i=start_point;i<=ind;i++){
		//cout<<" ///// "<<endl;
		bool s1=group_a_corners(a_corn.certain_c,junctions,store,location,current_corner,i,ind,k_stop);
		//cout<<"done1";
		bool s2=group_a_corners(a_corn.distant_c,junctions,store,location,current_corner,i,ind,k_stop);
		//cout<<"done2";
		bool s3=group_a_corners(a_corn.str_l_c,junctions,store,location,current_corner,i,ind,k_stop);	
		//cout<<"done3";
		//cout<<endl<<" ///// "<<endl;
		if (s1==true || s2==true ||s3==true){
			i_stop=i;
			break;
		}

		
	}
	/*cout<<endl<<"--------"<<endl;
	for (int i=start_point;i<=ind;i++){
		for (int j=0;j<store.at(i).size();j++){
			cout<<store.at(i).at(j)<<" "; 
		}
		cout<<endl;
		
		for (int j=0;j<current_corner.at(i).size();j++){
			cout<<current_corner.at(i).at(j)<<" "; 
		}
		cout<<endl;
	}
	
	cout<<"---------"<<endl;*/
	return i_stop;
}
