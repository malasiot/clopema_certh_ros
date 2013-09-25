#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "FoldDetector.h"
#include <fstream>
#include <iostream>
//#include <string>

using namespace cv;
using namespace std;

//struct a_corners{
//	vector<int> certain_c;
//	vector<int> str_l_c;
//	vector<int> distant_c;
//};
//
//struct ret_all{
//	vector<vector<int>> junctions;
//	vector<vector<int>> detailed_edges;
//	vector<int> disconnected;
//	vector<vector<int>> edges_of_junct;
//	vector<vector<int>> edges_t;
//	vector<vector<int>> table;
//	a_corners a_corn;
//};


struct colors{
	
	int c0;
	int c1;
	int c2;

};

ret_all call_main(Mat,Mat);
int choose_a_corner(a_corners, depths ,vector<vector<int> > ,int ,vector<vector<int> >& ,vector<vector<Point> >&,vector<vector<bool> >& ,vector<vector <bool> > & , vector<vector<float> >& ,int& );


bool folds::fold_detector(Mat bgrImage, Mat depthMap, int th, vector<double>& grasp_candidate, vector<vector<int> >& store, vector<vector<Point> >& location, vector<vector<bool> >& current_corner,  vector<vector <bool> > & side, vector<vector<float> >& depthD, int cx ){
	bool ret;
	int i_stop=-1,k_stop=-1;

	ret_all r=call_main( bgrImage, depthMap );
	
		
		//i_stop shows the image where more than 6 votes for a point are gathered
		//if there is no such a point yet, i_stop==-1
		
		i_stop=choose_a_corner(r.a_corn,r.d, r.junctions,th,store,location,current_corner, side, depthD,k_stop);
		
		cout<<"!";
		if (i_stop!=-1 && location.at(i_stop).at(k_stop).x<cx && current_corner.at(i_stop).at(k_stop)==true && depthD.at(i_stop).at(k_stop)>3000){
			
			ret=true;
			grasp_candidate.at(0)=i_stop;
			grasp_candidate.at(1)=location.at(i_stop).at(k_stop).x;
			grasp_candidate.at(2)=location.at(i_stop).at(k_stop).y;
			cout<<"COORDINATES "<<grasp_candidate.at(1)<<" "<<grasp_candidate.at(2)<<endl;
			cout<<"i_stop "<<i_stop<<endl;
		}
		else{
			ret=false;
			
		}
		
		return ret;
}
