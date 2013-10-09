#pragma once
#ifndef FoldDetector
#define FoldDetector
#include <Eigen/Geometry>

#include <vector>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;


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
extern bool grasp_point(bool  , vector<double>& , vector<Eigen::Matrix4d>&  , vector<vector<int> >&  , vector<vector<Point> >&  , vector<vector<bool> >&  , vector<vector<bool> >& ,vector<vector<float> >& ,  int ,bool &, vector<vector< int> > &, vector<vector <Point> > &  );

extern int normalPoint(int , ret_all , bool , Mat , Point & );
extern bool detectHorizontalEdge( vector<double>& , int,int );

//these functions are used for the detection of the corners that are created from a fold of the cloth(arrow junctions)
/////call_main ///
//this function detects the edges and junctions of the hanging cloth
//inputs: an rgb and a depth image of the hanging cloth
// outputs: junctions=the junctions detected on the cloth, detailed_edges: the edges detected on the cloth-each edge is given in detailed
//using small straight edges, edges_of_junct: the edges that end up to each junction, edges_t: the coordinates of the two end points of the 
//edges, a_corn: the numbers that correspond to the junctions that constitute the lowest points of corners.
/////fold_detector///
//This function detects the corners that imply the existance of a fold.  It is called repeatedly so that the best candidate for grasping is found.
//For all the candidate corners of each picture there is a score that shows how many pictures vote for that particular corner.
//inputs:rgb image,depth image, the rotation angle of the gripper holding the cloth, a vector(&) with 3 elements 
//that will contain the rotation angle(of the gripper) and the coordinates of the point that is suggested as a grasping point(informal output),
//a vector with all the rotation angles that "call" the function(informal output), a vector<vector> with all the scores of each point for each picture(informal output),
// a vector <vector> with the corresponding coordinates for the candidates whose score is shown by the previous vector(informal output), a vector<vector> that 
//says if the former points belong or not to the current picture(informal output), an int that shows the x coordinate of the gripper on the image
//output:a boolean saying whether a good candidate is found
class folds{

public:

    folds() {
        score.resize(1) ;
        score.back().push_back(0) ;

        location.resize(1) ;
        location.back().push_back(Point(0, 0)) ;

        current_corner.resize(1) ;
        current_corner.back().push_back(false) ;

        sider.resize(1);
        sider.back().push_back(false);

        depthD.resize(1);
        depthD.back().push_back(0.0);

        radius.resize(1);//<------------------------------------
        radius.back().push_back(0);

        Points.resize(1);
        Points.back().push_back(Point(0,0));//<------------------------


    }

	ret_all call_main(Mat,Mat);

    bool fold_detector(Mat, Mat, int, vector<double>& , vector<vector<int> > &, vector<vector<Point> >& , vector<vector<bool> >& ,vector<vector<bool> >& ,vector<vector<float> >& , int ,vector<vector< int> > & , vector<vector <Point> > &, bool&);

    bool detect(const Mat &clr, const Mat &depth, int frame, vector<double> &graspCandidates, int cx, bool &orientLeft ) {
        bool res =  fold_detector(clr, depth, frame, graspCandidates, score, location, current_corner, sider, depthD, cx, radius, Points, orientLeft) ;

    return res ;
    }

    bool select(bool detected, vector<double> &grasp_candidate, vector<Eigen::Matrix4d> &orientations, int cx, bool & orientLeft)
    {

        if ( grasp_point (detected , grasp_candidate, orientations ,  score , location , current_corner , sider, depthD, cx, orientLeft,radius, Points));

            return true;

        return false;
    }

    vector<vector<int> > score ;
    vector<vector<Point> > location ;
    vector<vector<bool> > current_corner ;
    vector<vector<bool> > sider;
    vector<vector<float> > depthD;
    vector<vector< int> > radius;
    vector<vector <Point> > Points;

};

#endif
