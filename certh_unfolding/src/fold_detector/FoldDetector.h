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

struct ret_all{
    vector<vector<int> > junctions;
    vector<vector<int> > detailed_edges;
	vector<int> disconnected;
    vector<vector<int> > edges_of_junct;
    vector<vector<int> > edges_t;
    vector<vector<int> > table;
	a_corners a_corn;
};
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
	ret_all call_main(Mat,Mat);

    bool fold_detector(Mat, Mat, int, vector<double>& , vector<vector<int> > &, vector<vector<Point> >& , vector<vector<bool> >& ,int );


};

#endif
