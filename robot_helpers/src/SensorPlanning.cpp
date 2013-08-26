#include "robot_helpers/Planner.h"
#include <Eigen/Geometry>
#include <vector>

#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_real.hpp>

using namespace Eigen ;
using namespace std ;

// geodesic sphere (from http://stackoverflow.com/questions/17705621/algorithm-for-a-geodesic-sphere)

static void subdivide(const Vector3d &v1, const Vector3d &v2, const Vector3d &v3, vector<Vector3d> &sphere_points,
                      const unsigned int depth) {
    if ( depth == 0 ) {
        sphere_points.push_back(v1);
        sphere_points.push_back(v2);
        sphere_points.push_back(v3);
        return;
    }

    const Vector3d v12 = (v1 + v2).normalized();
    const Vector3d v23 = (v2 + v3).normalized();
    const Vector3d v31 = (v3 + v1).normalized();

    subdivide(v1, v12, v31, sphere_points, depth - 1);
    subdivide(v2, v23, v12, sphere_points, depth - 1);
    subdivide(v3, v31, v23, sphere_points, depth - 1);
    subdivide(v12, v23, v31, sphere_points, depth - 1);
}

void make_geodesic_sphere(vector<Vector3d> &sphere_points, const unsigned int depth = 4)
{
    // create icosahedron

    const double X = 0.525731112119133606;
    const double Z = 0.850650808352039932;

    const Vector3d vdata[12] = {
        {-X, 0.0, Z}, { X, 0.0, Z }, { -X, 0.0, -Z }, { X, 0.0, -Z },
        { 0.0, Z, X }, { 0.0, Z, -X }, { 0.0, -Z, X }, { 0.0, -Z, -X },
        { Z, X, 0.0 }, { -Z, X, 0.0 }, { Z, -X, 0.0 }, { -Z, -X, 0.0 }
    };

    int tindices[20][3] = {
        {0, 4, 1}, { 0, 9, 4 }, { 9, 5, 4 }, { 4, 5, 8 }, { 4, 8, 1 },
        { 8, 10, 1 }, { 8, 3, 10 }, { 5, 3, 8 }, { 5, 2, 3 }, { 2, 7, 3 },
        { 7, 10, 3 }, { 7, 6, 10 }, { 7, 11, 6 }, { 11, 0, 6 }, { 0, 1, 6 },
        { 6, 1, 10 }, { 9, 0, 11 }, { 9, 11, 2 }, { 9, 2, 5 }, { 7, 2, 11 }
    };

    // subdivide to create final mesh

    for(int i = 0; i < 20; i++)
        subdivide(vdata[tindices[i][0]], vdata[tindices[i][1]], vdata[tindices[i][2]], sphere_points, depth);
}


////////////////////////////////////////////////////////////////////////////////

using namespace robot_helpers ;

class SensorGoalRegion: public GoalRegion {
public:
    SensorGoalRegion(const Vector3d &p_, const Vector3d &dir_, double radius):
        p(p_), dir(dir_), gen(time(0))
    {

    }

    void sample(std::vector<double> &xyz_rpy) ;

    Vector3d p, dir ;
    double radius ;

    boost::mt19937 gen;


};
