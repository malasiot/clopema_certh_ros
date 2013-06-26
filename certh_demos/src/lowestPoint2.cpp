#include <cv.h>

#include <Eigen/Core>
#include <Eigen/Eigenvalues> // for cwise access

   
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef Eigen::Vector3d Vec3 ;
typedef Eigen::Matrix3d Matrix3 ;

using namespace std ;



/**
 * @param x coordinates of point to be tested
 * @param t coordinates of apex point of cone
 * @param b coordinates of center of basement circle
 * @param aperture in radians
 */
static bool pointInsideCone(const Vec3 &x, const Vec3 &apex, const Vec3 &base, float
aperture)
{
    // This is for our convenience
    float halfAperture = aperture/2.f;

    // Vector pointing to X point from apex
    Vec3 apexToXVect = apex - x ;
    apexToXVect.normalize() ;

    // Vector pointing from apex to circle-center point.
    Vec3 axisVect = apex - base;
    axisVect.normalize() ;

    // determine angle between apexToXVect and axis.

    double d = apexToXVect.dot(axisVect) ;

    bool isInInfiniteCone = fabs(d) > cos(halfAperture) ;

    if(!isInInfiniteCone) return false;

    // X is contained in cone only if projection of apexToXVect to axis
    // is shorter than axis.
    // We'll use dotProd() to figure projection length.

    bool isUnderRoundCap = apexToXVect.dot(axisVect)  < 1 ;

    return isUnderRoundCap;
}


static void robustPlane3DFit(vector<Vec3> &x, Vec3  &c, Vec3 &u)
{
    int i, k, N = x.size() ;
    Vec3 u1, u2, u3 ;
    const int NITER = 5 ;

    double *weight = new double [N] ;
    double *res = new double [N] ;

    for( i=0 ; i<N ; i++ ) weight[i] = 1.0 ;

    double wsum = N ;

    for( k=0 ; k<NITER ; k++ )
    {

        c = Vec3::Zero() ;
        Matrix3 cov = Matrix3::Zero();

        for( i=0 ; i<N ; i++ )
        {
            const Vec3 &P = x[i] ;
            double w = weight[i] ;

            c += w * P ;
        }

        c /= wsum ;

        for( i=0 ; i<N ; i++ )
        {
            const Vec3 &P = x[i] ;
            double w = weight[i] ;

            cov += w *  (P - c) * (P - c).adjoint();

        }

        cov *= 1.0/wsum ;

        Matrix3 U ;
        Vec3 L ;

        Eigen::SelfAdjointEigenSolver<Matrix3> eigensolver(cov);
        L = eigensolver.eigenvalues() ;
        U = eigensolver.eigenvectors() ;

        // Recompute weights

        u1 = U.col(0) ;
        u2 = U.col(1) ;
        u3 = U.col(2) ;

        for( i=0 ; i<N ; i++ )
        {
            const Vec3 &P = x[i] ;

        //    double r = (P -c).dot(P-c) ;
        //    double ss = (P-c).dot(u1);

         //   r -= ss * ss ;

          //  r = sqrt(r) ;

            double r = fabs((P-c).dot(u1));

            weight[i] = r ;
            res[i] = r ;
        }

        // Estimate residual variance
        double sigma ;

        sort(weight, weight + N) ;

        sigma = weight[N/2]/0.6745 ;

        // Update weights using Hubers scheme

        wsum = 0.0 ;

        for( i=0 ; i<N ; i++ )
        {

            double r = fabs(res[i]) ;

            if ( r <= sigma )  weight[i] = 1.0 ;
            else if ( r > sigma && r <= 3.0*sigma ) weight[i] = sigma/r ;
            else weight[i] = 0.0 ;

            wsum += weight[i] ;
        }
    }

    u = u1 ;

    delete weight ;
    delete res ;
}

static Vec3 computeNormal(const PointCloud &pc, int x, int y)
{
    const int nrmMaskSize = 6 ;
    int w = pc.width, h = pc.height ;

    vector<Vec3> pts ;

    for(int i = y - nrmMaskSize ; i<= y + nrmMaskSize ; i++  )
        for(int j = x - nrmMaskSize ; j<= x + nrmMaskSize ; j++  )
        {
            if ( i < 0 || j < 0 || i > h-1 || j > w-1 ) continue ;

            PointT val = pc.at(j, i) ;

            if ( !pcl_isfinite(val.z) ) continue ;

            pts.push_back(Vec3(val.x, val.y, val.z)) ;
        }

    if ( pts.size() < 3 ) return Vec3() ;

    Vec3 u, c ;
    robustPlane3DFit(pts, c, u);

    if ( u(2) < 0 ) u = -u ;

    return u ;

}



bool findLowestPoint2(const PointCloud &depth, const Vec3 &orig, const Vec3 &base,
    float apperture, Vec3 &p, Vec3 &n)
{
    float max_length = (base - orig).norm() ;
    float max_openning = apperture ;

    int w = depth.width, h = depth.height ;

    int max_y = -1 ;
    int best_j = -1, best_i = -1 ;

    bool found = false ;

    double minv = DBL_MAX ;

    for(int j=0 ; j<w ; j++ )
    {

        for(int i=0 ; i<h ; i++)
        {
            PointT val = depth.at(j, i) ;

            if ( !pcl_isfinite(val.z) ) continue ;

            Eigen::Vector3d p(val.x, val.y, val.z) ;

            if ( !pointInsideCone(p, orig, base, apperture) ) continue ;

            double s = (p - orig).dot(base - orig) ;

            if ( s < minv )
            {
                minv = s ;
                best_j = j ;
                best_i = i ;
                found = true ;
            }

        }
    }

    if ( !found ) return false ;

    PointT p_ = depth.at(best_j, best_i) ;

    n = computeNormal(depth, best_j, best_i) ;
    p = Vec3(p_.x, p_.y, p_.z) ;
}

/*

bool findLowestPoint(const PointCloud &depth, const Vec3 &orig, const Vec3 &base,
float apperture,
                     Vec3 &p, Vec3 &n )
{


    int w = depth.width, h = depth.height ;
    cout<< "w= "<< w << " h= "<< h<< endl;
    int best_i=-1, best_j=-1;
    float best_x=-1, best_y=-1, best_z=-1;


    bool found = false ;
    float minx = 10 ;
    for(int j=66 ; j<626 ; j++ )
    {


        for(int i=100 ; i<350 ; i++)
        {
            PointT val = depth.at(j, i) ;

            if ( val.z<1 || val.z>1.5 ) continue ;

            if ( minx>val.x )
            {
                minx = val.x ;
              //  cout<<"minx= " << minx<< endl;
                best_j = j ;
                best_i = i ;
                best_x=val.x;
                best_y=val.y;
                best_z=val.z;

                found = true ;
            }

        }
    }

    cout<< "best= (" << best_j <<" , "<< best_i <<")"<<endl;

    cout<< "best point = "<< best_x <<" "<< best_y << " "<< best_z << endl;


    PointT p_ = depth.at(best_j, best_i) ;


    p = Vec3(p_.x, p_.y, p_.z) ;
    if ( !found ) return false ;



}

*/
