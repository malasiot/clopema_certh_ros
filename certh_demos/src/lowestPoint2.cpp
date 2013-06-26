#include <cv.h>

#include <Eigen/Core>
#include <Eigen/Eigenvalues> // for cwise access

   
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>


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
static bool pointInsideCone(const Vec3 &x, const Vec3 &apex, const Vec3 &base, float aperture)
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
    const int NITER = 1 ;

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

    if ( u(2) > 0 ) u = -u ;

    return u ;

}

// Mean Shift algorithm for point clouds

void findMeanShiftPoint(const PointCloud &depth, int x0, int y0, int &x1, int &y1, double radius, double variance = 1.0e-3, int maxIter = 10)
{
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

    boost::shared_ptr<PointCloud> cloud(new PointCloud(depth)) ;

    kdtree.setInputCloud (cloud);

    int w = depth.width, h = depth.height ;

    PointT p = depth.at(x0, y0) ;
    Vec3 center(p.x, p.y, p.z) ;

    int iter = 0 ;
    double centerDist ;

    do
    {

        Vec3 newCenter(0, 0, 0) ;

        vector<int> pointIdxRadiusSearch;
        vector<float> pointRadiusSquaredDistance;

        pcl::PointXYZ center_(center.x(), center.y(), center.z()) ;
        kdtree.radiusSearch (center_, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) ;

        int n = pointIdxRadiusSearch.size() ;

        double denom = 0.0 ;

        for(int i=0 ; i<n ; i++ )
        {
            PointT p = depth.at(pointIdxRadiusSearch[i]) ;
            Vec3 pc(p.x, p.y, p.z) ;

            double ep = exp(-pointRadiusSquaredDistance[i]/ (2.0 * variance));

            denom += ep ;

            newCenter += ep * pc ;
        }

        newCenter /= denom ;

        centerDist = (newCenter - center).norm() ;

        center = newCenter ;

        ++iter ;

        //cout << centerDist << endl ;

    } while ( centerDist > variance && iter < maxIter ) ;

    vector<int> pointIdxNNSearch;
    vector<float> pointNNSquaredDistance;

    pcl::PointXYZ center_(center.x(), center.y(), center.z()) ;
    if ( kdtree.nearestKSearch(center_, 1, pointIdxNNSearch, pointNNSquaredDistance) > 0 )
    {
        int idx = pointIdxNNSearch[0] ;

        y1 = idx / w ;
        x1 =  idx - y1 * w ;

    }
}

bool findLowestPoint2(const PointCloud &depth, const Vec3 &orig, const Vec3 &base,
    float apperture, Vec3 &p, Vec3 &n)
{

    int w = depth.width, h = depth.height ;

    int max_y = -1 ;
    int best_j = -1, best_i = -1 ;

    bool found = false ;

    double maxv = -DBL_MAX ;

    for(int j=0 ; j<w ; j++ )
    {

        for(int i=0 ; i<h ; i++)
        {
            PointT val = depth.at(j, i) ;

            if ( !pcl_isfinite(val.z) ) continue ;

            Eigen::Vector3d p(val.x, val.y, val.z) ;

            // test whether the point lies within the cone

            if ( !pointInsideCone(p, orig, base, apperture) ) continue ;

            // project the point on the cone axis to determine how low it is.

            double s = (p - orig).dot(base - orig) ;

            if ( s > maxv )
            {
                maxv = s ;
                best_j = j ;
                best_i = i ;
                found = true ;
            }

        }
    }

    if ( !found ) return false ;

    PointT p_ = depth.at(best_j, best_i) ;

    int ox, oy ;

    // find densest point cluster in a 3cm sphere around the detected point
    findMeanShiftPoint(depth, best_j, best_i, ox, oy, 0.03) ;

    // compute normal vector around this point
    n = computeNormal(depth, ox, oy) ;

    p = Vec3(p_.x, p_.y, p_.z) ;
}

