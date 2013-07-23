#include <certh_libs/Line2D.h>

#include <Eigen/Dense>
#include <float.h>

using namespace std ;
using namespace Eigen ;

namespace certh_libs {

Line2D::Line2D(const Point2D &p1, const Point2D &p2)
{
  p = p1 ;
  d = p2 - p1 ;
  d.normalize() ;
}

/*
Line2D::Line2D(const Point2D &pt, const Vec2D &dir)
{
  p = pt ;
  d = dir ;
  d.normalise() ;
}
*/
// ax + by + c = 0 ;

Line2D::Line2D(double a, double b, double c) { init(a, b, c) ; }

void Line2D::init(double a, double b, double c)
{


    if ( fabs(a) < DBL_EPSILON )
    {
        p.x() = 0.0 ;  p.y() = -c/b ;
        d.x() = 1.0 ;  d.y() = 0.0 ;
    }
    else if ( fabs(b) < DBL_EPSILON )
    {
        p.x() = -c/a ;  p.y() = 0.0 ;
        d.x() = 0.0  ;  d.y() = 1.0 ;
    }
    else
    {
        d.x() = -b ; d.y() = a ; d.normalize() ;
        p.x() = -c/a/2.0 ; p.y() = -c/b/2.0 ;
    }
}

Line2D::Line2D(double a[3]) { init(a[0], a[1], a[2]) ; }

static int compare(const void *elem1, const void *elem2 ) 
{
  double *v1 = (double *)elem1 ;
  double *v2 = (double *)elem2 ;

  return (*v1 < *v2) ? -1 : 1 ;
}

Line2D::Line2D(const vector<Point2D> &plist, bool robust)
{
    int i, k, N = plist.size() ;
    int NITER = 10 ;

    if ( !robust ) NITER = 1 ;

    assert(N >= 1);

    if ( N == 2 ) {
        p = plist[0] ;
        d = plist[1] - plist[0] ;
        d.normalize() ;
        return ;
    }
  
    if ( N == 1 ) {
        p = plist[0] ;
        d = Vec2D(1, 0) ;
    }

    double *weight = new double [N] ;
    double *res = new double [N] ;

    for( i=0 ; i<N ; i++ ) weight[i] = 1.0 ;

    double wsum = N ;
    double mx, my, ux, uy ;

    for( k=0 ; k<NITER ; k++ )
    {
        double mxx = 0, myy = 0, mxy = 0 ;

        mx = my = 0 ;

        for( i=0 ; i<N ; i++ )
        {
            const Point2D &p = plist[i] ;
            double w = weight[i] ;

            mx += w * p.x() ; my += w * p.y() ;
        }

        mx /= wsum ; my /= wsum ;

        for( i=0 ; i<N ; i++ )
        {
            const Point2D &p = plist[i] ;

            double w = weight[i] ;

            mxx += w * (p.x() - mx) * (p.x() - mx) ;
            myy += w * (p.y() - my) * (p.y() - my) ;
            mxy += w * (p.x() - mx) * (p.y() - my) ;
        }

        mxx /= wsum ; myy /= wsum ; mxy /= wsum ;

        Matrix2d M, U ;
        Vector2 L ;

        M << mxx, mxy, mxy, myy ;

        SelfAdjointEigenSolver<Matrix2d> solver(M) ;
        L = solver.eigenvalues() ;
        M = solver.eigenvectors() ;

        ux = U(0, 0) ; uy = U(1, 0) ;

        if ( robust )
        {
            for( i=0 ; i<N ; i++ )
            {
                const Point2D &p = plist[i] ;
    
                double r ; //= (p.x - mx)*(p.x - mx) + (p.y - my)*(p.y - my) ;

                r = fabs((p.x() - mx)*U(0, 1) + (p.y() - my)*U(1, 1)) ;

                weight[i] = r ;
                res[i] = r ;
            }
 
            // Estimate residual variance
            double sigma ;

            qsort(weight, N, sizeof(double), compare) ;

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
    }

    p.x() = mx ; p.y() = my ;
    d.x() = ux ; d.y() = uy ;

    delete weight ;
    delete res ;
}

void Line2D::GetCoefs(double &a, double &b, double &c) const
{
    a = -d.y() ; b = d.x() ;
    c = d.y() * p.x() - d.x() * p.y() ;
}

void Line2D::GetCoefs(double a[3]) const
{
    GetCoefs(a[0], a[1], a[2]) ;
}

// Find the intersection between  two lines

bool Line2D::Intersection(const Line2D &other, Point2D &pp) const 
{
    double det = d.x() * other.d.y() - d.y() * other.d.x() ;

    if ( fabs(det) < DBL_EPSILON ) return false ;

    double s = p.y() * d.x() - p.x() * d.y(), t = other.p.y() * other.d.x() - other.p.x() * other.d.y() ;
    pp.x() = (s * other.d.x() - t * d.x())/det ;
    pp.y() = (s * other.d.y() - t * d.y())/det ;

    return true ;
}

bool Line2D::IsParallel(const Line2D &other) const 
{
    return (fabs(d.x() * other.d.y() - d.y() * other.d.x()) < DBL_EPSILON) ;
}

// Find the distance of a point to the line. Optionally returns the closest point on the line

double Line2D::DistanceToPoint(const Point2D &q, Point2D *psd) const
{
    double dist = (d.x() * (p.y() - q.y()) - d.y() * (p.x() - q.x())) ;

    if ( psd )
    {
        psd->x() = q.x() - dist * d.y() ;
        psd->y() = q.y() + dist * d.x() ;
    }

    return fabs(dist) ;
}


LineSegment2D::LineSegment2D(const Point2D &p1, const Point2D &p2) 
{
  pa = p1 ; pb = p2 ;
}
/*
LineSegment2D::LineSegment2D(const Point2D &p, const Vec2D &dir)
{
  pa = p ; pb = p + dir ;
}
*/
LineSegment2D::LineSegment2D(const vector<Point2D> &pts, bool robust) 
{
    Line2D lf(pts, robust) ;

    Point2D o = lf.p ;
    Point2D d = lf.d ;

    double mins = DBL_MAX, maxs = DBL_MIN ;

    for( int i=0 ; i<pts.size() ; i++ )
    {
        double s = (o - pts[i]).dot(d) ;

        if ( s < mins )
        {
            mins = s ;
            pa = o + s * d ;
        }
    
        if ( s > maxs )
        {
            maxs = s ;
            pb = o + s * d ;
        }
    }
}

Line2D LineSegment2D::GetLine() const { return Line2D(pa, pb) ; }

Vector2 LineSegment2D::GetDir() const { 
    Vector2 d = pb - pa ;
    d.normalize() ;
    return d ;
}

void LineSegment2D::Invert() { swap(pa, pb) ; }

bool LineSegment2D::Intersection(const Line2D &other, Point2D &p) const 
{
    Line2D L(pa, pb) ;

    if ( !L.Intersection(other, p) ) return false ;
    else
    {
        // Check if intersection point is inside segment

        double s = (pa - p).dot(pb - p) ;

        if ( s > 0 ) return false ;
    }

    return true ;
}

bool LineSegment2D::Intersection(const LineSegment2D &other, Point2D &p) const
{
  Line2D L(pa, pb) ;

  if ( !other.Intersection(L, p) ) return false ;
  else
  {
    // Check if intersection point is inside segment

    double s = (pa - p).dot(pb - p) ;
    double t = (other.pa - p).dot(other.pb - p) ;

    if ( s > 0 || t > 0 ) return false ;
  }

  return true ;
}


bool LineSegment2D::IsParallel(const Line2D &other) const
{
  return Line2D(pa, pb).IsParallel(other) ;
}

bool LineSegment2D::IsParallel(const LineSegment2D &other) const
{
  return Line2D(pa, pb).IsParallel(Line2D(other.pa, other.pb)) ;
}

bool LineSegment2D::IsColinear(const Line2D &other) const
{
  if ( other.DistanceToPoint(pa) < DBL_EPSILON && 
       other.DistanceToPoint(pb) < DBL_EPSILON  ) return true ;
  else return false ;
}

bool LineSegment2D::IsColinear(const LineSegment2D &other) const 
{
    return IsColinear(Line2D(other.pa, other.pb)) ;
}

// Find the distance of a point to the line. 
// Optionally returns the closest point on the line
// Optionally returns a flag indicating whether the closest point is within the
// segment (flag == 0), before point pa (flag = -1), after point pb ( flag = 1 )
  
double LineSegment2D::DistanceToPoint(const Point2D &p, Point2D *psd) const
{
    Line2D LL(pa, pb) ;

    Point2D pp ;
    double dist = LL.DistanceToPoint(p, &pp) ;

    int flag = Contains(pp) ;

    if ( flag == -1 ) {
        pp = pa ;
        dist = (p - pp).norm() ;
    }
    else if ( flag == 1 )
    {
        pp = pb ;
        dist = (p - pp).norm() ;
    }
  
    if ( psd ) *psd = pp ;

    return dist ;
}


int LineSegment2D::Contains(const Point2D &pp) const 
{
    if ( (pp - pa).dot(pb - pa) < 0 ) return -1 ;
    else if ( (pp - pb).dot(pb - pa) > 0 ) return 1 ;
    else return 0 ;
}

}
