#include "cvlpch.h"
#include "PointList2D.h"
#include "Matrix.h"

using namespace std ;

_BEGIN_NAMESPACE

PointList2D::PointList2D(double *x, double *y, int n)
{
  for(int i=0 ; i<n ; i++ )
  {
    push_back(Point2D(x[i], y[i])) ;
  }
}

PointList2D::PointList2D(const PointList2D &other): vector<Point2D>(other) {}

PointList2D::PointList2D(const std::vector<Point2D> &pts): vector<Point2D>(pts) {} 

PointList2D::PointList2D(const Vector &x) 
{
  int n = x.Size()/2 ;

  for(int i=0, r=0 ; i<n ; i++ ) {
    push_back(Point2D(x[r], x[r+1])) ; r++ ; r++ ;
  }
}

Point2D PointList2D::Center() const 
{
  int n = size() ;

  Point2D cc ;

  for(int i=0 ; i<n ; i++ ) {
    cc += (*this)[i] ;
  }

  return (n == 0) ? cc : cc/n ;

}

void PointList2D::Axes(double &l1, Vector2 &v1, double &l2, Vector2 &v2) const 
{
  double mxx = 0, myy = 0, mxy = 0 ;

  double mx = 0, my = 0 ;

  int N = size(), i ;

  for( i=0 ; i<N ; i++ )
  {
    const Point2D &p = (*this)[i] ;
    
    mx += p.x ; my += p.y ;
  }

  mx /= N ; my /= N ;

  for( i=0 ; i<N ; i++ )
  {
    const Point2D &p = (*this)[i] ;
    mxx += (p.x - mx) * (p.x - mx) ; 
    myy += (p.y - my) * (p.y - my) ;
    mxy += (p.x - mx) * (p.y - my) ; 
  }

  mxx /= N ; myy /= N ; mxy /= N ;

  Matrix2x2 M(mxx, mxy, mxy, myy), U ;
  Vector2 L ;

  M.Eigen(L, U) ;

  l1 = L.m1 ; l2 = L.m2 ;
  v1.m1 = U.m11 ; v1.m2 = U.m21 ;
  v2.m1 = U.m12 ; v2.m2 = U.m22 ;

}

#define SQR(x) ((x)*(x))

void PointList2D::Align(const PointList2D &X, double &theta, double &scale, double &tx, double &ty) const
{
  int i ;
  int nX = X.size(), nY = size() ;

  const PointList2D &Y = *this ;

  assert( nX >= nY ) ;
  
  Point2D mX = X.Center(), mY = Y.Center() ;
  
  PointList2D X0(X), Y0(Y) ;

  for(i=0 ; i<nX ; i++ ) X0[i] -= mX ; 
  for(i=0 ; i<nY ; i++ ) Y0[i] -= mY ;

  Vector ssqX(nX), ssqY(nY) ;
  for(i=0 ; i<nX ; i++ ) ssqX[i] = X0[i].m1 * X0[i].m1 + X0[i].m2 * X0[i].m2 ;
  for(i=0 ; i<nY ; i++ ) ssqY[i] = Y0[i].m1 * Y0[i].m1 + Y0[i].m2 * Y0[i].m2 ;

  bool constX = true, constY = true ;
  const double eps = DBL_EPSILON ;


  for(i=0 ; i<nX ; i++ ) 
  {
    if ( ssqX[i] > SQR(eps * nX * mX.m1) || ssqX[i] > SQR(eps * nX * mX.m2) ) {
      constX = false ;
      break ;
    }
  }
      
  for(i=0 ; i<nY ; i++ ) 
  {
    if ( ssqY[i] > SQR(eps * nY * mY.m1) || ssqY[i] > SQR(eps * nY * mY.m2) ) {
      constY = false ;
      break ;
    }
  }


  if (  !constX && !constY )
  {
    Vector2 XF, YF ;

    for( i=0 ; i<nX ; i++)
    {
      const Point2D &v = X0[i] ;
      XF += Vector2(SQR(v.m1), SQR(v.m2)) ;
    }

    for( i=0 ; i<nY ; i++)
    {
      const Point2D &v = Y0[i] ;
      YF += Vector2(SQR(v.m1), SQR(v.m2)) ;
    }
    // the "centered" Frobenius norm
    double normX = sqrt(XF.x + XF.y) ;
    double normY = sqrt(YF.x + YF.y) ;
 
    // scale to equal (unit) norm

    for( i=0 ; i<nX ; i++ ) X0[i] /= normX ;
    for( i=0 ; i<nY ; i++ ) Y0[i] /= normY ;

    Matrix2x2 xy ; 

    for( int k=0 ; k<std::min(nX, nY) ; k++ )
    {      
      xy += Matrix2x2(X0[k].m1 * Y0[k].m1, X0[k].m1 * Y0[k].m2, 
                      X0[k].m2 * Y0[k].m1, X0[k].m2 * Y0[k].m2) ;
      
    }
     
    Matrix2x2 U, VT ;
    Vector2 S ;

  
    xy.SVD(U, S, VT) ;

    Matrix2x2 R( VT * U ) ;
   //cout<<"The rotation matrix T is"<<endl<<T<<endl ;

    theta = asin(R(0, 1)) ;

    // optimum (symmetric in X and Y) scaling of Y
    
    double sY = S[0] + S[1]; // == trace(sqrtm(A'*A))

    double s = sY * normX / normY ;
    //cout<<"The scale is "<<s<<endl ;

    scale = s ;
    // translation of Y
    Vector2 c( mX - s*R*mY );
    // cout<<"The translation vector is"<<endl<<c<<endl ;
  
    tx = c[0] ;
    ty = c[1] ;


  }
  
}

void PointList2D::Align(const PointList2D &Xt, double *weights, double &theta, double &scale, double &tx, double &ty) const
{
  int i, N = Xt.size() ;
  double S1 = 0.0, S2 = 0.0, S3 = 0.0, S4 = 0.0, S5 = 0.0, S6 = 0.0, S7 = 0.0, S8 = 0.0 ; 
  
  const PointList2D &X = *this ;

  for( i=0 ; i<N ; i++ )
  {
    double x = X[i].x ;
    double y = X[i].y ;
    double xt = Xt[i].x ;
    double yt = Xt[i].y ;
    double w = weights[i] ;
    
    S1 += w * (x * x + y * y) ;
    S2 += w * (x * xt + y * yt) ;
    S3 += w * x ;
    S4 += w * y ;
    S5 += w * (x * yt - y * xt) ;
    S6 += w * xt ;
    S7 += w * yt ;
    S8 += w ;
  }

  Matrix A(4, 4) ;

  A(0, 0) = S1 ; A(0, 1) =   0 ; A(0, 2) =  S3 ; A(0, 3) = S4 ;
  A(1, 0) = 0  ; A(1, 1) =  S1 ; A(1, 2) = -S4 ; A(1, 3) = S3 ;
  A(2, 0) = S3 ; A(2, 1) = -S4 ; A(2, 2) =  S8 ; A(2, 3) = 0  ;
  A(3, 0) = S4 ; A(3, 1) =  S3 ; A(3, 2) =   0 ; A(3, 3) = S8 ;

  Matrix B(4, 1), S(4, 1) ;

  B(0, 0) = S2 ;
  B(1, 0) = S5 ;
  B(2, 0) = S6 ;
  B(3, 0) = S7 ;

  Solve(A, B, S) ;

  double a = S(0, 0) ;
  double b = S(1, 0) ;
  tx = S(2, 0) ;
  ty = S(3, 0) ;

  scale = sqrt(a * a + b * b) ;
  theta = acos(a/scale) ;
}

void PointList2D::Transform(const AffineTransform &xf)
{
  for( int i = 0 ; i<size() ; i++ )
  {
    xf.Apply((*this)[i]) ;
  }
}

Vector PointList2D::GetData() const 
{
  Vector cc(2*size()) ;

  for(int i=0, r = 0 ; i<size() ; i++ )
  {
    cc[r] = (*this)[i].x ; r++ ;
    cc[r] = (*this)[i].y ; r++ ;
  }

  return cc ;
}

void PointList2D::SetData(const Vector &v)
{
  clear() ;

  for(int i=0, r = 0 ; i<v.Size()/2 ; i++ )
  {
    push_back(Point2D(v[2*i], v[2*i+1])) ;
  }

}

Archive &operator << (Archive &ar, const PointList2D &v) 
{
  ar << (const std::vector<Point2D> &)v ;

  return ar ;
}


Archive &operator >> (Archive &ar, PointList2D &v) 
{
  ar >> (std::vector<Point2D> &)v ;

  return ar ;
}


void PointList2D::BBox(Point2D &ul, Point2D &br) const 
{
  ul = br = (*this)[0] ;

  for( int i=0 ; i<size() ; i++ )
  {
    ul = min(ul, (*this)[i]) ;
    br = max(br, (*this)[i]) ;
  }



}



//////////////////////////////////////////////////////////////////////////////////////////////////////

PointList2Ds::PointList2Ds(float *x, float *y, int n)
{
  for(int i=0 ; i<n ; i++ )
  {
    push_back(Point2Ds(x[i], y[i])) ;
  }
}

PointList2Ds::PointList2Ds(const PointList2Ds &other): vector<Point2Ds>(other) {}

PointList2Ds::PointList2Ds(const std::vector<Point2Ds> &pts): vector<Point2Ds>(pts) {} 

PointList2Ds::PointList2Ds(const VectorS &x) 
{
  int n = x.Size()/2 ;

  for(int i=0, r=0 ; i<n ; i++ ) {
    push_back(Point2Ds(x[r], x[r+1])) ; r++ ; r++ ;
  }
}

Point2Ds PointList2Ds::Center() const 
{
  int n = size() ;

  Point2Ds cc ;

  for(int i=0 ; i<n ; i++ ) {
    cc += (*this)[i] ;
  }

  return (n == 0) ? cc : cc/n ;

}

void PointList2Ds::Axes(float &l1, Vector2s &v1, float &l2, Vector2s &v2) const 
{
  float mxx = 0, myy = 0, mxy = 0 ;

  float mx = 0, my = 0 ;

  int N = size(), i ;

  for( i=0 ; i<N ; i++ )
  {
    const Point2Ds &p = (*this)[i] ;
    
    mx += p.x ; my += p.y ;
  }

  mx /= N ; my /= N ;

  for( i=0 ; i<N ; i++ )
  {
    const Point2Ds &p = (*this)[i] ;
    mxx += (p.x - mx) * (p.x - mx) ; 
    myy += (p.y - my) * (p.y - my) ;
    mxy += (p.x - mx) * (p.y - my) ; 
  }

  mxx /= N ; myy /= N ; mxy /= N ;

  Matrix2x2s M(mxx, mxy, mxy, myy), U ;
  Vector2s L ;

  M.Eigen(L, U) ;

  l1 = L.m1 ; l2 = L.m2 ;
  v1.m1 = U.m11 ; v1.m2 = U.m21 ;
  v2.m1 = U.m12 ; v2.m2 = U.m22 ;

}

#define SQR(x) ((x)*(x))

void PointList2Ds::Align(const PointList2Ds &X, float &theta, float &scale, float &tx, float &ty) const
{
  int i ;
  int nX = X.size(), nY = size() ;

  const PointList2Ds &Y = *this ;

  assert( nX >= nY ) ;
  
  Point2Ds mX = X.Center(), mY = Y.Center() ;
  
  PointList2Ds X0(X), Y0(Y) ;

  for(i=0 ; i<nX ; i++ ) X0[i] -= mX ; 
  for(i=0 ; i<nY ; i++ ) Y0[i] -= mY ;

  Vector ssqX(nX), ssqY(nY) ;
  for(i=0 ; i<nX ; i++ ) ssqX[i] = X0[i].m1 * X0[i].m1 + X0[i].m2 * X0[i].m2 ;
  for(i=0 ; i<nY ; i++ ) ssqY[i] = Y0[i].m1 * Y0[i].m1 + Y0[i].m2 * Y0[i].m2 ;

  bool constX = true, constY = true ;
  const float eps = DBL_EPSILON ;


  for(i=0 ; i<nX ; i++ ) 
  {
    if ( ssqX[i] > SQR(eps * nX * mX.m1) || ssqX[i] > SQR(eps * nX * mX.m2) ) {
      constX = false ;
      break ;
    }
  }
      
  for(i=0 ; i<nY ; i++ ) 
  {
    if ( ssqY[i] > SQR(eps * nY * mY.m1) || ssqY[i] > SQR(eps * nY * mY.m2) ) {
      constY = false ;
      break ;
    }
  }


  if (  !constX && !constY )
  {
    Vector2s XF, YF ;

    for( i=0 ; i<nX ; i++)
    {
      const Point2Ds &v = X0[i] ;
      XF += Vector2s(SQR(v.m1), SQR(v.m2)) ;
    }

    for( i=0 ; i<nY ; i++)
    {
      const Point2Ds &v = Y0[i] ;
      YF += Vector2s(SQR(v.m1), SQR(v.m2)) ;
    }
    // the "centered" Frobenius norm
    float normX = sqrt(XF.x + XF.y) ;
    float normY = sqrt(YF.x + YF.y) ;
 
    // scale to equal (unit) norm

    for( i=0 ; i<nX ; i++ ) X0[i] /= normX ;
    for( i=0 ; i<nY ; i++ ) Y0[i] /= normY ;

    Matrix2x2s xy ; 

    for( int k=0 ; k<std::min(nX, nY) ; k++ )
    {      
      xy += Matrix2x2s(X0[k].m1 * Y0[k].m1, X0[k].m1 * Y0[k].m2, 
                      X0[k].m2 * Y0[k].m1, X0[k].m2 * Y0[k].m2) ;
      
    }
     
    Matrix2x2s U, VT ;
    Vector2s S ;

  
    xy.SVD(U, S, VT) ;

    Matrix2x2s R( VT * U ) ;
   //cout<<"The rotation matrix T is"<<endl<<T<<endl ;

    theta = asin(R(0, 1)) ;

    // optimum (symmetric in X and Y) scaling of Y
    
    float sY = S[0] + S[1]; // == trace(sqrtm(A'*A))

    float s = sY * normX / normY ;
    //cout<<"The scale is "<<s<<endl ;

    scale = s ;
    // translation of Y
    Vector2s c( mX - s*R*mY );
    // cout<<"The translation vector is"<<endl<<c<<endl ;
  
    tx = c[0] ;
    ty = c[1] ;


  }
  
}

void PointList2Ds::Align(const PointList2Ds &Xt, float *weights, float &theta, float &scale, float &tx, float &ty) const
{
  int i, N = Xt.size() ;
  float S1 = 0.0, S2 = 0.0, S3 = 0.0, S4 = 0.0, S5 = 0.0, S6 = 0.0, S7 = 0.0, S8 = 0.0 ; 
  
  const PointList2Ds &X = *this ;

  for( i=0 ; i<N ; i++ )
  {
    float x = X[i].x ;
    float y = X[i].y ;
    float xt = Xt[i].x ;
    float yt = Xt[i].y ;
    float w = weights[i] ;
    
    S1 += w * (x * x + y * y) ;
    S2 += w * (x * xt + y * yt) ;
    S3 += w * x ;
    S4 += w * y ;
    S5 += w * (x * yt - y * xt) ;
    S6 += w * xt ;
    S7 += w * yt ;
    S8 += w ;
  }

  Matrix A(4, 4) ;

  A(0, 0) = S1 ; A(0, 1) =   0 ; A(0, 2) =  S3 ; A(0, 3) = S4 ;
  A(1, 0) = 0  ; A(1, 1) =  S1 ; A(1, 2) = -S4 ; A(1, 3) = S3 ;
  A(2, 0) = S3 ; A(2, 1) = -S4 ; A(2, 2) =  S8 ; A(2, 3) = 0  ;
  A(3, 0) = S4 ; A(3, 1) =  S3 ; A(3, 2) =   0 ; A(3, 3) = S8 ;

  Matrix B(4, 1), S(4, 1) ;

  B(0, 0) = S2 ;
  B(1, 0) = S5 ;
  B(2, 0) = S6 ;
  B(3, 0) = S7 ;

  Solve(A, B, S) ;

  float a = S(0, 0) ;
  float b = S(1, 0) ;
  tx = S(2, 0) ;
  ty = S(3, 0) ;

  scale = sqrt(a * a + b * b) ;
  theta = acos(a/scale) ;
}

void PointList2Ds::Transform(const AffineTransformS &xf)
{
  for( int i = 0 ; i<size() ; i++ )
  {
    xf.Apply((*this)[i]) ;
  }
}

VectorS PointList2Ds::GetData() const 
{
  VectorS cc(2*size()) ;

  for(int i=0, r = 0 ; i<size() ; i++ )
  {
    cc[r] = (*this)[i].x ; r++ ;
    cc[r] = (*this)[i].y ; r++ ;
  }

  return cc ;
}

void PointList2Ds::SetData(const VectorS &v)
{
  clear() ;

  for(int i=0, r = 0 ; i<v.Size()/2 ; i++ )
  {
    push_back(Point2Ds(v[2*i], v[2*i+1])) ;
  }

}

Archive &operator << (Archive &ar, const PointList2Ds &v) 
{
  ar << (const std::vector<Point2Ds> &)v ;

  return ar ;
}


Archive &operator >> (Archive &ar, PointList2Ds &v) 
{
  ar >> (std::vector<Point2Ds> &)v ;

  return ar ;
}


void PointList2Ds::BBox(Point2Ds &ul, Point2Ds &br) const 
{
  ul = br = (*this)[0] ;

  for( int i=0 ; i<size() ; i++ )
  {
    ul = min(ul, (*this)[i]) ;
    br = max(br, (*this)[i]) ;
  }
}

void PointList2D::Copy(const PointList2Ds &v) 
{
	resize(v.size());

	for(int i=0 ; i<v.size() ; i++ )
		(*this)[i].Copy(v[i]) ;
}

void PointList2Ds::Copy(const PointList2D &v) 
{
	resize(v.size());

	for(int i=0 ; i<v.size() ; i++ )
		(*this)[i].Copy(v[i]) ;
}




_END_NAMESPACE