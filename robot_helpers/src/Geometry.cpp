#include <robot_helpers/Geometry.h>
#include <iostream>
#include <Eigen/Geometry>


#include <geometric_shapes/shape_operations.h>

using namespace std ;
using namespace Eigen ;

namespace robot_helpers {

ostream &operator << (ostream &strm, const Quaterniond &q)
{
    cout << "[ " << q.x() << ' ' << q.y() << ' ' << q.z() << ' ' << q.w() << " ]";

    return strm ;
}


Quaterniond quatFromRPY(double roll, double pitch, double yaw)
{
    Quaterniond q ;

    q = AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());

    return q ;
}

void rpyFromQuat(const Quaterniond &q, double &roll, double &pitch, double &yaw)
{
    Matrix3d r = q.toRotationMatrix() ;
    Vector3d euler = r.eulerAngles(2, 1, 0) ;
    yaw = euler.x() ;
    pitch = euler.y() ;
    roll = euler.z() ;
}

Quaterniond lookAt(const Eigen::Vector3d &dir, double roll)
{
     Vector3d nz = dir, na, nb ;
     nz.normalize() ;

     double q = sqrt(nz.x() * nz.x() + nz.y() * nz.y()) ;

     if ( q < 1.0e-4 )
     {
         na = Vector3d(0, 1, 0) ;
         nb = nz.cross(na) ;
     }
     else {
         na = Vector3d(-nz.y()/q, nz.x()/q, 0) ;
         nb = Vector3d(-nz.x() * nz.z()/q, -nz.y() * nz.z()/q, q) ;
     }

     Matrix3d r ;
     r << na, nb, nz ;

     return Quaterniond(r) * AngleAxisd(roll, Eigen::Vector3d::UnitZ()) ;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////


// hacked from freeglut

static void fghCircleTable(double **sint,double **cost,const int n)
{
    int i;

    /* Table size, the sign of n flips the circle direction */

    const int size = abs(n);

    /* Determine the angle between samples */

    const double angle = 2*M_PI/(double)( ( n == 0 ) ? 1 : n );

    /* Allocate memory for n samples, plus duplicate of first entry at the end */

    *sint = (double *) calloc(sizeof(double), size+1);
    *cost = (double *) calloc(sizeof(double), size+1);

    /* Bail out if memory allocation fails, fgError never returns */

    if (!(*sint) || !(*cost))
    {
        free(*sint);
        free(*cost);

    }

    /* Compute cos and sin around the circle */

    (*sint)[0] = 0.0;
    (*cost)[0] = 1.0;

    for (i=1; i<size; i++)
    {
        (*sint)[i] = sin(angle*i);
        (*cost)[i] = cos(angle*i);
    }

    /* Last sample is duplicate of the first */

    (*sint)[size] = (*sint)[0];
    (*cost)[size] = (*cost)[0];
}


void makeSolidCone( shapes::Mesh  &mesh, double base, double height, int slices, int stacks )
{
    int i,j;

    /* Step in z and radius as stacks are drawn. */

    double z0, z1;
    double r0, r1;

    const double zStep = height / ( ( stacks > 0 ) ? stacks : 1 );
    const double rStep = base / ( ( stacks > 0 ) ? stacks : 1 );

    /* Scaling factors for vertex normals */

    const double cosn = ( height / sqrt ( height * height + base * base ));
    const double sinn = ( base   / sqrt ( height * height + base * base ));

    /* Pre-computed circle */

    double *sint,*cost;


    fghCircleTable(&sint,&cost,-slices);

    /* Cover the circular base with a triangle fan... */

    z0 = -height ;

    r0 = base;
    r1 = r0 - rStep;

    int maxVert = 6*(slices * stacks + 10) ;

    mesh.vertexCount = 0 ;
    mesh.vertices = new double [maxVert] ;
    mesh.triangles = new unsigned int [maxVert] ;
    mesh.normals =  new double [maxVert] ;

    int vc = 0, nc = 0, tc = 0 ;

    // make bottom faces

    mesh.vertices[vc++] = 0.0 ;
    mesh.vertices[vc++] = 0.0 ;
    mesh.vertices[vc++] = z0 ;


    mesh.vertexCount ++ ;

    for (j=0; j<slices; j++ )
    {
        mesh.vertices[vc++] = cost[j]*r0 ;
        mesh.vertices[vc++] = sint[j]*r0 ;
        mesh.vertices[vc++] = z0 ;

        mesh.vertexCount ++ ;
    }

#define CYCLE(a) (((a)==slices+1) ? 1 : (a))

    for (j=0; j<slices; j++ )
    {

        mesh.triangles[tc++] = j+1 ;
        mesh.triangles[tc++] = 0 ;
        mesh.triangles[tc++] = CYCLE(j+2) ;


        mesh.normals[nc++] = 0.0 ;
        mesh.normals[nc++] = 0.0 ;
        mesh.normals[nc++] = 1 ;

        mesh.triangleCount ++ ;
    }


    /* Cover each stack with a quad strip, except the top stack */

    r1 = r0 ;
    z1 = z0 ;

    for( i=1; i<stacks-1; i++ )
    {
        r1 -= rStep ;
        z1 += zStep ;

        for(j=0; j<slices; j++)
        {
            mesh.vertices[vc++] = cost[j]*r1 ;
            mesh.vertices[vc++] = sint[j]*r1 ;
            mesh.vertices[vc++] = z1 ;

            mesh.vertexCount ++ ;
        }



        for(j=0; j<slices; j++)
        {

            mesh.triangles[tc++] = (i -1)*(slices) + j + 1 ;
            mesh.triangles[tc++] = (i -1)*(slices) + CYCLE(j+2);
            mesh.triangles[tc++] = i * (slices)  + CYCLE(j+2) ;
            mesh.triangleCount ++ ;

            mesh.normals[nc++] = cost[j]*cosn ;
            mesh.normals[nc++] = sint[j]*cosn ;
            mesh.normals[nc++] = sinn ;


            mesh.triangles[tc++] = (i -1)*(slices) + j + 1 ;
            mesh.triangles[tc++] = i * (slices) + CYCLE(j+2) ;
            mesh.triangles[tc++] = i * (slices)  + j + 1 ;
            mesh.triangleCount ++ ;

            mesh.normals[nc++] = cost[j]*cosn ;
            mesh.normals[nc++] = sint[j]*cosn ;
            mesh.normals[nc++] = sinn ;

        }
     }

    mesh.vertices[vc++] = 0 ;
    mesh.vertices[vc++] = 0 ;
    mesh.vertices[vc++] = 0 ;

    mesh.vertexCount ++ ;

    for(j=0; j<slices; j++)
    {

        mesh.triangles[tc++] = mesh.vertexCount -1 - slices + j + 1 ;
        mesh.triangles[tc++] = mesh.vertexCount -1 - slices + CYCLE(j+2) ;
        mesh.triangles[tc++] = mesh.vertexCount -1 ;

        mesh.normals[nc++] = cost[j]*cosn ;
        mesh.normals[nc++] = sint[j]*cosn ;
        mesh.normals[nc++] = sinn ;

        mesh.triangleCount ++ ;
    }

    /* Release sin and cos tables */

    free(sint);
    free(cost);
}


} // namespace robot_helpers
