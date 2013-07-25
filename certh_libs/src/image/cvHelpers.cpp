#include <certh_libs/cvHelpers.h>


#if defined(_WIN32) && !defined(__CYGWIN32__)
#include <io.h>
#else
#include <unistd.h>
#define _MIN(a,b) ((a<b)?(a):(b))
#endif

#include <cassert>

#ifndef SGN
#define SGN(a)          (((a)<0) ? -1 : 1)
#endif

using namespace std ;

namespace certh_libs {

void getScanLine(const cv::Point &p1, const cv::Point &p2, vector<cv::Point> &pts)
{
    int d, x, y, ax, ay, sx, sy, dx, dy;

    int x1 = p1.x ; int y1 = p1.y ;
    int x2 = p2.x ; int y2 = p2.y ;

    dx = x2-x1;  ax = abs(dx)<<1;  sx = SGN(dx);
    dy = y2-y1;  ay = abs(dy)<<1;  sy = SGN(dy);

    x = x1;
    y = y1;

    if (ax>ay)
    {                /* x dominant */
        d = ay-(ax>>1);
        for (;;)
        {
            pts.push_back(cv::Point(x, y)) ;

            if (x==x2) break;
            if (d>=0)
            {
                y += sy;
                d -= ax;
            }
            x += sx;
            d += ay;
        }
    }
    else
    {                      /* y dominant */
        d = ax-(ay>>1);
        for (;;)
        {
            pts.push_back(cv::Point(x, y)) ;

            if (y==y2) break;
            if (d>=0)
            {
                x += sx;
                d -= ay;
            }
            y += sy;
            d += ax;
        }
    }

}

void computeGradient(const cv::Mat &clr, cv::Mat &mag, cv::Mat &ang, double gradMagThreshold)
{
    int w = clr.cols, h = clr.rows ;

    cv::Mat gx(h, w, CV_16SC3 ), gy(h, w, CV_16SC3) ;
    mag = cv::Mat(h, w, CV_32FC1) ;
    ang = cv::Mat(h, w, CV_32FC1) ;

    cv::Sobel(clr, gx, CV_16S, 1, 0, 1) ;
    cv::Sobel(clr, gy, CV_16S, 0, 1, 1) ;

    signed short *glx = (signed short *)gx.data ;
    signed short *gly = (signed short *)gy.data ;
    int gstride = gx.step1(0) ;
    int dstride = mag.step1(0) ;

    float *dlp =  (float *)mag.data, *alp = (float *)ang.data ;

    unsigned long threshold = gradMagThreshold * gradMagThreshold ;

    for(int i=0 ; i<h ; i++ )
    {
       signed short *gpx = glx, *gpy = gly ;
       float *dp = dlp, *ap = alp ;

        for(int j=0 ; j<w ; j++ )
        {
            long gx_r = *gpx++, gx_g = *gpx++, gx_b = *gpx++ ;
            long gy_r = *gpy++, gy_g = *gpy++, gy_b = *gpy++ ;

            unsigned long mag_r = gx_r * gx_r + gy_r * gy_r ;
            unsigned long mag_g = gx_g * gx_g + gy_g * gy_g ;
            unsigned long mag_b = gx_b * gx_b + gy_b * gy_b ;

            if ( mag_r > mag_g && mag_r > mag_b )
            {
                if ( mag_r < threshold )  {
                    *dp++ = 0.0 ; *ap++ = 0 ;
                }
                else {
                    *dp++ = sqrt((float) mag_r/4) ;
                    *ap++ = atan2((float) gx_r, -gy_r) ;
                }
            }
            else if ( mag_g > mag_r && mag_g > mag_b )
            {
                if ( mag_g < threshold ) {
                    *dp++ = 0.0 ; *ap++ = 0 ;
                }
                else {
                    *dp++ = sqrt((float) mag_g/4) ;
                    *ap++ = atan2((float) gx_g, -gy_g) ;
                }
            }
            else
            {
                if ( mag_b < threshold ) {
                    *dp++ =0.0 ; *ap++ = 0 ;
                }
                else {
                    *dp++ = sqrt((float) mag_b) ;
                    *ap++ = atan2((float) gx_b, -gy_b) ;
                }
            }
        }

        glx += gstride ; gly += gstride ;
        dlp += dstride ; alp += dstride ;
    }

}

void computeGradientField(const cv::Mat &clr, cv::Mat &ogx, cv::Mat &ogy, double gradMagThreshold)
{
    int w = clr.cols, h = clr.rows ;

    cv::Mat gx(h, w, CV_16SC3 ), gy(h, w, CV_16SC3) ;
    ogx = cv::Mat(h, w, CV_32FC1) ;
    ogy = cv::Mat(h, w, CV_32FC1) ;

    cv::Sobel(clr, gx, CV_16S, 1, 0, 1) ;
    cv::Sobel(clr, gy, CV_16S, 0, 1, 1) ;

    signed short *glx = (signed short *)gx.data ;
    signed short *gly = (signed short *)gy.data ;
    int gstride = gx.step1(0) ;
    int dstride = ogx.step1(0) ;

    float *gxlp =  (float *)ogx.data, *gylp = (float *)ogy.data ;

    unsigned long threshold = gradMagThreshold * gradMagThreshold ;

    for(int i=0 ; i<h ; i++ )
    {
       signed short *gpx = glx, *gpy = gly ;
       float *gxp = gxlp, *gyp = gylp ;

        for(int j=0 ; j<w ; j++ )
        {
            long gx_r = *gpx++, gx_g = *gpx++, gx_b = *gpx++ ;
            long gy_r = *gpy++, gy_g = *gpy++, gy_b = *gpy++ ;

            unsigned long mag_r = gx_r * gx_r + gy_r * gy_r ;
            unsigned long mag_g = gx_g * gx_g + gy_g * gy_g ;
            unsigned long mag_b = gx_b * gx_b + gy_b * gy_b ;

            if ( mag_r > mag_g && mag_r > mag_b )
            {
                if ( mag_r < threshold )  {
                    *gxp++ = 0.0 ; *gyp++ = 0 ;
                }
                else {
                    *gxp++ = gx_r ;
                    *gyp++ = gy_r ;
                }
            }
            else if ( mag_g > mag_r && mag_g > mag_b )
            {
                if ( mag_g < threshold ) {
                    *gxp++ = 0.0 ; *gyp++ = 0 ;
                }
                else {
                    *gxp++ = gx_g ;
                    *gyp++ = gy_g ;
                }
            }
            else
            {
                if ( mag_b < threshold ) {
                    *gxp++ = 0.0 ; *gyp++ = 0 ;
                }
                else {
                    *gxp++ = gx_b ;
                    *gyp++ = gy_b ;
                }
            }
        }

        glx += gstride ; gly += gstride ;
        gxlp += dstride ; gylp += dstride ;
    }

}





}
