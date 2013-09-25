#include <certh_libs/cvHelpers.h>
#include "cvblob/cvBlob/cvblob.h"
#include "psimpl.h"

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
using namespace cv ;

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

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void simplifyContour(const std::vector<cv::Point2d> &in_pts, std::vector<cv::Point2d> &out_pts, double avgError)
{
    vector<double> srcData, dstData ;

    for(uint j=0 ; j<in_pts.size() ; j++)
    {
        const cv::Point &p = in_pts[j] ;
        srcData.push_back(p.x) ;
        srcData.push_back(p.y) ;
    }

    psimpl::simplify_douglas_peucker <2> (srcData.begin(), srcData.end(), avgError,  std::back_inserter (dstData));

    for(uint j=0 ; j<dstData.size() ; j+=2)
    {
        cv::Point2d p (dstData[j], dstData[j+1]) ;
        out_pts.push_back(p) ;
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


static void followContour(cv::Mat_<uchar> &bmap, int i, int j, vector<cv::Point> &c)
{
    int k, x, y, npts ;
    int xx, yy ;
    int w, h ;

    w = bmap.cols ;
    h = bmap.rows ;

    stack<cv::Point> ptstack ;

    ptstack.push(cv::Point(j, i)) ;

    x = j ;
    y = i ;

    npts = 1 ;
    uchar &val = bmap[y][x] ;


    do
    {
        if ( val == 'E' || val == 'C' ) bmap[y][x] = 0 ;

        int x1, y1 ;
        bool isolated = true ;

        xx = x-1 ; yy = y ;

        for( k=0 ; k<8 ; k++ )
        {
            switch (k)
            {
                case 0:
                    yy-- ; break ;
                case 1:
                    xx++ ; break ;
                case 2:
                    xx++ ; break ;
                case 3:
                    yy++ ; break ;
                case 4:
                    yy++ ; break ;
                case 5:
                    xx-- ; break ;
                case 6:
                    xx-- ; break ;
                case 7:
                    yy-- ; break ;
            }

            if ( xx<0 || yy<0 || xx>w-1 || yy>h-1 ) continue ;

            val = bmap[yy][xx] ;

            if ( val == 0 ) continue ;

            x1 = xx ; y1 = yy ;	isolated = false ;

            if ( val == 'J' )	break ;
        }

        if ( isolated ) break ;

        x = x1 ; y = y1 ;
        ptstack.push(cv::Point(x, y)) ;
        npts ++ ;

        val = bmap[y][x] ;

    } while ( val && val != 'E' && val != 'J' ) ;

    if ( val == 'E' ) bmap[y][x] = 0 ;

    for( i=0 ; i<npts ; i++ )
    {
        cv::Point p = ptstack.top() ;

        ptstack.pop() ;
        c.push_back(p) ;
    }
}


void edgeLinking(const cv::Mat_<uchar> &edges, vector< vector<cv::Point> > &clist)
{
    int i, j, k, w, h ;

    w = edges.cols ;
    h = edges.rows ;

    cv::Mat_<uchar> map(h, w) ;

    for( i=0 ; i<h ; i++ )
        for( j=0 ; j<w ; j++ )
        {
            int nn = 0, oldk = -1, pp = 0  ;

            map[i][j] = 0 ;

            if ( !edges[i][j] ) continue ;

            int x = j-1, y = i ;

            for( k=0 ; k<9 ; k++ )
            {
                switch (k%8)
                {
                    case 0:
                        y-- ; break ;
                    case 1:
                        x++ ; break ;
                    case 2:
                        x++ ; break ;
                    case 3:
                        y++ ; break ;
                    case 4:
                        y++ ; break ;
                    case 5:
                        x-- ; break ;
                    case 6:
                        x-- ; break ;
                    case 7:
                        y-- ; break ;
                }

                if ( x<0 || y<0 || x>w-1 || y>h-1 ) continue ;

                if ( ! edges[y][x] ) continue ;

                if ( oldk >= 0 && oldk + 1 == k ) pp ++ ;

                if ( k < 8 ) nn++ ;
                oldk = k ;
            }

            switch(nn)
            {
                case 0 :
                    map[i][j] = 0 ;   /* isolated point */
                    break ;
                case 1 :
                    map[i][j] = 'E' ; /* contour start - end point */
                    break ;
                case 2 :
                    map[i][j] = 'C' ; /* internal contour point */
                    break ;
                case 3 :
                    if ( pp ) map[i][j] = 'C' ;
                    else map[i][j] = 'J' ;
                    break ;
                case 4 :
                    if ( pp == 1 ) map[i][j] = 'J' ;
                    else if ( pp == 2 ) map[i][j] = 'C' ;
                    else map[i][j] = 'J' ; /* junction point */
                    break ;
                case 5 :
                    map[i][j] = 'J' ;
                    break ;
            }
        }

    for( i=0 ; i<h ; i++ )
        for( j=0 ; j<w ; j++ )
        {
            if ( map[i][j] == 0 || map[i][j] == 'C' ) continue ;

            if ( map[i][j] == 'E' )
            {
                vector<cv::Point> cnt ;
                followContour(map, i, j, cnt) ;

                if ( cnt.size() > 1 ) clist.push_back(cnt) ;

            }
        }

    for( i=0 ; i<h ; i++ )
        for( j=0 ; j<w ; j++ )
        {
            if ( map[i][j] == 0 || map[i][j] == 'C' ) continue ;
            else if ( map[i][j] == 'J' )
            {
                map[i][j] = 'E' ;

                vector<cv::Point> cnt ;
                followContour(map, i, j, cnt) ;
                if ( cnt.size() > 1 ) clist.push_back(cnt) ;

            }
        }

    for( i=0 ; i<h ; i++ )
        for( j=0 ; j<w ; j++ )
        {
            if ( map[i][j] == 'C' )
            {
                vector<cv::Point> cnt ;
                followContour(map, i, j, cnt) ;

                //cnt->SetClosed() ;
                if ( cnt.size() > 1 ) clist.push_back(cnt) ;

            }
        }
}

//////////////////////////////////////////////////////////////////////////////////////

class PtSorter
{
public:
    PtSorter() {}

    bool operator () (const Point &p1, const Point &p2) {
        return p1.x * p1.x + p1.y * p1.y <  p2.x * p2.x + p2.y * p2.y ;
    }
};

bool sampleNearestNonZeroDepth(const cv::Mat &dim, int x, int y, ushort &z)
{
    assert ( dim.type() == CV_16UC1 ) ;

    static vector<Point> dpts ;
    const int ws = 3 ;

    if ( dpts.empty() )
    {
        for(int i=-ws ; i<=ws ; i++ )
            for(int j=-ws ; j<=ws ; j++ )
                dpts.push_back(Point(j, i))  ;

        PtSorter sorter ;
        std::sort(dpts.begin(), dpts.end(), sorter) ;
    }

    bool found = true ;

    for(int i=0 ; i<dpts.size() ; i++)
    {
        const Point &p = dpts[i] ;

        int x_ = p.x + x ;
        int y_ = p.y + y ;

        if ( x_ < 0 || y_ < 0 || x_ >= dim.cols || y_ >= dim.rows ) continue ;
        if ( ( z = dim.at<ushort>(y_, x_) ) == 0 ) continue ;

        found = true ;
    }

    return found ;

}

bool sampleBilinearDepth(const cv::Mat &dim, float x, float y, float &z)
{
    assert ( dim.type() == CV_16UC1 ) ;

    int ix = x, iy = y ;
    float hx = x - ix, hy = y - iy ;

    if ( ( ix + 1 < 0 || ix + 1 >= dim.cols ) ||
         ( iy + 1 < 0 || iy + 1 >= dim.rows ) ||
         ( ix < 0 ) || ( iy < 0 ) )
    {
        ushort uz ;
        bool res = sampleNearestNonZeroDepth(dim, ix, iy, uz) ;
        z = uz ;
        return res ;
    }

    ushort z1 = dim.at<ushort>(iy, ix) ;
    ushort z2 = dim.at<ushort>(iy, ix+1) ;
    ushort z3 = dim.at<ushort>(iy+1, ix) ;
    ushort z4 = dim.at<ushort>(iy+1, ix+1) ;

    if ( z1 == 0 || z2 == 0 || z3 == 0 || z4 == 0 )
    {
        ushort uz ;
        bool res = sampleNearestNonZeroDepth(dim, ix, iy, uz) ;
        z = uz ;
        return res ;
    }
    else
    {
        float s1 = (1 - hx) * z1 + hx * z2 ;
        float s2 = (1 - hx) * z3 + hx * z4 ;

        z = ( 1 - hy ) * s1 + hy * s2 ;

        return true ;
    }
}


cv::Mat findLargestBlob(const cv::Mat &mask_ref, vector<cv::Point> &hull)
{
    int w = mask_ref.cols, h = mask_ref.rows ;
    cv::Mat planeMask = cv::Mat::zeros(h, w, CV_8UC1) ;

    // find largest blob in image

    cv::Mat_<uchar> mask_erode ;

    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::erode(mask_ref, mask_erode, element) ;

    cvb::CvBlobs blobs ;
    IplImage mask_erode_ipl = mask_erode ;

    IplImage *labelImg = cvCreateImage(mask_erode.size(),IPL_DEPTH_LABEL,1);
    cvb::cvLabel(&mask_erode_ipl, labelImg, blobs) ;

    if ( blobs.size() == 0 ) return planeMask ;

    cv::Mat labels(labelImg) ;

    cvb::CvLabel gb = cvb::cvGreaterBlob(blobs) ;
    cvb::cvFilterByLabel(blobs, gb) ;

    cvb::CvBlobs::const_iterator it = blobs.begin() ;

    cvb::CvBlob *blob = (*it).second ;

    for(int y=blob->miny ; y<=blob->maxy ; y++)
        for(int x=blob->minx ; x<=blob->maxx ; x++)
        {
            if ( labels.at<cvb::CvLabel>(y, x) == gb )
                planeMask.at<uchar>(y, x) = 255 ;
        }

    cvb::CvContourPolygon *poly = cvb::cvConvertChainCodesToPolygon(&blob->contour) ;

    vector<cv::Point> contour ;

    for(int i=0 ; i<poly->size() ; i++ )
        contour.push_back(cv::Point((*poly)[i].x, (*poly)[i].y)) ;

    /// Find the convex hull object for each contour

    cv::convexHull( cv::Mat(contour), hull, false );

    cvReleaseImage(&labelImg);
}

}
