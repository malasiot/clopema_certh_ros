#include "calibration.h"

#include <highgui.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <iostream>
#include <fstream>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/regex.hpp>
#include <boost/algorithm/string.hpp>

#include <Eigen/Geometry>

using namespace std ;
using namespace cv ;
using namespace Eigen ;

int counter = 0 ;
static bool findCorners(const cv::Mat &im, const cv::Size &boardSize, vector<Point2f> &corners)
{
    cv::Mat gray ;

    cv::cvtColor(im, gray, CV_BGR2GRAY) ;

    //CALIB_CB_FAST_CHECK saves a lot of time on images
    //that do not contain any chessboard corners

    bool patternfound = cv::findChessboardCorners(gray, boardSize, corners,
            CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE );

    if ( patternfound && (corners[0].y > corners[boardSize.width].y) )
    {
        // reverse direction

        vector<Point2f> corners_ ;

        for( int i=corners.size()-1 ; i>=0 ; i-- )
            corners_.push_back(corners[i]) ;

        corners = corners_ ;

    }

    if(patternfound)
        cv::cornerSubPix(gray, corners, Size(5, 5), Size(-1, -1),
            TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));


    cv::drawChessboardCorners(im, boardSize, Mat(corners), patternfound);

    stringstream str ;

    str << "calib_pts_" << counter++  << ".png" ;

    cv::imwrite("/tmp/" + str.str(), im) ;



    return patternfound ;
}

static double computeReprojectionErrors(
        const vector<vector<Point3f> >& objectPoints,
        const vector<vector<Point2f> >& imagePoints,
        const vector<Mat>& rvecs, const vector<Mat>& tvecs,
        const Mat& cameraMatrix, const Mat& distCoeffs,
        vector<float>& perViewErrors )
{
    vector<Point2f> imagePoints2;
    int i, totalPoints = 0;
    double totalErr = 0, err;
    perViewErrors.resize(objectPoints.size());

    for( i = 0; i < (int)objectPoints.size(); i++ )
    {

        projectPoints(Mat(objectPoints[i]), rvecs[i], tvecs[i],
                      cameraMatrix, distCoeffs, imagePoints2);
        err = norm(Mat(imagePoints[i]), Mat(imagePoints2), CV_L2);
        int n = (int)objectPoints[i].size();
        perViewErrors[i] = (float)std::sqrt(err*err/n);
        totalErr += err*err;
        totalPoints += n;
    }

    return std::sqrt(totalErr/totalPoints);
}

static bool runCalibration( const vector<vector<Point2f> > &imagePoints,
                            const vector<vector<Point3f> > &objectPoints,
                            const cv::Size &imageSize,
                            float aspectRatio,
                            int flags, Mat& cameraMatrix, Mat& distCoeffs,
                            vector<Mat>& rvecs, vector<Mat>& tvecs,
                            vector<float>& reprojErrs,
                            double& totalAvgErr)
{
    cameraMatrix = Mat::eye(3, 3, CV_64F);

  //  if( flags & CV_CALIB_FIX_ASPECT_RATIO )
  //      cameraMatrix.at<double>(0,0) = aspectRatio;

    cameraMatrix.at<double>(0, 0) = 525 ;
    cameraMatrix.at<double>(1, 1) = 525 ;
    cameraMatrix.at<double>(0, 2) = imageSize.width/2 - 0.5 ;
    cameraMatrix.at<double>(1, 2) = imageSize.height/2 - 0.5 ;

    distCoeffs = Mat::zeros(8, 1, CV_64F);

    double rms = cv::calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix,
                    distCoeffs, rvecs, tvecs, flags|CV_CALIB_FIX_K4|CV_CALIB_FIX_K5);
                    ///*|CV_CALIB_FIX_K3*/|CV_CALIB_FIX_K4|CV_CALIB_FIX_K5);
    printf("RMS error reported by calibrateCamera: %g\n", rms);

    bool ok = cv::checkRange(cameraMatrix) && cv::checkRange(distCoeffs);

    totalAvgErr = computeReprojectionErrors(objectPoints, imagePoints,
                rvecs, tvecs, cameraMatrix, distCoeffs, reprojErrs);

    return ok;
}


Affine3d randomAffine(double scale=0.1)
{

    Vector3d trans = 0.1 * Vector3d::Random() ;
    Vector4d rot = Vector4d::Random()*scale ;
    rot.normalize() ;

    return Translation3d(trans) * Quaterniond(rot).toRotationMatrix() ;
}

void estimateTargetToSensorProjective(const vector<vector<Point2f> > &img_corners, const vector<vector<Point3f> > &obj_corners,
                                      const cv::Mat &camMatrix, vector<Affine3d> & target_to_sensor)

{

    vector<Mat> rvecs, tvecs;
    vector<float> reprojErrs;

    double totalAvgErr = 0;

    cv::Mat distCoeffs, camMatrix_ = camMatrix ;

    int flag = CALIB_USE_INTRINSIC_GUESS | CALIB_FIX_FOCAL_LENGTH | CALIB_FIX_PRINCIPAL_POINT ;
    runCalibration(img_corners, obj_corners, cv::Size(640, 480), 640/480.0, flag,
                   camMatrix_, distCoeffs, rvecs, tvecs, reprojErrs, totalAvgErr) ;

/*
    Affine3d sensor_to_base, target_to_gripper ;

    sensor_to_base = randomAffine() ;

    cout << sensor_to_base.matrix() << endl ;
    sensor_to_base.translation() = Vector3d(0, 1.5, 0.2) ;

    target_to_gripper = randomAffine() ;
*/
    for(unsigned int i=0 ; i<tvecs.size() ; i++ )
    {
        Matrix3d r ;
        Vector3d t ;

        cv::Mat rmat_ ;
        cv::Rodrigues(rvecs[i], rmat_) ;

        cv::Mat_<double> rmat(rmat_) ;
        cv::Mat_<double> tmat(tvecs[i]) ;

        r << rmat(0, 0), rmat(0, 1), rmat(0, 2), rmat(1, 0), rmat(1, 1), rmat(1, 2), rmat(2, 0), rmat(2, 1), rmat(2, 2) ;
        t << tmat(0, 0), tmat(0, 1), tmat(0, 2) ;

        Affine3d tr = Translation3d(t) * r ;

        target_to_sensor.push_back(tr) ;

   /*    Affine3d tr_ = sensor_to_base.inverse() * gripper_to_base[i] * target_to_gripper;

        Quaterniond qq ;

        qq.vec() += Vector3d::Random()*0.001 ;
        qq.normalize();

        tr_.rotate(qq.toRotationMatrix()) ;

        target_to_sensor.push_back(tr_) ;
        */
    }

}

void estimateTargetToSensorRigid(const vector<vector<Point3f> > &img_corners, const vector<vector<Point3f> > &obj_corners,
                                      const cv::Mat &camMatrix, vector<Affine3d> & target_to_sensor)
{
    assert(img_corners.size() == obj_corners.size()) ;

    int nFrames = img_corners.size() ;

    double fx = camMatrix.at<double>(0, 0) ;
    double fy = camMatrix.at<double>(1, 1) ;
    double cx = camMatrix.at<double>(0, 2) ;
    double cy = camMatrix.at<double>(1, 2) ;

    for(int i=0 ; i<nFrames ; i++ )
    {
        const vector<Point3f> &ic = img_corners[i] ;
        const vector<Point3f> &oc = obj_corners[i] ;

        vector<Eigen::Vector3f> srcPts, dstPts ;

        const double scale = 1.0e-3 ;

        for(int j=0 ; j<ic.size() ; j++ )
        {
            ushort z = ic[j].z ;

            if ( z == 0 ) continue ;

            Eigen::Vector3f ip( scale * ( ic[j].x - cx ) * z / fx, scale * ( ic[j].y - cy ) * z / fy, scale * z) ;
            Point3f op = oc[j] ;

            srcPts.push_back(ip) ;
            dstPts.push_back(Eigen::Vector3f(op.x, op.y, op.z)) ;
        }

        int nPts = srcPts.size() ;

        MatrixXf src(3, nPts), dst(3, nPts) ;

        for( int j=0 ; j<nPts ; j++ )
        {
            src.col(j) = srcPts[j] ;
            dst.col(j) = dstPts[j] ;
        }

        MatrixXf res = umeyama(dst, src, false) ;

        Affine3d tr ;
        tr.matrix() <<  res(0, 0), res(0, 1), res(0, 2), res(0, 3),
                        res(1, 0), res(1, 1), res(1, 2), res(1, 3),
                        res(2, 0), res(2, 1), res(2, 2), res(2, 3),
                        res(3, 0), res(3, 1), res(3, 2), res(3, 3);

        cout << tr.matrix() << endl ;
        target_to_sensor.push_back(tr) ;

    }
}

class PtSorter
{
public:
    PtSorter() {}

    bool operator () (const Point &p1, const Point &p2) {
        return p1.x * p1.x + p1.y * p1.y <  p2.x * p2.x + p2.y * p2.y ;
    }
};

bool nearestNonZeroDepth(const cv::Mat &dim, int x, int y, ushort &z)
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

bool bilinearDepth(const cv::Mat &dim, float x, float y, float &z)
{
    assert ( dim.type() == CV_16UC1 ) ;

    int ix = x, iy = y ;
    float hx = x - ix, hy = y - iy ;

    if ( ( ix + 1 < 0 || ix + 1 >= dim.cols ) ||
         ( iy + 1 < 0 || iy + 1 >= dim.rows ) ||
         ( ix < 0 ) || ( iy < 0 ) )
    {
        ushort uz ;
        bool res = nearestNonZeroDepth(dim, ix, iy, uz) ;
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
        bool res = nearestNonZeroDepth(dim, ix, iy, uz) ;
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




void find_target_motions(const string &filePrefix, const string &dataFolder, const cv::Size boardSize, const double squareSize, bool useDepth,
                           vector<Affine3d> &gripper_to_base, vector<Affine3d> &target_to_sensor )
{
    string fileSuffix = "_c.*" ;

    namespace fs = boost::filesystem  ;

    std::string fileNameRegex = filePrefix + "([[:digit:]]+)" + fileSuffix ;

    boost::regex rx(fileNameRegex) ;

    vector<vector<Point2f> > img_corners ;
    vector<vector<Point3f> > obj_corners, img_corners3 ;

    fs::directory_iterator it(dataFolder), end ;

    for( ; it != end ; ++it)
    {
        fs::path p = (*it).path() ;

        string fileName = p.filename().string() ;

        boost::smatch sm ;

        // test of this is a valid filename

        if ( !boost::regex_match(fileName, sm, rx ) ) continue ;

        // read color image

        cv::Mat im = imread(p.string(), -1) ;

        // read robot pose

        Affine3d tr ;

        {
            boost::replace_all(fileName,"_c.png", "_pose.txt") ;

            string filePath = p.parent_path().string() + '/' + fileName ;
            ifstream strm(filePath.c_str()) ;

            Matrix3d r ;
            Vector3d t ;

            double vals[12] ;

            for(int i=0 ; i<12 ; i++ ) strm >> vals[i] ;

            r << vals[0], vals[1], vals[2], vals[3], vals[4], vals[5], vals[6], vals[7], vals[8] ;
            t << vals[9], vals[10], vals[11] ;

            tr = Translation3d(t) * r ;
        }

        // read depth image

        cv::Mat dim ;

        if ( useDepth )
        {
            boost::replace_all(fileName, "_pose.txt", "_d.png") ;

            string filePath = dataFolder + '/' + fileName ;

            dim = cv::imread(filePath, -1) ;
        }

        img_corners.push_back(vector<Point2f>()) ;
        img_corners3.push_back(vector<Point3f>()) ;
        obj_corners.push_back(vector<Point3f>()) ;

        if ( findCorners(im, boardSize, img_corners.back()) )
        {

            vector<Point2f> &ic = img_corners.back() ;

            for( int i = 0, k=0; i < boardSize.height; i++ )
                for( int j = 0; j < boardSize.width; j++, k++ )
                {
                    if ( useDepth )
                    {
                        float z ;

                        if ( bilinearDepth(dim, ic[k].x, ic[k].y, z) )
                            img_corners3.back().push_back(Point3f(ic[k].x, ic[k].y, z)) ;
                        else
                            img_corners3.back().push_back(Point3f(ic[k].x, ic[k].y, 0.0)) ;

                    }
                    obj_corners.back().push_back(Point3f(float(j*squareSize), float(i*squareSize), 0));
                }

            gripper_to_base.push_back(tr.inverse()) ;
        }
        else
        {
            img_corners.pop_back() ;
            obj_corners.pop_back() ;
            img_corners3.pop_back() ;
        }

    }

    cv::Mat cameraMatrix ;

    cameraMatrix = Mat::eye(3, 3, CV_64F);

    cameraMatrix.at<double>(0, 0) = 525 ;
    cameraMatrix.at<double>(1, 1) = 525 ;
    cameraMatrix.at<double>(0, 2) = 640/2 - 0.5 ;
    cameraMatrix.at<double>(1, 2) = 480/2 - 0.5 ;


    if ( useDepth )
        estimateTargetToSensorRigid(img_corners3, obj_corners, cameraMatrix, target_to_sensor) ;
    else
        estimateTargetToSensorProjective(img_corners, obj_corners, cameraMatrix, target_to_sensor) ;

}
