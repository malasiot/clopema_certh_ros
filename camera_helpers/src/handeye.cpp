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
#include <Eigen/Eigenvalues>
#include <unsupported/Eigen/NonLinearOptimization>

using namespace std ;
using namespace cv ;
using namespace Eigen ;

static bool findCorners(const cv::Mat &im, const cv::Size &boardSize, vector<Point2f> &corners)
{
    cv::Mat gray ;

    cv::cvtColor(im, gray, CV_BGR2GRAY) ;

    //CALIB_CB_FAST_CHECK saves a lot of time on images
    //that do not contain any chessboard corners

    bool patternfound = cv::findChessboardCorners(gray, boardSize, corners,
            CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE );

    if(patternfound)
        cv::cornerSubPix(gray, corners, Size(5, 5), Size(-1, -1),
            TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
/*

    cv::drawChessboardCorners(im, boardSize, Mat(corners), patternfound);

    cv::imwrite("/home/malasiot/tmp/oo.png", im) ;

    cout << "ok here" ;
*/

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

// Generic functor
template<typename _Scalar, int NX=Dynamic, int NY=Dynamic>
struct Functor
{
  typedef _Scalar Scalar;
  enum {
    InputsAtCompileTime = NX,
    ValuesAtCompileTime = NY
  };
  typedef Matrix<Scalar,InputsAtCompileTime,1> InputType;
  typedef Matrix<Scalar,ValuesAtCompileTime,1> ValueType;
  typedef Matrix<Scalar,ValuesAtCompileTime,InputsAtCompileTime> JacobianType;

  const int m_inputs, m_values;

  Functor() : m_inputs(InputsAtCompileTime), m_values(ValuesAtCompileTime) {}
  Functor(int inputs, int values) : m_inputs(inputs), m_values(values) {}

  int inputs() const { return m_inputs; }
  int values() const { return m_values; }

  // you should define that in the subclass :
//  void operator() (const InputType& x, ValueType* v, JacobianType* _j=0) const;
};

struct SolverFunctor: public Functor<double>
{

    SolverFunctor(const vector<Affine3d> &A_, const vector<Affine3d> &B_):
        A(A_), B(B_) {}

  int operator()(const VectorXd &x, VectorXd &fvec) const
  {
      Matrix4d X, Z ;

      for(int i=0, k=0 ; i<3 ; i++)
          for(int j=0 ; j<4 ; j++, k++)
          {
              X(i, j) = x[k] ;
              Z(i, j) = x[k+12];
          }

      int n = A.size() ;

      Matrix3d Rx = X.block<3, 3>(0, 0), Rz = Z.block<3, 3>(0, 0) ;
      Vector3d Tx = X.block<3, 1>(0, 3), Tz = Z.block<3, 1>(0, 3) ;

      int k = 0 ;

      for(int r=0 ; r<n ; r++ )
      {
          Matrix3d Ra = A[r].rotation(), Rb = B[r].rotation() ;
          Vector3d Ta = A[r].translation(), Tb = B[r].translation() ;

          Matrix3d C = Ra * Rx - Rz * Rb ;
          Vector3d S = Ra * Tx + Ta - Rz * Tb - Tz ;

          for(int i=0 ; i<3 ; i++ )
              for(int j=0 ; j<3 ; j++ )
                  fvec[k++] = C(i, j) ;

          for(int i=0 ; i<3 ; i++ )
              fvec[k++] = S[i] ;
      }

      const double cfactor = 1.0e3 ;

      Matrix3d Cx = Rx.transpose() * Rx - Matrix3d::Identity() ;
      Matrix3d Cz = Rz.transpose() * Rz - Matrix3d::Identity() ;

      for(int i=0 ; i<3 ; i++ )
          for(int j=0 ; j<3 ; j++ )
              fvec[k++] = cfactor * Cx(i, j) ;

      for(int i=0 ; i<3 ; i++ )
          for(int j=0 ; j<3 ; j++ )
              fvec[k++] = cfactor * Cz(i, j) ;

      return 0;
  }

  int inputs() const { return 24 ; }
  int values() const { return 12*A.size() + 18; } // number of constraints

  const vector<Affine3d> &A,  &B ;

};

bool solveHandEyeNonLinear(const vector<Affine3d> &A, const vector<Affine3d> &B,
                  Affine3d &X, Affine3d &Z )
{
    SolverFunctor functor(A, B);
    NumericalDiff<SolverFunctor> numDiff(functor);

    LevenbergMarquardt<NumericalDiff<SolverFunctor>, double> lm(numDiff);
//    lm.parameters.factor =10 ;

    VectorXd Y(24) ;

    for(int i=0, k=0 ; i<3 ; i++)
        for(int j=0 ; j<4 ; j++, k++)
        {
            Y[k] = X(i, j) ;
            Y[k+12] = Z(i, j) ;
        }

    LevenbergMarquardtSpace::Status status = lm.minimizeInit(Y);
    do {
        status = lm.minimizeOneStep(Y);
          double fnorm = lm.fvec.blueNorm();
          cout << "-------\n" ;
          cout << lm.fvec << endl ;
    } while (status==LevenbergMarquardtSpace::Running);


    lm.minimize(Y) ;

    for(int i=0, k=0 ; i<3 ; i++)
        for(int j=0 ; j<4 ; j++, k++)
        {
            X(i, j) = Y[k] ;
            Z(i, j) = Y[k+12];
        }

    Matrix3d Rx = X.rotation(), Rz = Z.rotation() ;
    Vector3d Tx = X.translation(), Tz = Z.translation() ;

    double fnorm = lm.fvec.blueNorm();

    return true ;
}

bool solveHandEyeLinear(const vector<Affine3d> &A, const vector<Affine3d> &B,
                  Affine3d &X, Affine3d &Z )
{
    assert(A.size() == B.size()) ;

    int n = A.size() ;
    Quaterniond qz, qx ;

    // Solve for rotation

    Matrix4d C = Matrix4d::Zero() ;

    for(int i=0 ; i<n ; i++)
    {
        Quaterniond qa, qb ;
        qa = Quaterniond(A[i].rotation()) ;
        qb = Quaterniond(B[i].rotation()) ;

        // compute -Q^t * W

        Matrix4d Qt, W ;

        Qt <<   -qa.w(), -qa.x(), -qa.y(), -qa.z(),
                 qa.x(), -qa.w(), -qa.z(),  qa.y(),
                 qa.y(),  qa.z(), -qa.w(), -qa.x(),
                 qa.z(), -qa.y(),  qa.x(), -qa.w();

        W <<     qb.w(), -qb.x(), -qb.y(), -qb.z(),
                 qb.x(),  qb.w(),  qb.z(), -qb.y(),
                 qb.y(), -qb.z(),  qb.w(),  qb.x(),
                 qb.z(),  qb.y(), -qb.x(),  qb.w();

        C += Qt*W ;
    }

    SelfAdjointEigenSolver<Matrix4d> eigensolver(C.transpose() * C);

    if ( eigensolver.info() != Success ) return false ;

    Vector4d eval = eigensolver.eigenvalues() ;


    int best = -1 ;
    double min_eig = DBL_MAX ;

    for(int i=0 ; i<4 ; i++)
    {
        double val ;
        if ( ( val = n + sqrt(eval[i])) < min_eig )
        {
            min_eig = val ;
            best = i ;
        }

        if ( ( val = n - sqrt(eval[i])) < min_eig )
        {
            min_eig = val ;
            best = i ;
        }
    }


    Vector4d eg = eigensolver.eigenvectors().col(best) ;

    qz = Quaterniond(eg[0], eg[1], eg[2], eg[3]) ;

    Vector4d qx_ = (1.0/(min_eig - n)) * C * eg ;

    qx = Quaterniond(qx_[0], qx_[1], qx_[2], qx_[3]) ;
    qx.normalize();

    Matrix3d RX = qx.toRotationMatrix() ;
    Matrix3d RZ = qz.toRotationMatrix() ;


    // Solve for translation

    MatrixXd MA(3*n, 6) ;
    VectorXd MB(3*n) ;

    for(int i=0 ; i<n ; i++)
    {
        MA.block<3, 3>(3*i, 0) = A[i].rotation() ;
        MA.block<3, 3>(3*i, 3) = -Matrix3d::Identity() ;
        MB.segment<3>(3*i) = RZ * B[i].translation() - A[i].translation() ;
    }

    VectorXd sol = MA.jacobiSvd(ComputeThinU | ComputeThinV).solve(MB) ;

    Vector3d TX = sol.segment<3>(0) ;
    Vector3d TZ = sol.segment<3>(3) ;

    X = Translation3d(TX) * RX ;
    Z = Translation3d(TZ) * RZ ;

    return true ;
}


int main( int argc, char* argv[] )
{

    string filePrefix = "grab_" ;
    string fileSuffix = "_c.*" ;
    string dataFolder = "/home/malasiot/images/clothes/calibration/" ;

    const cv::Size boardSize(3, 5) ;
    const double squareSize = 0.04 ;

    namespace fs = boost::filesystem  ;

    std::string fileNameRegex = filePrefix + "([[:digit:]]+)" + fileSuffix ;

    boost::regex rx(fileNameRegex) ;

    vector<vector<Point2f> > img_corners ;
    vector<vector<Point3f> > obj_corners ;
    vector<Affine3d> gripper_to_base ;

    fs::directory_iterator it(dataFolder), end ;

    for( ; it != end ; ++it)
    {
        fs::path p = (*it).path() ;

        string fileName = p.filename().string() ;

        boost::smatch sm ;

        // test of this is a valid filename

        if ( !boost::regex_match(fileName, sm, rx ) ) continue ;

        std::string imageNumber = sm[1] ;

        cv::Mat im = imread(p.string(), -1) ;

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


        vector<Point2f> img_corners_ ;
        vector<Point3f> obj_corners_ ;

        img_corners.push_back(vector<Point2f>()) ;
        obj_corners.push_back(vector<Point3f>()) ;

        if ( findCorners(im, boardSize, img_corners.back()) )
        {

            for( int i = 0; i < boardSize.height; i++ )
                for( int j = 0; j < boardSize.width; j++ )
                    obj_corners.back().push_back(Point3f(float(j*squareSize), float(i*squareSize), 0));

            gripper_to_base.push_back(tr.inverse()) ;
        }
        else
        {
            img_corners.pop_back() ;
            obj_corners.pop_back() ;

        }

    }

    vector<Mat> rvecs, tvecs;
    vector<float> reprojErrs;
    vector<Affine3d> target_to_sensor ;

    double totalAvgErr = 0;

    cv::Mat camMatrix, distCoeffs ;

    int flag = CALIB_USE_INTRINSIC_GUESS | CALIB_FIX_FOCAL_LENGTH | CALIB_FIX_PRINCIPAL_POINT ;
    runCalibration(img_corners, obj_corners, cv::Size(640, 480), 640/480.0, flag,
                   camMatrix, distCoeffs, rvecs, tvecs, reprojErrs, totalAvgErr) ;

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
    }

    Affine3d sensor_to_base, target_to_gripper ;

//    gripper_to_base * target_to_gripper = sensor_to_base * target_to_sensor
//
//            A       *        X          =       Z        *        B

    solveHandEyeLinear(gripper_to_base, target_to_sensor,
                       target_to_gripper,  sensor_to_base ) ;


    //solveHandEyeNonLinear(base_to_gripper, sensor_to_target, gripper_to_target,  base_to_sensor) ;

    cout << sensor_to_base.inverse().translation() << endl ;

    return 0;
}
