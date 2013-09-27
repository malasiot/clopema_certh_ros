#include "utilities.h"

using namespace std;
using namespace cv;
 
void init_port(int *fd, unsigned int baud)
{
  struct termios options;
  tcgetattr(*fd,&options);
  switch(baud)
    {
    case 9600: cfsetispeed(&options,B9600);
      cfsetospeed(&options,B9600);
      break;
    case 19200: cfsetispeed(&options,B19200);
      cfsetospeed(&options,B19200);
      break;
    case 38400: cfsetispeed(&options,B38400);
      cfsetospeed(&options,B38400);
      break;
    default:cfsetispeed(&options,B9600);
      cfsetospeed(&options,B9600);
      break;
   
      options.c_cflag |= (CLOCAL | CREAD);
      options.c_cflag &= ~PARENB;
      options.c_cflag &= ~CSTOPB;
      options.c_cflag &= ~CSIZE;
      options.c_cflag |= CS8;
      tcsetattr(*fd,TCSANOW,&options);
    }
}

void photometricStereo(Mat &Lights, Mat *IgrFF, int w, int lowbound, int upbound, int numImages, Mat &Nx, Mat &Ny, Mat &Nz, Mat &alb_gr)
{
  Mat M(Mat_<double>(0, w * w)); // PS matrix
  Scalar a; // Intensity values for check

  int index1, index2, index3; // Counters

  for (index1 = 0; index1 < w; index1++)
    {
      for (index2 = 0; index2 < w; index2++)
        {
	  Mat Lights_inv;
	  Mat M1(Mat_<double>(3, 1));
	  Mat L1(Mat_<double>(0, 3));
	  Mat S(Mat_<double>(0, 1));
	  for (index3 = 0; index3 < numImages; index3++)
            {
	      a = IgrFF[index3].at<double>(index1, index2);
	      if (a.val[0] < upbound && a.val[0] > lowbound)
                {
		  S.push_back(IgrFF[index3].row(index1).col(index2));
		  L1.push_back(Lights.row(index3));
                }
            }
	  invert(L1, Lights_inv, DECOMP_SVD);
	  M1 = Lights_inv * S;
	  Mat M2 = M1.t();
	  M.push_back(M2);
        }
    }

  Mat alb_gr1 = M.col(0).mul(M.col(0)) + M.col(1).mul(M.col(1)) + M.col(2).mul(M.col(2));

  sqrt(alb_gr1, alb_gr);

  divide((M.col(0)), alb_gr, Nx);
  divide((M.col(1)), alb_gr, Ny);
  divide((M.col(2)), alb_gr, Nz);
    
  Nx = Nx.reshape(0, w);
  Ny = Ny.reshape(0, w);
  Nz = Nz.reshape(0, w);
	
  double max_albedo = *max_element(alb_gr.begin<double>(), alb_gr.end<double>());
  double min_albedo = *min_element(alb_gr.begin<double>(), alb_gr.end<double>());
  alb_gr = (alb_gr - min_albedo + 1) / (max_albedo - min_albedo);

  alb_gr = alb_gr.reshape(0, w);

}

Mat playCamera(VideoCapture *cap, int serialPort, Rect roi, const char *ledNumber, string whichImages)
{
  Mat frame;
  string case1 = "Full"; // Acquire full images
  string case2 = "Multi"; // Acquire multiview images
  string case3 = "Focus"; // Acquire image for focus metric
  write(serialPort, "0", 1);

  if (whichImages.compare(case1) == 0)
    {
        
      namedWindow("Preview",1);
      int c = -1;
      while(c == -1)
        {
	  write(serialPort, ledNumber, 1);
	  *cap >> frame;            
	  cvtColor(frame(roi), frame, CV_RGB2GRAY);
	  imshow("Preview", frame);
	  c = waitKey(5);
        }
      //destroyWindow("Preview");

    }
  else if (whichImages.compare(case2) == 0)
    {
      write(serialPort, ledNumber, 1);
      for (int i = 1; i < 20; i++)
        {
	  *cap >> frame;
        }
      cvtColor(frame(roi), frame, CV_RGB2GRAY);
    }
  else if (whichImages.compare(case3) == 0)
    {
      write(serialPort, "0", 1);
      write(serialPort, ledNumber, 1);
      for (int i = 1; i < 30; i++)
        {            
	  *cap >> frame;
        }
      cvtColor(frame(roi), frame, CV_RGB2GRAY);
    }

  write(serialPort, "0", 1);

  return frame;
}


Mat flatFielding(int w, Mat FF_Image, Mat inImage, string How)
{
  Mat IgrFF; // Flat fielded image

  if (How.compare("fitting") == 0)
    {
      // y = f(x1, x2)
      Mat x1(Mat_<double>(w * w, 1));
      Mat x2(Mat_<double>(w * w, 1));
      Mat y(Mat_<double>(w * w, 1));

      Mat ffCoefs;
      Mat Igr_norm(Mat_<double>(w, w)), IFF; // Igr_norm: normalized gr-scale image || IFF: flatfielded image
      Mat inImageuchar(Mat_<double>(w, w));

      int index1 = 0, index2 = 0, k = 0;
      for (index1 = 0; index1 < w; index1++)
        {
	  for (index2 = 0; index2 < w; index2++)
            {
	      x1.row(k) = index1 + 1;
	      x2.row(k) = index2 + 1;
	      y.row(k) = FF_Image.at<uchar>(index1, index2);
	      inImageuchar.row(index1).col(index2) = inImage.at<uchar>(index1, index2);
	      k++;
            }
        }

      ffCoefs = linRegress(x1, x2, y, w * w);

      Mat x11 = x1.reshape(0, w);
      Mat x21 = x2.reshape(0, w);

      IFF = (((ffCoefs.row(3)).mul(x11)).mul(x11)) + (((ffCoefs.row(5)).mul(x11)).mul(x21)) + ((ffCoefs.row(1)).mul(x11)) + (((ffCoefs.row(4)).mul(x21)).mul(x21)) + ((ffCoefs.row(2)).mul(x21)) + ffCoefs.row(0);

      divide(inImageuchar, IFF, Igr_norm);

      Scalar mean_v = mean(inImageuchar);

      IgrFF = mean_v.val[0] * Igr_norm;
    }
  else if (How.compare("normalize") == 0)
    {
      Mat Igr_norm(Mat_<double>(w, w)), IFF(Mat_<double>(w, w)); // Igr_norm: normalized gr-scale image || IFF: flatfielded image
      Mat inImageuchar(Mat_<double>(w, w));

      int index1 = 0, index2 = 0, k = 0;
      for (index1 = 0; index1 < w; index1++)
        {
	  for (index2 = 0; index2 < w; index2++)
            {
	      inImageuchar.row(index1).col(index2) = inImage.at<uchar>(index1, index2);
	      IFF.row(index1).col(index2) = FF_Image.at<uchar>(index1, index2);
	      k++;
            }
        }

      divide(inImageuchar, IFF, Igr_norm);

      Scalar mean_v = mean(inImageuchar);

      IgrFF = mean_v.val[0] * Igr_norm * 0.7;
    }


  return IgrFF;

}

Mat linRegress(Mat x1, Mat x2, Mat y, double n)
{
  Mat b;

  Scalar sx1, sx12, sx13, sx14;
  Scalar sx2, sx22, sx23, sx24;
  Scalar sx1x2, sx12x2, sx13x2, sx1x22, sx1x23, sx12x22;
  Scalar sy, sx1y, sx12y, sx2y, sx22y, sx1x2y;
  /* Compute some things we need */
  sx1 = sum(x1);
  sx12 = sum(x1.mul(x1));
  sx13 = sum((x1.mul(x1)).mul(x1));
  sx14 = sum(((x1.mul(x1)).mul(x1)).mul(x1));
  sx2 = sum(x2);
  sx22 = sum(x2.mul(x2));
  sx23 = sum((x2.mul(x2)).mul(x2));
  sx24 = sum(((x2.mul(x2)).mul(x2)).mul(x2));
  sx1x2 = sum(x1.mul(x2));
  sx12x2 = sum((x1.mul(x1)).mul(x2));
  sx13x2 = sum(((x1.mul(x1)).mul(x1)).mul(x2));
  sx1x22 = sum((x2.mul(x2)).mul(x1));
  sx1x23 = sum(((x2.mul(x2)).mul(x2)).mul(x1));
  sx12x22 = sum(((x1.mul(x1)).mul(x2)).mul(x2));
  sy = sum(y);
  sx1y = sum(x1.mul(y));
  sx12y = sum((x1.mul(x1)).mul(y));
  sx2y = sum(x2.mul(y));
  sx22y = sum((x2.mul(x2)).mul(y));
  sx1x2y = sum((x1.mul(x2)).mul(y));


  Mat A(Mat_<double>(6,6));
  A.row(0).col(0) = n;
  A.row(0).col(1) = sx1.val[0];
  A.row(0).col(2) = sx2.val[0];
  A.row(0).col(3) = sx12.val[0];
  A.row(0).col(4) = sx22.val[0];
  A.row(0).col(5) = sx1x2.val[0];
  A.row(1).col(0) = sx1.val[0];
  A.row(1).col(1) = sx12.val[0];
  A.row(1).col(2) = sx1x2.val[0];
  A.row(1).col(3) = sx13.val[0];
  A.row(1).col(4) = sx1x22.val[0];
  A.row(1).col(5) = sx12x2.val[0];
  A.row(2).col(0) = sx2.val[0];
  A.row(2).col(1) = sx1x2.val[0];
  A.row(2).col(2) = sx22.val[0];
  A.row(2).col(3) = sx12x2.val[0];
  A.row(2).col(4) = sx23.val[0];
  A.row(2).col(5) = sx1x22.val[0];
  A.row(3).col(0) = sx12.val[0];
  A.row(3).col(1) = sx13.val[0];
  A.row(3).col(2) = sx12x2.val[0];
  A.row(3).col(3) = sx14.val[0];
  A.row(3).col(4) = sx12x22.val[0];
  A.row(3).col(5) = sx13x2.val[0];
  A.row(4).col(0) = sx22.val[0];
  A.row(4).col(1) = sx1x22.val[0];
  A.row(4).col(2) = sx23.val[0];
  A.row(4).col(3) = sx12x22.val[0];
  A.row(4).col(4) = sx24.val[0];
  A.row(4).col(5) = sx1x23.val[0];
  A.row(5).col(0) = sx1x2.val[0];
  A.row(5).col(1) = sx12x2.val[0];
  A.row(5).col(2) = sx1x22.val[0];
  A.row(5).col(3) = sx13x2.val[0];
  A.row(5).col(4) = sx1x23.val[0];
  A.row(5).col(5) = sx12x22.val[0];

  Mat c(Mat_<double>(6,1));
  c.row(0).col(0) = sy.val[0];
  c.row(1).col(0) = sx1y.val[0];
  c.row(2).col(0) = sx2y.val[0];
  c.row(3).col(0) = sx12y.val[0];
  c.row(4).col(0) = sx22y.val[0];
  c.row(5).col(0) = sx1x2y.val[0];


  /* Solve For Coefficients */
  Mat D;
  invert(A, D);
  b = D * c;
  return b;
}

void quadrantShift(int w, Mat &wx, Mat &wy)
{
  //Mat wx(Mat_<double>(w, w));
  //Mat wy(Mat_<double>(w, w));

  int index1, index2;
  int wHalf1, wHalf2;

  if (w % 2 == 0)
    {
      wHalf1 = w / 2;
      wHalf2 = wHalf1;
    }
  else
    {
      wHalf1 = (w + 1) / 2;
      wHalf2 = wHalf1 - 1;
    }

  for (index1 = 0; index1 < w; index1++)
    {
      for (index2 = 0; index2 < wHalf1; index2++)
        {
	  wx.row(index1).col(index2) = (wHalf2 + index2) * ((atan((double) 1) * 4) / (w - 1)) - ((atan((double) 1) * 4) / 2);
	  wy.row(index2).col(index1) = (wHalf2 + index2) * ((atan((double) 1) * 4) / (w - 1)) - ((atan((double) 1) * 4) / 2);
        }
      for (index2 = wHalf1; index2 < w; index2++)
        {
	  wx.row(index1).col(index2) = (index2 - wHalf1) * ((atan((double) 1) * 4) / (w - 1)) - ((atan((double) 1) * 4) / 2);
	  wy.row(index2).col(index1) = (index2 - wHalf1) * ((atan((double) 1) * 4) / (w - 1)) - ((atan((double) 1) * 4) / 2);
        }
    }
}

Mat psCloud(int w, Mat Nx, Mat Ny, Mat Nz)
{
  // Load shifts
  Mat wx(Mat_<double>(w, w));
  Mat wy(Mat_<double>(w, w));

  quadrantShift(w, wx, wy);

  Mat dzdx, dzdy, DZDX, DZDY, Z11, Z12, Z2, Z0[2], z1, z0, z[2], z_;

  divide(Ny, Nz, dzdx);
  divide((-Nx), Nz, dzdy);

  dft(dzdx, DZDX, DFT_COMPLEX_OUTPUT);
  dft(dzdy, DZDY, DFT_COMPLEX_OUTPUT);

  Mat AA[2], BB[2];
  split(DZDX, AA);
  split(DZDY, BB);

  Z11 = wx.mul(BB[1]) + wy.mul(AA[1]);
  Z12 = -(wx.mul(BB[0]) + wy.mul(AA[0]));
  Z2 = (wx.mul(wx) + wy.mul(wy));
  divide(Z11, Z2, Z0[0]);
  divide(Z12, Z2, Z0[1]);
  merge(Z0, 2, z1);

  idft(z1, z0, DFT_SCALE);
  split(z0, z);
  double mmin, mmax;
  minMaxIdx(z[0], &mmin, &mmax);
  z_ = z[0] - mmin;

  return z_;
}

Scalar focusMetric(Mat frame, Rect roiFM)
{

  if (roiFM.width < frame.cols)
    {
      if (roiFM.height < frame.rows)
        {
	  frame = frame(roiFM);
        }
    }

  // Horizontal and vertical diff images
  Mat DH(Mat_<uchar>(frame.rows, frame.cols));
  Mat DV(Mat_<uchar>(frame.rows, frame.cols));

  Mat FM, DH1, DV1; // FM: focus metric image / DH1, DV1: temp images

  int index1, index2;

  Scalar focusMeasure; // Focus Metric

  if (frame.channels() > 1)
    {
      cvtColor(frame, frame, CV_RGB2GRAY);
    }

  DH1 = frame;
  DV1 = frame;

  for (index1 = 0; index1 < (frame.rows - 2); index1++)
    {
      for (index2 = 0; index2 < (frame.cols - 2); index2++)
        {
	  DH.row(index1).col(index2) = DH1.row(index1 + 2).col(index2) - DH1.row(index1).col(index2);
	  DV.row(index1).col(index2) = DV1.row(index1).col(index2 + 2) - DV1.row(index1).col(index2);
        }
    }

  max(DH, DV, FM);
  FM = FM.mul(FM);
  focusMeasure = mean(FM);

  return focusMeasure;
}

