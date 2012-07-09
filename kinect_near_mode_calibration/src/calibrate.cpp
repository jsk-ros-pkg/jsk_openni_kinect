/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/**
   Based on kinect_calibration (Kurt Konolige, Patrick Mihelich)
   \author Atsushi Tsuda, Kei Okada

**/

#include <stdio.h>
#include <string.h>
#include <iostream>
#include <iomanip>
#include <unistd.h> // getopt
#include <cstdlib>

#include <math.h>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <Eigen/Core>
#include <Eigen/LU>

using namespace cv;
using namespace Eigen;
using namespace std;

// image size
#define ROWS 480
#define COLS 640

// Pixel offset from IR image to depth image
cv::Point2f ir_depth_offset = cv::Point2f(-4, -3);

#define SHIFT_SCALE 0.125

// change shift to disparity
double shift2disp(int shift, double shift_offset)
{
  return SHIFT_SCALE*(double)(shift_offset - shift);
}

// colorizes a depth pixel
uint16_t t_gamma[2048];         // gamma conversion for depth

void setDepthColor(uint8_t *cptr, int d)
{
  int pval = t_gamma[d];
  int lb = pval & 0xff;
  switch (pval >> 8)
    {
    case 0:
      cptr[2] = 255;
      cptr[1] = 255 - lb;
      cptr[0] = 255 - lb;
      break;
    case 1:
      cptr[2] = 255;
      cptr[1] = lb;
      cptr[0] = 0;
      break;
    case 2:
      cptr[2] = 255 - lb;
      cptr[1] = 255;
      cptr[0] = 0;
      break;
    case 3:
      cptr[2] = 0;
      cptr[1] = 255;
      cptr[0] = lb;
      break;
    case 4:
      cptr[2] = 0;
      cptr[1] = 255 - lb;
      cptr[0] = 255;
      break;
    case 5:
      cptr[2] = 0;
      cptr[1] = 0;
      cptr[0] = 255 - lb;
      break;
    default:
      cptr[2] = 0;
      cptr[1] = 0;
      cptr[0] = 0;
      break;
    }
}

void writeCalibration(FILE *f, const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs)
{
  const double *K = cameraMatrix.ptr<double>();
  const double *D = distCoeffs.ptr<double>();
  fprintf(f, "image_width: 640\n");
  fprintf(f, "image_height: 480\n");
  fprintf(f, "camera_name: kinect\n");
  fprintf(f,
          "camera_matrix:\n"
          "   rows: 3\n"
          "   cols: 3\n"
          "   data: [ %.8f, %.8f, %.8f, %.8f, %.8f, %.8f, %.8f, %.8f, %.8f ]\n",
          K[0], K[1], K[2], K[3], K[4], K[5], K[6], K[7], K[8]);
  fprintf(f,
          "distortion_coefficients:\n"
          "   rows: 1\n"
          "   cols: 5\n"
          "   data: [ %.8f, %.8f, %.8f, %.8f, %.8f ]\n",
          D[0], D[1], D[2], D[3], D[4]);
  fprintf(f,
          "rectification_matrix:\n"
          "   rows: 3\n"
          "   cols: 3\n"
          "   data: [ 1., 0., 0., 0., 1., 0., 0., 0., 1. ]\n");
  fprintf(f,
          "projection_matrix:\n"
          "   rows: 3\n"
          "   cols: 4\n"
          "   data: [ %.8f, %.8f, %.8f, 0., %.8f, %.8f, %.8f, 0., %.8f, %.8f, %.8f, 0. ]\n",
          K[0], K[1], K[2], K[3], K[4], K[5], K[6], K[7], K[8]);
}

#define RADIUS 260

void pcdwrite(char *fname, Mat depth, float cx, float cy, float fx, float fy, float baseline, float shift_offset, float D = 1, float U = 0, float V = 0) {
  FILE *f = fopen(fname, "w");
  fprintf(f, "# .PCD v.7 - Point Cloud Data file format\n");
  fprintf(f, "VERSION .7\n");
  fprintf(f, "FIELDS x y z\n");
  fprintf(f, "SIZE 4 4 4\n");
  fprintf(f, "TYPE F F F\n");
  fprintf(f, "COUNT 1 1 1\n");
  fprintf(f, "WIDTH %d\n", depth.cols); // x
  fprintf(f, "HEIGHT %d\n", depth.rows); // y
  fprintf(f, "VIEWPOINT 0 0 0 1 0 0 0\n");
  fprintf(f, "POINTS %lu\n", depth.total());
  fprintf(f, "DATA ascii\n");

  for (int v = 0; v < depth.rows; v++) { // y
    for (int u = 0; u < depth.cols; u++) { // x
      float disparity = D*SHIFT_SCALE*(shift_offset-depth.at<uint16_t>(cv::Point2f(u,v))) + U*(u-cx)*(u-cx) + V*(v-cy)*(v-cy);
      if ( disparity <= 0 ) {
        fprintf(f, "%f %f %f\n", 0.0f, 0.0f, 0.0f);
      } else {
        float z = fx * baseline / disparity * 0.001;
        float x = (u - cx) / fx * z;
        float y = (v - cy) / fy * z;
        fprintf(f, "%f %f %f\n", x, y, z);
      }
    }
  }
  fclose(f);
}

void pcdwrite_chessboard(char *fname, vector<cv::Vec3f> patterns) {
  FILE *f = fopen(fname, "w");
  fprintf(f, "# .PCD v.7 - Point Cloud Data file format\n");
  fprintf(f, "VERSION .7\n");
  fprintf(f, "FIELDS x y z\n");
  fprintf(f, "SIZE 4 4 4\n");
  fprintf(f, "TYPE F F F\n");
  fprintf(f, "COUNT 1 1 1\n");
  fprintf(f, "WIDTH %lu\n", patterns.size());
  fprintf(f, "HEIGHT 1\n");
  fprintf(f, "VIEWPOINT 0 0 0 1 0 0 0\n");
  fprintf(f, "POINTS %lu\n", patterns.size());
  fprintf(f, "DATA ascii\n");

  for (unsigned int i = 0; i < patterns.size(); i++ ) {
	fprintf(f, "%f %f %f\n", patterns[i][0]*0.001, patterns[i][1]*0.001, patterns[i][2]*0.001);
  }
  fclose(f);
}

void undistort_nearest(Mat img, Mat &imgRect, Mat camMatrix, Mat distCoeffs) {
  Mat src = img, &dst = imgRect;
  dst.create( src.size(), src.type() );
  int stripe_size0 = std::min(std::max(1, (1 << 12) / std::max(src.cols, 1)), src.rows);
  Mat map1(stripe_size0, src.cols, CV_16SC2), map2(stripe_size0, src.cols, CV_16UC1);
  Mat_<double> A, Ar, I = Mat_<double>::eye(3,3);
  camMatrix.convertTo(A, CV_64F);
  A.copyTo(Ar);

  double v0 = Ar(1, 2);
  for( int y = 0; y < src.rows; y += stripe_size0 )
    {
      int stripe_size = std::min( stripe_size0, src.rows - y );
      Ar(1, 2) = v0 - y;
      Mat map1_part = map1.rowRange(0, stripe_size),
	    map2_part = map2.rowRange(0, stripe_size),
	    dst_part = dst.rowRange(y, y + stripe_size);

      initUndistortRectifyMap( A, distCoeffs, I, Ar, Size(src.cols, stripe_size),
                               map1_part.type(), map1_part, map2_part );
      remap( src, dst_part, map1_part, map2_part, INTER_NEAREST, BORDER_CONSTANT );
    }
}

double apply_fitting_param_to_kd(double kd, double u, double v, double shift_offset, double D, double U, double V)
{
  double disp_fitted = D*SHIFT_SCALE*(shift_offset-kd) + U*u*u + V*v*v;
  return( shift_offset - (disp_fitted/SHIFT_SCALE) );
}

// 
// read in IR images, perform monocular calibration, check distortion
// arguments:
//   [dir]          data directory (without trailing slash); default cwd
//   [cell size, m] size of edge of each square in chessboard
//   [#rows #cols]  number of rows and cols of interior chessboard;
//                  default 6x7
//

int
main(int argc, char **argv)
{
  // checkerboard pattern
  int ccols = 0;
  int crows = 0;

  // cell size
  double csize = 0.0;

  char *fdir = NULL;

  opterr = 0;
  int c;
  while ((c = getopt(argc, argv, "r:c:s:d:")) != -1)
    {
      switch (c)
        {
        case 'r':
          crows = atoi(optarg);
          break;
        case 'c':
          ccols = atoi(optarg);
          break;
        case 's':
          csize = atof(optarg);
          break;
        }
    }

  if (optind < argc)
    fdir = argv[optind];

  if (crows == 0 || ccols == 0 || csize == 0.0 || fdir == NULL)
    {
      printf("Must give the checkerboard dimensions and data directory.\n"
             "Usage:\n"
             "%s -r ROWS -c COLS -s SQUARE_SIZE my_data_dir\n", argv[0]);
      return 1;
    }

  // construct the planar pattern
  vector<Point3f> pat;
  for (int i=0; i<ccols; i++)
    for (int j=0; j<crows; j++)
      pat.push_back(Point3f(i*csize,j*csize,0));

  // read in images, set up feature points and patterns
  vector<vector<Point3f> > pats;
  vector<vector<Point2f> > points;

  int fnum = 0;

  while (1)
    {
      char fname[1024];
      sprintf(fname,"%s/img_ir_%02d.png",fdir,fnum++);
      Mat img = imread(fname,-1);
      if (img.data == NULL) break; // no data, not read, break out

      vector<cv::Point2f> corners;
      bool ret = cv::findChessboardCorners(img,Size(crows,ccols),corners);

      if (ret)
        printf("Found corners in image %s\n",fname);
      else {
        printf("*** Didn't find corners in image %s\n",fname);
        return 1;
      }

      cv::cornerSubPix(img, corners, Size(5,5), Size(-1,-1),
                       TermCriteria(TermCriteria::MAX_ITER+TermCriteria::EPS, 30, 0.1));

      // Adjust corners detected in IR image to where they would appear in the depth image
      for (unsigned int i = 0; i < corners.size(); ++i)
        corners[i] += ir_depth_offset;

      pats.push_back(pat);
      points.push_back(corners);
    }


  // Monocular calibration of depth camera
  Mat camMatrix;
  Mat distCoeffs;
  vector<Mat> rvecs;
  vector<Mat> tvecs;
  double rp_err;
  rp_err = calibrateCamera(pats, points, Size(COLS,ROWS), camMatrix, distCoeffs,
                           rvecs, tvecs,
                           CV_CALIB_FIX_K3 | 
                           //CV_CALIB_FIX_K2 | 
                           //CV_CALIB_FIX_K1 | 
                           CV_CALIB_ZERO_TANGENT_DIST //|
                           //CV_CALIB_FIX_PRINCIPAL_POINT |
                           //CV_CALIB_FIX_ASPECT_RATIO
                           );

  printf("\nCalibration results:\n");

  // print camera matrix
  printf("\nCamera matrix\n");
  double *dptr = camMatrix.ptr<double>(0);
  for (int i=0; i<3; i++)
    {
      for (int j=0; j<3; j++)
        printf("%f ",*dptr++);
      printf("\n");
    }
  dptr = distCoeffs.ptr<double>(0);
  printf("\nDistortion coefficients:\n"
         "k1: %f\n"
         "k2: %f\n"
         "t1: %f\n"
         "t2: %f\n"
         "k3: %f\n", dptr[0], dptr[1], dptr[2], dptr[3], dptr[4]);
  
  printf("\nReprojection error = %f\n\n", rp_err);

  char depth_fname[1024];
  sprintf(depth_fname, "%s/calibration_depth.yaml", fdir);
  FILE *depth_file = fopen(depth_fname, "w");
  if (depth_file) {
    writeCalibration(depth_file, camMatrix, distCoeffs);
    printf("Wrote depth camera calibration to %s\n\n", depth_fname);
  }
  
  // Read in depth images, fit readings to computed depths
  /// @todo Not checking that we actually got depth readings!
  fnum = 0;
  std::vector<cv::Vec2d> ls_src1;
  std::vector<double> ls_src2;
  std::vector<cv::Point2f> ls_src3;
  while (1)
    {
      // Load raw depth readings
      char fname[1024];
      sprintf(fname,"%s/img_depth_%02d.png",fdir,fnum);
      Mat img_depth = imread(fname,-1);
      if (img_depth.data == NULL) break; // no data, not read, break out
      Mat img_depth_rect;
      undistort_nearest(img_depth, img_depth_rect, camMatrix, distCoeffs);

      // Get corner points and extrinsic parameters
      const cv::Mat pattern(pats[fnum]); // 3-channel matrix view of vector<Point3f>
      vector<Point2f> &corners = points[fnum];
      cv::Mat rvec = rvecs[fnum];
      cv::Mat tvec = tvecs[fnum];
      cv::Mat rot3x3;
      cv::Rodrigues(rvec, rot3x3);

      // Transform object points into camera coordinates using (rvec, tvec)
      cv::Mat world_points;
      cv::Mat xfm(3, 4, cv::DataType<double>::type);
      cv::Mat xfm_rot = xfm.colRange(0,3);
      cv::Mat xfm_trans = xfm.col(3);
      rot3x3.copyTo(xfm_rot);
      tvec.reshape(1,3).copyTo(xfm_trans);
      cv::transform(pattern, world_points, xfm);
 
      for (unsigned int j = 0; j < corners.size(); ++j) {
        double Z = world_points.at<cv::Vec3f>(j)[2];   // actual depth
        double r = img_depth_rect.at<uint16_t>(corners[j]); // sensor reading
        if ( (r < 2047.0) // 2047 is invalid data
             && (Z < 1.0) // use near data
             ){
          ls_src1.push_back(cv::Vec2d(-1.0, Z));
          ls_src2.push_back(Z*r);
          ls_src3.push_back(corners[j]);
        }
      }

      fnum++;
    }

  double A = 200, B = 1080;
  double b; // baseline
  double prev_A = 0, prev_B = 0;

  // calibrate projector distortion
  double dd = 1, U = 0, V = 0;
  double prev_dd = 0, prev_U = 0, prev_V = 0;

  double thr_A = 1, thr_B = 1, thr_dd = 0.01, thr_U = 1e-5, thr_V = 1e-5;

  std::vector<double> ls_src2_buf = ls_src2; //buffer

  while ( (abs(A - prev_A) > thr_A)
          || (abs(B - prev_B) > thr_B)
          || (abs(dd - prev_dd) > thr_dd)
          || (abs(U - prev_U) > thr_U)
          || (abs(V - prev_V) > thr_V) ){

    prev_A = A;
    prev_B = B;
    prev_dd = dd;
    prev_U = U;
    prev_V = V;

    // update sensor value with fitting params
    for(unsigned int i = 0; i < ls_src2.size(); i++){
      double Z = ls_src1[i][1];
      double r = ls_src2_buf[i]/Z;
      double u = ls_src3[i].x - camMatrix.at<double>(0,2);
      double v = ls_src3[i].y - camMatrix.at<double>(1,2);
      ls_src2[i] = apply_fitting_param_to_kd(r, u, v, B, dd, U, V) * Z;
    }
    
    {
      cv::Mat depth_params;
      if (cv::solve(cv::Mat(ls_src1).reshape(1), cv::Mat(ls_src2), depth_params,
                    DECOMP_LU | DECOMP_NORMAL)) {
        A = depth_params.at<double>(0);
        B = depth_params.at<double>(1);
        double f = camMatrix.ptr<double>()[0];
        b = SHIFT_SCALE * A / f;
        printf("Reading to depth fitting parameters:\n"
               "A = %f\n"
               "B = %f\n"
               "Baseline between projector and depth camera = %f\n\n",
               A, B, b);
        double rp_err = 0;
        // -A + BZ = Zr, Z = -A/(r - B), r = -A/Z + B
        for(unsigned int i = 0; i < ls_src2.size() ; i++){
          double Z = ls_src1[i][1];
          double r = ls_src2[i]/Z;
          rp_err += abs(Z + A / ( r - B ));
        }
        rp_err /= ls_src2.size();
        printf("\nReprojection error = %f\n\n", rp_err);
        std::cerr << depth_params << std::endl;
      }
      else {
        printf("**** Failed to solve least-squared problem ****\n");
        return 1;
      }
    }

    std::vector<cv::Vec3d> ls_src10;
    std::vector<double> ls_src11;
    double f = camMatrix.at<double>(0,0);

    for(unsigned int i = 0; i < ls_src2.size() ; i++){
      double Z = ls_src1[i][1];
      double r = ls_src2[i]/Z;
      double u = ls_src3[i].x - camMatrix.at<double>(0,2);
      double v = ls_src3[i].y - camMatrix.at<double>(1,2);
      ls_src10.push_back(cv::Vec3d( (SHIFT_SCALE * (B-r)), u*u, v*v));
      ls_src11.push_back(b*f/Z);
    }
    cv::Mat depth_params;
    double rp_err1 = 0, rp_err2 = 0;
    if (cv::solve(cv::Mat(ls_src10).reshape(1), cv::Mat(ls_src11), depth_params,
                  DECOMP_LU | DECOMP_NORMAL)) {
      dd = depth_params.at<double>(0);
      U = depth_params.at<double>(1);
      V = depth_params.at<double>(2);
      std::cerr << "D = " << dd << ", U = " << U << ", V = " << V << std::endl;

      for(unsigned int i = 0; i < ls_src2.size() ; i++){
        double Z = ls_src1[i][1];
        double r = ls_src2[i]/Z;
        double r_buf = ls_src2_buf[i]/Z;
        double u = ls_src3[i].x - camMatrix.at<double>(0,2);
        double v = ls_src3[i].y - camMatrix.at<double>(1,2);
        rp_err1 += fabs(Z + A / (r_buf - B));
        rp_err2 += fabs(Z + b*f/((SHIFT_SCALE*(r-B))+U*u*u+V*v*v) );
      }
      rp_err1 /= ls_src2.size();
      rp_err2 /= ls_src2.size();
      printf("\nReprojection error will change from %f to %f by hyperboloid fitting\n\n", rp_err1, rp_err2);
    }
  }

  // 
  // calibrate IR to RGB images
  //

  // read in rgb files
  fnum = 0;
  vector<vector<Point2f> > pointsRGB; // RGB corners
  printf("\n");
  while (1)
    {
      char fname[1024];
      sprintf(fname,"%s/img_rgb_%02d.png",fdir,fnum);
      Mat img = imread(fname,1);
      if (img.data == NULL) break; // no data, not read, break out

      vector<cv::Point2f> corners;
      bool ret = cv::findChessboardCorners(img,Size(crows,ccols),corners);

      if (ret)
        printf("Found corners in image %s\n",fname);
      else {
        printf("*** Didn't find corners in image %s\n",fname);
        return 1;
      }

      Mat gray;
      cv::cvtColor(img, gray, CV_RGB2GRAY);
      cv::cornerSubPix(gray, corners, Size(5,5), Size(-1,-1),
                       TermCriteria(TermCriteria::MAX_ITER+TermCriteria::EPS, 30, 0.1));

      pointsRGB.push_back(corners);

      fnum++;
    }

  // calibrate monocular camera
  Mat camMatrixRGB;
  Mat distCoeffsRGB = Mat::zeros(5,1,CV_64F);
  // initialize camera matrix
  camMatrixRGB = (Mat_<double>(3,3) << 1, 0, 320, 0, 1, 240, 0, 0, 1);

  vector<Mat> rvecs_rgb;
  vector<Mat> tvecs_rgb;
  rp_err = calibrateCamera(pats, pointsRGB, Size(COLS,ROWS), camMatrixRGB, distCoeffsRGB,
                           rvecs_rgb, tvecs_rgb,
                           //CV_CALIB_FIX_K1 |
                           //CV_CALIB_FIX_K2 |
                           CV_CALIB_FIX_K3 |
                           CV_CALIB_ZERO_TANGENT_DIST |
                           //CV_CALIB_FIX_PRINCIPAL_POINT |
                           CV_CALIB_FIX_ASPECT_RATIO
                           );

  // distortion results
  printf("\nCalibration results:\n");

  // print camera matrix
  printf("\nCamera matrix\n");
  dptr = camMatrixRGB.ptr<double>(0);
  for (int i=0; i<3; i++)
    {
      for (int j=0; j<3; j++)
        printf("%f ",*dptr++);
      printf("\n");
    }

  dptr = distCoeffsRGB.ptr<double>(0);
  printf("\nDistortion coefficients:\n"
         "k1: %f\n"
         "k2: %f\n"
         "t1: %f\n"
         "t2: %f\n"
         "k3: %f\n", dptr[0], dptr[1], dptr[2], dptr[3], dptr[4]);
  
  printf("\nReprojection error = %f\n\n", rp_err);

  char rgb_fname[1024];
  sprintf(rgb_fname, "%s/calibration_rgb.yaml", fdir);
  FILE *rgb_file = fopen(rgb_fname, "w");
  if (rgb_file) {
    writeCalibration(rgb_file, camMatrixRGB, distCoeffsRGB);
    printf("Wrote RGB camera calibration to %s\n\n", rgb_fname);
  }

  // stereo calibration between IR and RGB
  Mat R,T,E,F;
  rp_err = stereoCalibrate(pats,points,pointsRGB,camMatrix,distCoeffs,
                           camMatrixRGB,distCoeffsRGB,Size(crows,ccols),
                           R,T,E,F);
  
  dptr = T.ptr<double>(0);
  printf("\nTranslation between depth and RGB sensors (m):\n");
  for (int i=0; i<3; i++)
    printf("%f ",dptr[i]);
  printf("\n");

  printf("\nRotation matrix:\n");
  dptr = R.ptr<double>(0);
  for (int i=0; i<3; i++)
    {
      for (int j=0; j<3; j++)
        printf("%f ",*dptr++);
      printf("\n");
    }
  printf("\nReprojection error = %f\n\n", rp_err);


  Matrix4d Q,S;                 // transformations
  Matrix<double,3,4> P;         // projection

  // from u,v,d of depth camera to XYZ
  double *cptr = camMatrix.ptr<double>(0);
  Q << 1, 0, 0,    -cptr[2],  // -cx
    0, 1, 0,    -cptr[5],  // -cy
    0, 0, 0,     cptr[0],  // focal length
    0, 0, 1.0/b, 0;        // baseline

  std::cerr << "from u,v,d of depth camera to XYZ : " << std::endl << Q << std::endl << std::endl;

  // from XYZ of depth camera to XYZ of RGB camera
  dptr = R.ptr<double>(0);
  double *tptr = T.ptr<double>(0);
  S << dptr[0], dptr[1], dptr[2], tptr[0],
    dptr[3], dptr[4], dptr[5], tptr[1],
    dptr[6], dptr[7], dptr[8], tptr[2],
    0,       0,       0,       1;
  std::cerr << "from XYZ of depth camera to XYZ of RGB camera : " << std::endl << S << std::endl << std::endl;

  // from XYZ to u,v in RGB camera
  cptr = camMatrixRGB.ptr<double>(0);
  P << cptr[0], 0,       cptr[2], 0,
    0,       cptr[4], cptr[5], 0,
    0,       0,       1,       0;
  std::cerr << "from XYZ to u,v in RGB camera : " << std::endl << P << std::endl << std::endl;

  Matrix<double,3,4> D = P*S*Q;
  std::cout << "Transform matrix:" << std::endl << D << std::endl << std::endl;

  char params_fname[1024];
  sprintf(params_fname, "%s/kinect_params.yaml", fdir);
  FILE *params_file = fopen(params_fname, "w");
  if (params_file) {
    fprintf(params_file, "shift_offset: %.4f\n", B);
    fprintf(params_file, "projector_depth_baseline: %.5f\n", b);
    dptr = R.ptr<double>(0);
    fprintf(params_file,
            "depth_rgb_rotation: [ %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f ]\n",
            dptr[0], dptr[1], dptr[2], dptr[3], dptr[4], dptr[5], dptr[6], dptr[7], dptr[8]);
    dptr = T.ptr<double>(0);
    fprintf(params_file,
            "depth_rgb_translation: [ %.6f, %.6f, %.6f ]\n", dptr[0], dptr[1], dptr[2]);
    fprintf(params_file,
            "projector_coefficients:\n"
            "    u_coeff: %.8e\n"
            "    v_coeff: %.8e\n"
            "    disp_coeff: %.8f\n",
            U, V, dd);
    printf("Wrote additional calibration parameters to %s\n", params_fname);
  }
  
  //
  // create rectified disparity images and save
  //

  // set up gamma for depth colorizer
  for (int i = 0; i < 2048; i++)
    {
      float v = i / 2048.0;
      v = powf (v, 3) * 6;
      t_gamma[i] = v * 6 * 256;
    }


  fnum = 0;
  printf("Creating output images\n");
  while (1)
    {
      char fname[1024];
      sprintf(fname,"%s/img_depth_%02d.png",fdir,fnum);
      Mat img = imread(fname,-1);
      if (img.data == NULL) break; // no data, not read, break out

      // Rectify Depth image
      cv::Mat imgRect;
      undistort_nearest(img, imgRect, camMatrix, distCoeffs);

      sprintf(fname,"%s/img_rgb_%02d.png",fdir,fnum);
      Mat imgRGB = imread(fname,1);
      if (imgRGB.data == NULL) break; // no data, not read, break out

      // Rectify RGB image
      cv::Mat imgRgbRect;
      cv::undistort(imgRGB, imgRgbRect, camMatrixRGB, distCoeffsRGB);

      sprintf(fname,"%s/img_ir_%02d.png",fdir,fnum);
      Mat imgIr = imread(fname,-1);
      if (imgIr.data == NULL) break; // no data, not read, break out

      // Rectify IR image
      cv::Mat imgIrRect;
      cv::undistort(imgIr, imgIrRect, camMatrix, distCoeffs);
      //imgIrRect = imgIr;

      uint16_t *dptr = img.ptr<uint16_t>(0);
      uint16_t *drptr = imgRect.ptr<uint16_t>(0);
      uint8_t *irptr = imgIrRect.ptr<uint8_t>(0);

      Mat imgr  = Mat::zeros(ROWS,COLS,CV_16UC1); // depth image mapped to RGB image
      Mat imgrc = Mat::zeros(ROWS,COLS,CV_8UC3); // depth image mapped to RGB image, colorized
      Mat imgc  = Mat::zeros(ROWS,COLS,CV_8UC3); // original depth image colorized
      Mat imgcr  = Mat::zeros(ROWS,COLS,CV_8UC3); // original rectified depth image colorized
      Mat imgdc = Mat::zeros(ROWS,COLS,CV_8UC3); // RGB mapped to depth image
      Mat imgdi = Mat::zeros(ROWS,COLS,CV_8UC1); // IR mapped to depth image

      uint16_t *rptr = imgr.ptr<uint16_t>(0);
      uint8_t *rcptr = imgrc.ptr<uint8_t>(0);
      uint8_t *cptr  = imgc.ptr<uint8_t>(0);
      uint8_t *crptr = imgcr.ptr<uint8_t>(0);
      uint8_t *dcptr = imgdc.ptr<uint8_t>(0);
      uint8_t *diptr = imgdi.ptr<uint8_t>(0);
      uint8_t *rgbptr = imgRgbRect.ptr<uint8_t>(0);

      int k=0;
      for (int i=0; i<ROWS; i++)
        for (int j=0; j<COLS; j++,k++) // k is depth image index
          {
            double d = shift2disp(drptr[k], B);
            if (d <= 0)
              d = 0.0;          // not valid
            Vector4d p;
            p << j,i,d,1;
            Vector3d q;
            q = D*p;
            int u = (int)(q[0]/q[2]+0.5);
            int v = (int)(q[1]/q[2]+0.5);
            setDepthColor(&cptr[3*(i*COLS+j)],dptr[k]);
            setDepthColor(&crptr[3*(i*COLS+j)],drptr[k]);
            if (u < 0 || v < 0 || u >= COLS || v >= ROWS)
              continue;
            int disp = (int)(d*16+0.499);
            int kk = v*COLS+u;  // kk is corresponding RGB image index
            if (rptr[kk] < disp) // z-buffer check
              {
                rptr[kk] = disp;
                setDepthColor(&rcptr[3*kk],drptr[k]);
              }
            if (d != 0.0) {
              memcpy(&dcptr[3*k],&rgbptr[3*kk],3); // RGB mapped to depth image
              diptr[k]=irptr[k];                  // IR mapped to depth image
            }
          }

      sprintf(fname,"%s/img_depth_rect_%02d.png",fdir,fnum);
      printf("Writing %s\n", fname);
      imwrite(fname,imgr);
      sprintf(fname,"%s/img_depth_rect_color_%02d.png",fdir,fnum);
      printf("Writing %s\n", fname);
      imwrite(fname,imgrc);
      sprintf(fname,"%s/img_depth_color_%02d.png",fdir,fnum);
      printf("Writing %s\n", fname);
      imwrite(fname,imgc);
      sprintf(fname,"%s/img_depth_color_rect_%02d.png",fdir,fnum);
      printf("Writing %s\n", fname);
      imwrite(fname,imgcr);
      sprintf(fname,"%s/img_rgb_mapped_%02d.png",fdir,fnum);
      printf("Writing %s\n", fname);
      imwrite(fname,imgdc);
      sprintf(fname,"%s/img_ir_mapped_%02d.png",fdir,fnum);
      printf("Writing %s\n", fname);
      imwrite(fname,imgdi);
      sprintf(fname,"%s/img_rgb_rect_%02d.png",fdir,fnum);
      printf("Writing %s\n", fname);
      imwrite(fname,imgRgbRect);
      sprintf(fname,"%s/img_ir_rect_%02d.png",fdir,fnum);
      printf("Writing %s\n", fname);
      imwrite(fname,imgIrRect);

      sprintf(fname,"%s/depth_%02d.pcd",fdir,fnum);
      printf("Writing %s\n", fname);
      pcdwrite(fname,img,
               camMatrix.at<double>(0,2), // cx
               camMatrix.at<double>(1,2), // cy
               camMatrix.at<double>(0,0), // focal length
               camMatrix.at<double>(1,1), // focal length
               b,      // baseline
               B      // shift offset
               );

      sprintf(fname,"%s/depth_rect_%02d.pcd",fdir,fnum);
      printf("Writing %s\n", fname);
      pcdwrite(fname,imgRect,
               camMatrix.at<double>(0,2), // cx
               camMatrix.at<double>(1,2), // cy
               camMatrix.at<double>(0,0), // focal length
               camMatrix.at<double>(1,1), // focal length
               b,      // baseline
               B,      // shift offset
               dd, U, V // projector parameters
               );
      fnum++;
    }

  // write calibration board
  for (int i = 0 ; i < fnum; i++ ) {
    // Get corner points and extrinsic parameters
    const cv::Mat pattern(pats[i]); // 3-channel matrix view of vector<Point3f>
    vector<Point2f> &corners = points[i];
    cv::Mat rvec = rvecs[i];
    cv::Mat tvec = tvecs[i];
    cv::Mat rot3x3;
    cv::Rodrigues(rvec, rot3x3);

    // Transform object points into camera coordinates using (rvec, tvec)
    cv::Mat world_points;
    cv::Mat xfm(3, 4, cv::DataType<double>::type);
    cv::Mat xfm_rot = xfm.colRange(0,3);
    cv::Mat xfm_trans = xfm.col(3);
    rot3x3.copyTo(xfm_rot);
    tvec.reshape(1,3).copyTo(xfm_trans);
    cv::transform(pattern, world_points, xfm);

    vector<cv::Vec3f> patterns;
    for (unsigned int j = 0; j < corners.size(); ++j) {
      patterns.push_back(world_points.at<cv::Vec3f>(j));
    }
    char fname[1024];
    sprintf(fname,"%s/chess_%02d.pcd",fdir, i);
    printf("Writing %s\n", fname);
    pcdwrite_chessboard(fname, patterns);
  }

  return 0;
}
