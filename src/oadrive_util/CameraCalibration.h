// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// This program is free software licensed under the CDDL
// (COMMON DEVELOPMENT AND DISTRIBUTION LICENSE Version 1.0).
// You can find a copy of this license in LICENSE in the top
// directory of the source code.
//
// © Copyright 2018 FZI Forschungszentrum Informatik, Karlsruhe, Germany
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Peter Zimmer <peter-zimmer@gmx.net>
 * \author  Micha Pfeiffer <ueczz@student.kit.edu>
 * \date    2015-11-24
 *
 */
//----------------------------------------------------------------------

#ifndef OADRIVE_UTIL_CAMERACALIBRATION_H
#define OADRIVE_UTIL_CAMERACALIBRATION_H
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
//#include <opencv2/imgcodecs.hpp>
#include <string.h>
namespace oadrive {
namespace util {

/*!
  \brief This class generates generate a warp matrix to perform a birdview conversion. It writes also the pixel/m and the car coordinates of down left corner.
  It also estimates the camera postion (including yaw pitch roll). (Only with OpenCV 3) This postion is needed to generate the refernce Image for the depth image.


A Note on Coordinate systems:
/////////////////////////////////////////
Car coordinate system:
The origin is between the car's rear wheels (center of the rear axis).
X increases towards the right of the car, Y increases towards the front of the car.
          ^ y
          |
          |
          |
        / | \
        | | |
        \ |-/---------> x

yaw of 0 points upwards towards y.

//////////////////////////////////////////
Bird View (image) coordinate system:
Top left of image is origin, x increases towards the right, y increases towards the bottom (similar to OpenCV).
-------------> x
|
|
|
|
| y

//////////////////////////////////////////
World coordinate system:

| y
|
|
|
|
|
-------------> x

Yaw of zero points towards positive x. This is due to the way oadrive_control/LateralController uses the yaw
angle.

*/
class CameraCalibration
{
public:
  CameraCalibration();
  //!search for circle calibration pattern to define the right points
  //@param img image with number of circles x hight number of circles y
  //@return success = true
  bool autoCalPoints(cv::Mat img);

  //!draw calibration Points
  cv::Mat drawCalPoints(cv::Mat img);

  //!transform the image into Birdview
  cv::Mat transform(cv::Mat image);

  //!read Camera Cal file to undistort the image
  void readCameraCalFile (std::string path);

  //!save calibration file
  void writeCalFile (std::string path);

  //!estimate camera Pose
  void estimateCameraPose();

  //!static CallBack Function. It calls the member Callback
  static void CallBackFuncSliderScale(int event,void* userdata);
  //!Member Slider Scale Callback
  void CallBackFuncSliderScale(int event);
  OADRIVE_UTIL_CAMERACALIBRATION_H
  //!static CallBack Function. It calls the member Callback
  static void CallBackFuncSliderOffsetY(int event,void* userdata);
  //!Member Slider Offset Y Callback
  void CallBackFuncSliderOffsetY(int event);

  double getMPerPixel();

  //! \brief adjustPicture Generate a Gui to adjust the birdview (scale and y offset)
  void adjustPicture(cv::Mat img);

private:
  //!state of calibration
  int m_calibrationState;
  //!ratio of the Calibration Pattern
  double mRatio;
  //! y offset in pixel
  int mYOffset;
  //! y offset SLider
  int mYOffsetSlider;
  //! Scaling of the picture
  double mScale;
  //!slider Value Scale
  int mScaleSlider;
  //!distance of 2 Points in the Calibration pattern in m center to center
  double mDistance2PointM;
  //!distance front of the car to the middle of the first line in the calibration pattern in m
  double mDistanceCarToCalPattern;
  //!coordinates of the bottom left corner of the birdview relative to the center of the front bumber
  cv::Point2f mCoordinatesBirdView;
  //!m per pixel in the Birdview
  double mMPerPixel;
  //!number of Points of the pattern (width)
  int mPointsWidth;
  //!number of Points of the pattern (height)
  int mPointsHeight;

  //! size of the picture
  cv::Size mImgSize;
  //! Path to the complete calibration FIle
  std::string mCompleteCalFile;



  //! Calibration Points in the src Picture
#define numberOfCalPoints 4
  cv::Point2f mCalPoints[numberOfCalPoints];
  //! Destination Vertices in the Bird View
  cv::Point2f mDestVertices[4];



  //! Matrix to disort image
  cv::Mat mCameraMatrix, mDistCoeffs;
  //!Matrix to transform the Image
  cv::Mat mWrapMatrix;
  //!CameraCal available
  bool mCameraCalAvailable;


  //! \note xAxis to the right yAxis depth and zAxis height
  //!angle x axis
  double mAngleX;
  //!angle y axis
  double mAngleY;
  //!andle z axis
  double mAngleZ;

  double mCamPosX;
  double mCamPosY;
  double mCamPosZ;

  //!load camera cal file and remove camera disturbance
  cv::Mat unDisortImg(cv::Mat img);
  //!calculate the position of the Birdview in the car coordinates
  void calcPosBirdviewInCarGrid();
  //!read Camera Calibration File from Disk


  //!calculate the warpmatrix
  //**This function calculates the wrapmatrix out of 4 given points of a quadrat.	*/
  void calculateWrapMatrix();


  void setYOffset(int YOffset);
  void setScale(int scale);
  cv::Mat mAdjustImage;

  const std::string mWindowNameScale;
};

//work arround for int to string
namespace patch
{
template < typename T > std::string to_string( const T& n )
{
  std::ostringstream stm ;
  stm << n ;
  return stm.str() ;
}
}
}
}
#endif // OADRIVE_UTIL_CAMERACALIBRATION_H
