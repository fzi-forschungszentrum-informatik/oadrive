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

#include "CameraCalibration.h"
#include "utilLogging.h"
#include <iostream>
#include <vector>
#include "opencv2/core/version.hpp"
using icl_core::logging::endl;
using icl_core::logging::flush;
namespace oadrive {
namespace util {
CameraCalibration::CameraCalibration():
  mWindowNameScale("AdjustScale")
{
  mRatio = 1;
  mYOffset = 400;
  mScale = 1;
  mPointsHeight = 6;
  mPointsWidth = 7;
  mRatio = ((double)mPointsHeight-1.0)/((double)mPointsWidth-1.0);
  //mRatio = 0.57286f;    // Measured ratio of calibration pattern
  LOGGING_INFO( utilLogger, "[CameraCalibration] mRatio" << mRatio << endl );
  mDistance2PointM = 0.066;      // Distance between two points in calibration pattern
  // mDistance2PointM = 0.05;      // Distance between two points in calibration pattern
  //mDistanceCarToCalPattern = 0.276;
  mDistanceCarToCalPattern = 0.276 + 0.502;
  mCompleteCalFile = "/home/aadc/AADC/src/aadcUser/config/Goffin/BirdviewCal_lowRes.yml";
  mCameraCalAvailable = false;
}


void CameraCalibration::writeCalFile (std::string path)
{
  cv::FileStorage fs(path, cv::FileStorage::WRITE);
  if(mCameraCalAvailable == true)
  {
    fs << "camera_matrix" << mCameraMatrix << "distortion_coefficients" << mDistCoeffs<<"warpMatrix"<<mWrapMatrix<<"Ratio"
       <<mRatio<<"yOffset"<<mYOffset<<"scale"<<mScale<<"CalPoints0"<<mCalPoints[0]<<"CalPoints1"<<mCalPoints[1]
      <<"CalPoints2"<<mCalPoints[2]<<"CalPoints3"<<mCalPoints[3]<<"Distance2PointM"<<mDistance2PointM
     <<"DistanceCarToCalPattern"<<mDistanceCarToCalPattern<<"CoordinatesBirdView"<<mCoordinatesBirdView
    <<"MPerPixel"<<mMPerPixel<<"PointsHeight"<<mPointsHeight<<"PointsWidth"<<mPointsWidth<<"ImgSize"<<mImgSize
    <<"mCameraPosX"<<mCamPosX<<"mCameraPosY"<<mCamPosY<<"mCameraPosZ"<<mCamPosZ
    <<"mCameraPitch"<<mAngleX<<"mCameraRoll"<<mAngleY<<"mCameraYaw"<<mAngleZ;
    std::cout<<"saved complete calibration file"<<std::endl;
  }
  else
  {
    fs << "warpMatrix"<<mWrapMatrix<<"Ratio"
       <<mRatio<<"yOffset"<<mYOffset<<"scale"<<mScale<<"CalPoints0"<<mCalPoints[0]<<"CalPoints1"<<mCalPoints[1]
      <<"CalPoints2"<<mCalPoints[2]<<"CalPoints3"<<mCalPoints[3]<<"Distance2PointM"<<mDistance2PointM
     <<"DistanceCarToCalPattern"<<mDistanceCarToCalPattern<<"CoordinatesBirdView"<<mCoordinatesBirdView
    <<"MPerPixel"<<mMPerPixel<<"PointsHeight"<<mPointsHeight<<"PointsWidth"<<mPointsWidth<<"ImgSize"<<mImgSize;
    std::cout<<"saved UNcomplete calibration file; CameraMatrix and Distortion coefficents are missing"<<std::endl;
  }
  std::cout<<"Save path: "<<path<<std::endl;

}

//inspiered from:
//http://stackoverflow.com/questions/18637494/camera-position-in-world-coordinate-from-cvsolvepnp
void CameraCalibration::estimateCameraPose()
{

  std::vector<cv::Point3f> realPoints;
  //    realPoints.push_back( cv::Point3f(-mDistance2PointM*((mPointsWidth/2)-0.5),mDistanceCarToCalPattern+mDistance2PointM*((mPointsHeight/2)-0.5),0));
  //    realPoints.push_back( cv::Point3f(mDistance2PointM*((mPointsWidth/2)-0.5),mDistanceCarToCalPattern+mDistance2PointM*((mPointsHeight/2)-0.5),0));
  //    realPoints.push_back( cv::Point3f(-mDistance2PointM*((mPointsWidth/2)-0.5),mDistanceCarToCalPattern,0));
  //    realPoints.push_back( cv::Point3f(mDistance2PointM*((mPointsWidth/2)-0.5),mDistanceCarToCalPattern,0));
  //note: OpenCV thinks in a other way X goes Right y goes UP and z is Depth
  realPoints.push_back( cv::Point3f(mDistance2PointM*((mPointsWidth/2)-0.5),0,mDistanceCarToCalPattern+mDistance2PointM*((mPointsHeight/2)-0.5)));
  realPoints.push_back( cv::Point3f(-mDistance2PointM*((mPointsWidth/2)-0.5),0,mDistanceCarToCalPattern+mDistance2PointM*((mPointsHeight/2)-0.5)));
  realPoints.push_back( cv::Point3f(mDistance2PointM*((mPointsWidth/2)-0.5),0,mDistanceCarToCalPattern));
  realPoints.push_back( cv::Point3f(-mDistance2PointM*((mPointsWidth/2)-0.5),0,mDistanceCarToCalPattern));
  std::cout<<"Real Points"<<realPoints<<std::endl;

  std::vector<cv::Point2f> imagePoints;
  imagePoints.push_back(mCalPoints[0]);
  imagePoints.push_back(mCalPoints[1]);
  imagePoints.push_back(mCalPoints[2]);
  imagePoints.push_back(mCalPoints[3]);
  cv::Mat rvec(cv::Size(3,1),CV_32F);
  cv::Mat tvec(cv::Size(3,1),CV_32F);

#if CV_MAJOR_VERSION == 2
  std::cerr<<"You are using OpenCv2. You can't estimate the camerapose with OpenCV 2"<<std::endl
          <<"If you only want a Birdview you can deactivate this function. (Camera Pose is needed for depth image)"<<std::endl
            << "Maybe you can insert the camera pose by your self into the calibration file"<<std::endl;
  throw;
#elif CV_MAJOR_VERSION == 3
//  cv::solvePnP(realPoints,imagePoints,mCameraMatrix,mDistCoeffs,rvec, tvec,false,cv::SOLVEPNP_DLS);
#endif

  cv::Mat R;
  cv::Rodrigues(rvec, R); // R is 3x3

  R = R.t();  // rotation of inverse
  tvec = -R * tvec; // translation of inverse
  double r11 = R.at<double>(0,0);
  double r21 = R.at<double>(1,0);
  double r31 = R.at<double>(2,0);
  double r32 = R.at<double>(2,1);
  double r33 = R.at<double>(2,2);

  //we will NOT store in opencv cord system
  mAngleX = std::atan2(r32,r33);
  mAngleZ = std::atan2(r21,r11);
  mAngleY = std::atan2(-r31,std::sqrt(r32*r32+r33*r33));
  mCamPosX = tvec.at<double>(0);
  mCamPosY = tvec.at<double>(2);
  mCamPosZ = tvec.at<double>(1);
  std::cout<<"angle X: "<<mAngleX*(180/M_PI)<<std::endl;
  std::cout<<"angle Y: "<<mAngleY*(180/M_PI)<<std::endl;
  std::cout<<"angle Z: "<<mAngleZ*(180/M_PI)<<std::endl;
  std::cout<<"CamPos X:"<<mCamPosX<<std::endl;
  std::cout<<"CamPos Y:"<<mCamPosY<<std::endl;
  std::cout<<"CamPos Z:"<<mCamPosZ<<std::endl;
  //    std::cout<<"Translation Vector: "<<std::endl<<tvec<<std::endl<<"Rotation Vector: "<<std::endl<<rvec<<std::endl;

}

void  CameraCalibration::calculateWrapMatrix()
{
  std::cout<<"[BirdViewCalibration] mRatio"<<mRatio<<std::endl;
  std::cout<<"[BirdViewCalibration]source Points"<<std::endl;
  std::cout<<mCalPoints[0]<<std::endl;
  std::cout<<mCalPoints[1]<<std::endl;
  std::cout<<mCalPoints[2]<<std::endl;
  std::cout<<mCalPoints[3]<<std::endl;


  //calculate destination pixels. We suppose following pattern:
  /*				x
   * Point0			Point1
   *
   *Y
   *
   * Point2			Point3
   *
   * in the real world the points must be quadratic and parallel to the car
   */



  int distance2Points;
  distance2Points = cv::norm(mCalPoints[2]-mCalPoints[3]);
  distance2Points = distance2Points*mScale;
  //Point 2 will be distance2Points/2 left of the center
  mDestVertices[2] = cv::Point(mImgSize.width/2-(distance2Points/2),mYOffset);
  //Point 3 will have the same y coordinates as point 2 and the same x coordinates as in the src img
  mDestVertices[3] = cv::Point(mImgSize.width/2+(distance2Points/2),mDestVertices[2].y);
  //Point 1 has the same x coordinates as point 3 and the same y distance as the x distance of Point 2 and 3
  mDestVertices[1] = cv::Point(mDestVertices[3].x,mDestVertices[3].y-distance2Points*mRatio);
  //Point 0 has the same x coordinates as point 2 and the same y distance as the x distance of Point 2 and 3
  mDestVertices[0] = cv::Point(mDestVertices[2].x,mDestVertices[2].y-distance2Points*mRatio);


  std::cout<<"[BirdViewCalibration]destination Points"<<std::endl;
  std::cout<<mDestVertices[0]<<std::endl;
  std::cout<<mDestVertices[1]<<std::endl;
  std::cout<<mDestVertices[2]<<std::endl;
  std::cout<<mDestVertices[3]<<std::endl;

  mWrapMatrix = cv::getPerspectiveTransform(mCalPoints, mDestVertices);
  //also recalculate the position of birdview
  calcPosBirdviewInCarGrid();



}



void CameraCalibration::readCameraCalFile (std::string path)
{
  //cv::FileStorage fs2("../../Testbilder/xtion_intrinsic_calib_High_res.yml", cv::FileStorage::READ);
  cv::FileStorage fs2(path, cv::FileStorage::READ);
  if(fs2.isOpened()){
    std::cout<<"open camera calibration file"<<std::endl;
    cv::FileNode nodeCameraMatrix, nodeDistCoeffs;

    nodeCameraMatrix = fs2["camera_matrix"];
    nodeDistCoeffs = fs2["distortion_coefficients"];
    if(nodeCameraMatrix.empty()|| nodeDistCoeffs.empty()){
      std::cout<<"can't find node camera_matrix or distortion_coefficients please check the file"<<path<<std::endl;
      mCameraCalAvailable = false;
    }
    else{
      fs2["camera_matrix"] >> mCameraMatrix;
      fs2["distortion_coefficients"] >> mDistCoeffs;
      std::cout<<"Load camera matrix und distortion coefficients"<<std::endl;
      mCameraCalAvailable = true;
    }
  }
  else
  {
    std::cout<<"can't open camera calibration File"<<std::endl;

  }
  fs2.release();


}


cv::Mat CameraCalibration::drawCalPoints(cv::Mat img)
{
  cv::Mat dest;
  img.copyTo(dest);
  int thickness = 2;
  int lineType = 8;

  for (int i= 0; i < 4; i++) {
    cv::circle( dest,
                mCalPoints[i],
                5,
                cv::Scalar( 0, 0, 255 ),
                thickness,
                lineType );
    //put the number to the points
    std::string PointNumberString;
    PointNumberString = patch::to_string(i);
    cv::putText(dest, PointNumberString , mCalPoints[i], cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0,200,200), 4);

  }
  return dest;
}


bool CameraCalibration::autoCalPoints(cv::Mat img){
  mImgSize = img.size();

  bool found;
  cv::Mat view;
  std::vector<cv::Point2f> pointBuf;
  cv::Size patternsize(mPointsWidth,mPointsHeight);
  //first distort Image
  img = unDisortImg(img);
  found = cv::findCirclesGrid( img, patternsize, pointBuf, cv::CALIB_CB_SYMMETRIC_GRID| cv::CALIB_CB_CLUSTERING);


  //only go go forward if we have found the pattern
  if(found){
    /*manchmal macht opencv ein paar knoten Die Punkte aus der Kalibrierung sind wie folgt:
     * Point 3              Point 2
     *
     *
     * Point 1				Point 0
     *
     * Sie werden so abgespeichert
     *
     * Point 0				Point 1
     *
     *
     * Point 2				Point 3
     */
    mCalPoints[3] = pointBuf[0];
    mCalPoints[2] = pointBuf[mPointsWidth-1];
    mCalPoints[1] = pointBuf[mPointsWidth*(mPointsHeight-1)];
    mCalPoints[0] = pointBuf[mPointsWidth*mPointsHeight -1];
    calculateWrapMatrix();

  } else {

    found = true;
    mCalPoints[0] = cv::Point2f(553,557);
    mCalPoints[1] = cv::Point2f(688,558);
    mCalPoints[2] = cv::Point2f(507,607);
    mCalPoints[3] = cv::Point2f(738,609);
    calculateWrapMatrix();
  }


  return found;

}

cv::Mat CameraCalibration::unDisortImg(cv::Mat img)
{
  cv::Mat dest;
  if(mCameraCalAvailable)
  {
    cv::undistort(img,dest,mCameraMatrix,mDistCoeffs);
  }
  else
  {
    dest = img;
  }
  return dest;


}

void CameraCalibration::calcPosBirdviewInCarGrid() {
  double distance2PointsPx = mDestVertices[3].x-mDestVertices[2].x;//the vertices are the 2 outer Points so there are N Points between them
  mMPerPixel = (mDistance2PointM*(mPointsWidth-1))/distance2PointsPx;
  mCoordinatesBirdView.y = mDistanceCarToCalPattern-(mImgSize.height-mDestVertices[2].y)*mMPerPixel;
  mCoordinatesBirdView.x = ((-1)*(mDestVertices[2].x+(distance2PointsPx/2)))*mMPerPixel;
  std::cout<<"mPerPixel:"<<mMPerPixel;
  std::cout<<"viewRange is about"<<((mMPerPixel)*mImgSize.height)<<std::endl;
  std::cout<<"20mm are"<<((1/mMPerPixel)*0.02)<<"pixel"<<std::endl;



}

cv::Mat CameraCalibration::transform(cv::Mat image)
{
  cv::Mat dest;
  cv::Mat temp;
  temp = unDisortImg(image);
  //cv::warpPerspective(image,dest,mWrapMatrix,mImgSize,cv::INTER_LINEAR | cv::WARP_FILL_OUTLIERS, cv::BORDER_CONSTANT);
  cv::warpPerspective(image,dest,mWrapMatrix,mImgSize,cv::INTER_LINEAR, cv::BORDER_CONSTANT);
  return dest;


}

void CameraCalibration::adjustPicture(cv::Mat img){
  mAdjustImage = img;
  cv::namedWindow(mWindowNameScale,CV_WINDOW_NORMAL);
  cv::createTrackbar( "scale", mWindowNameScale, &mScaleSlider, 200, CallBackFuncSliderScale,this );
  cv::createTrackbar( "YOffset", mWindowNameScale, &mYOffsetSlider, 1500, CallBackFuncSliderOffsetY,this);
  cv::imshow(mWindowNameScale, mAdjustImage);
  cv::waitKey(0);



}
void CameraCalibration::CallBackFuncSliderOffsetY(int event,void* userdata) {
  if(userdata == NULL)
  {
    std::cerr<<"Nullpointer";
  }
  else
  {
    CameraCalibration* myBirdview = reinterpret_cast<CameraCalibration*>(userdata);
    myBirdview->CallBackFuncSliderOffsetY(event);
  }
}

void CameraCalibration::CallBackFuncSliderScale(int event,void* userdata) {
  if(userdata == NULL)
  {
    std::cerr<<"Nullpointer";
  }
  else
  {
    CameraCalibration* myBirdview = reinterpret_cast<CameraCalibration*>(userdata);
    myBirdview->CallBackFuncSliderScale(event);
  }
}

void CameraCalibration::CallBackFuncSliderOffsetY(int event) {
  setYOffset(mYOffsetSlider);
  cv::Mat view;
  view = transform(mAdjustImage);
  cv::imshow(mWindowNameScale,view);

}

double CameraCalibration::getMPerPixel()
{
  return mMPerPixel;
}

void CameraCalibration::CallBackFuncSliderScale(int event) {
  setScale(mScaleSlider);
  cv::Mat view;
  view = transform(mAdjustImage);
  cv::imshow(mWindowNameScale,view);



}

void CameraCalibration::setYOffset(int YOffset)
{
  mYOffset = YOffset;
  calculateWrapMatrix();

}

void CameraCalibration::setScale(int scale)
{
  mScale = (double)scale/100;
  calculateWrapMatrix();
}
}
}
