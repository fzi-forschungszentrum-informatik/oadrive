// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// This program is free software licensed under the CDDL
// (COMMON DEVELOPMENT AND DISTRIBUTION LICENSE Version 1.0).
// You can find a copy of this license in LICENSE in the top
// directory of the source code.
//
// © Copyright 2017 FZI Forschungszentrum Informatik, Karlsruhe, Germany
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

#include "BirdViewConverter.h"
#include <iostream>
#include "utilLogging.h"
#include <opencv2/flann/timer.h>
using icl_core::logging::endl;
using icl_core::logging::flush;

namespace oadrive {
namespace util {
BirdViewConverter::BirdViewConverter():
  mUndistEnable(false)
{
}


cv::Mat BirdViewConverter::transform(cv::Mat image)
{
  cv::Mat dest;

  // undistortion coefficient not provided for simulation
  if(mUndistEnable)
  {
    cv::remap(image, dest, undistortAndWarpXMapInt, undistortAndWarpYMapInt, cv::INTER_LINEAR);
  }
  else
  {
    cv::warpPerspective(image, dest, mWarpMatrix, mImgSize, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
  }

  return dest;
}

bool BirdViewConverter::loadConfig(std::string path)
{
  bool success = false;
  cv::FileStorage fs2(path, cv::FileStorage::READ);
  LOGGING_INFO(utilLogger,"try to load File"<<path<<endl);
  if(fs2.isOpened()){
    LOGGING_TRACE(utilLogger,"open calibration file"<<endl);
    cv::FileNode nodeCameraMatrix, nodeDistCoeffs, nodewarpMatrix, nodeSize;

    nodewarpMatrix = fs2["warpMatrix"];
    nodeSize = fs2["ImgSize"];
    nodeCameraMatrix = fs2["camera_matrix"];
    nodeDistCoeffs = fs2["distortion_coefficients"];
    if(nodewarpMatrix.empty()||nodeSize.empty()){
      LOGGING_ERROR(utilLogger,"can't find warpMatrix or Image Size in Calibration File please check the file" <<endl);
      throw("can't find warpMatrix or Image Size in Calibration File please check the file");
    }
    else{
      LOGGING_INFO(utilLogger,"Read  Warp Matrix and Image Size:" <<endl);
      fs2["warpMatrix"]>>mWarpMatrix;
      fs2["ImgSize"]>>mImgSize;
      if(nodeCameraMatrix.empty()||nodeDistCoeffs.empty())
      {
        LOGGING_WARNING(utilLogger,"Can't find camera matrix and or distortion coefficients undistortion is turned off"<<endl);
      }
      else
      {
        LOGGING_INFO(utilLogger,"Read Camera Matrix and distortion coefficients" <<endl);
        fs2["distortion_coefficients"]>>mDistCoeffs;
        fs2["camera_matrix"]>>mCameraMatrix;
        mUndistEnable = true;
      }
    }

    success = true;

    // precalculate matrices for birdview transform if camera matrix and distortion coefficients are given
    if(mUndistEnable) {
      // Idea taken from http://stackoverflow.com/questions/29944709/how-to-combine-two-remap-operations-into-one
      cv::Mat optimalCameraMatrixInverse = cv::getOptimalNewCameraMatrix(mCameraMatrix, mDistCoeffs, mImgSize, 0).inv();
      cv::Mat warpMatrixInverse = mWarpMatrix.inv();
      cv::Mat_<float> productOptimalCameraMatrixInverseWithWarpMatrixInverse = optimalCameraMatrixInverse * warpMatrixInverse;


      cv::Mat undistortAndWarpXMapFloat(mImgSize, CV_32F);
      cv::Mat undistortAndWarpYMapFloat(mImgSize, CV_32F);

      for(int y=0; y<mImgSize.height; ++y) {
        std::vector<cv::Point3f> pointsUndistortedNormalized(mImgSize.width);
        // For each pixel on the current row, first use the inverse perspective mapping, then multiply by the
        // inverse camera matrix (i.e. map from pixels to normalized coordinates to prepare use of projectPoints function)
        for(int x=0; x<mImgSize.width; ++x) {
          cv::Mat_<float> pt(3,1); pt << x,y,1;
          pt = productOptimalCameraMatrixInverseWithWarpMatrixInverse * pt;
          pointsUndistortedNormalized[x].x = pt(0) / pt(2);
          pointsUndistortedNormalized[x].y = pt(1) / pt(2);
          pointsUndistortedNormalized[x].z = 1;
        }
        // For each pixel on the current row, compose with the inverse undistortion mapping (i.e. the distortion
        // mapping) using projectPoints function
        std::vector<cv::Point2f> pointsDistorted;
        cv::projectPoints(pointsUndistortedNormalized, cv::Mat::zeros(3,1,CV_32F), cv::Mat::zeros(3,1,CV_32F), mCameraMatrix, mDistCoeffs, pointsDistorted);
        // Store the result in the appropriate pixel of the output maps
        for(int x=0; x<mImgSize.width; ++x) {
          undistortAndWarpXMapFloat.at<float>(y,x) = pointsDistorted[x].x;
          undistortAndWarpYMapFloat.at<float>(y,x) = pointsDistorted[x].y;
        }
      }
      // Finally, convert the float maps to signed-integer maps for best efficiency of the remap function
      cv::convertMaps(undistortAndWarpXMapFloat, undistortAndWarpYMapFloat, undistortAndWarpXMapInt, undistortAndWarpYMapInt, CV_16SC2);
    }
  }
  else
  {
    LOGGING_ERROR(utilLogger,"can't open Calibration File"<<endl);
    success = false;
  }
  fs2.release();
  return success;

}


cv::Mat BirdViewConverter::getDistortionCoefficients() {
  return mDistCoeffs;
}

cv::Mat BirdViewConverter::getCameraMatrix() {

  return mCameraMatrix;
}

}
}
