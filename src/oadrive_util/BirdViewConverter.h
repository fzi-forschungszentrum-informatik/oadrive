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

#ifndef OADRIVE_UTIL_BIRDVIEWCONVERTER_H
#define OADRIVE_UTIL_BIRDVIEWCONVERTER_H
#include <opencv2/core/core.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
//#include <opencv2/imgcodecs.hpp>

#define DEPTH_IMAGE_HEIGHT 240
#define DEPTH_IMAGE_WIDTH 320
namespace oadrive {
namespace util {
/*!
   \brief The BirdViewConverter class
   This class converts the image into a Birdview image via the warp perspective. (It is speed up via remap)
   You must provide a special camera calibration file which contains a warp matrix (This is not the matrix wich is provided by the normal opencv camera calibration) This file can be generated with
    the test test_oadrive_birdViewCal.cpp or the class CameraCalibration
 */
class BirdViewConverter
{
public:
  BirdViewConverter();
  /*!
     * \brief transform undisort image and transform image into Birdview. Configfile must be read first
     * \param image Image to Transform
     * \return Birdview Image
     */
  cv::Mat transform(cv::Mat image);
  /*!
    * \brief loadConfig Read Calibration File with WarpMatrix
    * \param path Path to Calibration File (Hint: Warp Matrix ist not the camera calibration from OPENCV)
    * \return true if reading was successfull
    */
  bool loadConfig(std::string path);

  //! return distortion coefficients which were read from config file
  cv::Mat getDistortionCoefficients();

  //! return camera matrix which was read from config file
  cv::Mat getCameraMatrix();

private:
  //!Enable undistort during transform
  bool mUndistEnable;
  //!Matrix to transform the Image
  cv::Mat mWarpMatrix;
  //!Matrix to transform depth image
  cv::Mat mWarpDepthMatrix;
  //! Matrix to distort image
  cv::Mat mCameraMatrix, mDistCoeffs;
  //! Precalculated distortion and transformation map
  cv::Mat undistortAndWarpXMapInt, undistortAndWarpYMapInt;

  //! size of the picture
  cv::Size mImgSize;

  //! Path to the complete calibration FIle
  std::string mCompleteCalFile;
};
}
}
#endif // OADRIVE_UTIL_BIRDVIEWCONVERTER_H
