// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// This program is free software licensed under the CDDL
// (COMMON DEVELOPMENT AND DISTRIBUTION LICENSE Version 1.0).
// You can find a copy of this license in LICENSE in the top
// directory of the source code.
//
// © Copyright 2016 FZI Forschungszentrum Informatik, Karlsruhe, Germany
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Peter Zimmer <peter-zimmer@gmx.net>
 * \date    2015-10-21
 *
 */
//----------------------------------------------------------------------

#ifndef OADRIVE_UTIL_COORDINATECONVERTER_H
#define OADRIVE_UTIL_COORDINATECONVERTER_H

/*
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

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
//#include <opencv2/imgcodecs.hpp>
#include "string.h"
#include <iostream>
#include "time.h"

#include <oadrive_core/ExtendedPose2d.h>

// Distance from front bumper to the center of the rear axis
#define CAR_ORIGIN_TO_FRONT 0.48		// in m
#define DEPTH_IMAGE_HEIGHT 240
#define DEPTH_IMAGE_WIDTH 320
namespace oadrive {
namespace util {

class CoordinateConverter
{
public:
  /* \brief Constructor
         * \param path The path/filename of the configuration file which holds the bird view configuration.
         */
  CoordinateConverter(std::string path);

  /* \brief Constructor which will use default values (only for testing purposes!) instead of config file. */
  CoordinateConverter();
  ~CoordinateConverter();

  /*! \brief get from image coordinates to car coordinates
   * \param input Pixel position from upper left corner of image
   * \return Position of pixel in car coordinates.
   */
  oadrive::core::ExtendedPose2d pixel2Car( cv::Point2f input );

  /*! \brief convert coordinates in car-space to pixel positions in bird view image coordinates
   * \param pose 2D-Position in car coordinates
   * \return Pixel position in the coordinate system of the bird-view image (0,0 is upper left corner).
   */
  cv::Point2f car2Pixel( const oadrive::core::ExtendedPose2d &pose );

  /*! \brief convert coordinates from car-space to (absolute) world space.
   * \param car the current position and orientation of the car in world space.
   * \param point the coordinates to be transformed
   * \note TODO: Check if angle is converted correctly!
   * \return The point in world coordinates
   */
  oadrive::core::ExtendedPose2d car2World( const oadrive::core::ExtendedPose2d &car, const oadrive::core::ExtendedPose2d &point);

  oadrive::core::ExtendedPose2d world2Car( const oadrive::core::ExtendedPose2d &car, const oadrive::core::ExtendedPose2d &point);


  /*! \brief convert pixel position of point in bird view image to position in world space.
   * Calls pixel2Car and car2World internally.
   * \param car the current position and orientation of the car in world space.
   * \param pixelPos the coordinates in the image to be transformed
   * \return position in world space.
   */
  oadrive::core::ExtendedPose2d pixel2World( const oadrive::core::ExtendedPose2d &car, cv::Point2f pixelPos );

  /*! Calculates pixel coordinates in bird view image from a 2d world position.*/
  cv::Point2f world2Pixel( const oadrive::core::ExtendedPose2d &car, const oadrive::core::ExtendedPose2d &point );

  //! reads the config file
  void readConfigFile(std::string path);

  //! returns the number of meters per pixel used.
  float getMetersPerPixel() { return mMetersPerPixel; }

  //! returns the number of pixels per meter used.
  float getPixelsPerMeter() { return 1.0/mMetersPerPixel; }

  //! returns the ImageSize of the BirdView Image
  cv::Size getImgSizeBirdView(){return mImgSize;}

  oadrive::core::ExtendedPose2d depthPixel2Car( cv::Point2f pixel, float depth );
  /*! Calculate the distance to an infinite, ideal ground plane for the pixel.
         * param planeZ Offset from true ground:*/
  float calcZeroPlaneDepth( cv::Point2f pixel, float planeZ );

  //! return the four points of the birdview cone (first two points are for the left cone, the last two points are for the right cone)
  void getConePoints(std::vector<cv::Point2f>& cornerPointsBirdview);
  //! calculate first and last row
  void setFirstAndLastRow();
  //!get first row from the original image which is maped into the birdview.
  int getFirstRow(){return mFirstRow;}
  //!get last row from the original image which is maped into the birdview.
  int getLastRow(){return mLastRow;}
private:
  //!coordinates of the bottom left corner of the birdview relative to the center of the front bumper (from config file)
  oadrive::core::ExtendedPose2d mCoordinatesBirdView;
  //!m per pixel in the Birdview (from config file)
  float mMetersPerPixel;
  //! size of the picture (from config file)
  cv::Size mImgSize;
  //!warp Matrix to transform to the birdview
  cv::Mat mWarpMatrix;
  //! this matrix is calculate from the WarpMatrix
  cv::Mat mWarpDepthMatrix;
  cv::Mat mCameraMatrix;
  //! focal lengths:
  float mfX;
  float mfY;
  //! center x and y (half of image size, could use mImgSize):
  float mcX;
  float mcY;

  //! x, y, z position of camera, relative to car origin (in car coordinates).
  float mCameraPosX;
  float mCameraPosY;
  float mCameraPosZ;
  float mCameraPitch;

  //!first row from the original image which is maped into the birdview.
  int mFirstRow;
  //last row from the original image which is maped into the birdview.
  int mLastRow;
public:
  // use a proper alignment when calling the constructor.
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}	// namespace
}	// namespace

#endif  // OADRIVE_UTIL_COORDINATECONVERTER_H
