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
 * \date    2015-10-21
 *
 */
//----------------------------------------------------------------------

#include "CoordinateConverter.h"

#include <opencv2/core/core.hpp>
#include <oadrive_core/Trajectory2d.h>
#include <oadrive_core/Interpolator.h>
#include <oadrive_core/Pose.h>
#include <oadrive_core/ExtendedPose2d.h>
#include <oadrive_util/Config.h>
#include "utilLogging.h"

using icl_core::logging::endl;
using icl_core::logging::flush;

using namespace oadrive::core;
using namespace oadrive::lanedetection;

namespace oadrive
{
namespace util
{

CoordinateConverter::CoordinateConverter(std::string path)
{
  //readConfigFile(path);
  mMetersPerPixelX = Config::getDouble("BirdviewCal", "MPerPixelX", 1.0 / 400.0);
  mMetersPerPixelY = Config::getDouble("BirdviewCal", "MPerPixelY", 1.0 / 400.0);
  cv::Mat tmp = Config::getMat("BirdviewCal", "ImgSize");
  if (!tmp.empty())
  {
    mImgSize = cv::Size(tmp.at<double>(0), tmp.at<double>(1));
  }
  else
  {
    mImgSize = cv::Size(640, 480);
  }

  tmp = Config::getMat("BirdviewCal", "CoordinatesBirdView");
  if (!tmp.empty())
  {
    double x = tmp.at<double>(0) + mImgSize.height * mMetersPerPixelY;
    double y = tmp.at<double>(1);
    mCoordinatesBirdView = ExtendedPose2d(x, y, 0.f);
    mCoordinatesBirdView2 = OadrivePose(x, y, 0.f);
  }
  else
  {
    mCoordinatesBirdView = ExtendedPose2d(0.f, 0.f, 0.f);
    mCoordinatesBirdView2 = OadrivePose(0.f, 0.f, 0.f);
  }


  mWarpMatrix = Config::getMat("BirdviewCal", "warpMatrix");
  mCameraMatrix = Config::getMat("BirdviewCal", "camera_matrix");
  if (!mCameraMatrix.empty())
  {
    mfX = mCameraMatrix.at<double>(0, 0);
    mfY = mCameraMatrix.at<double>(1, 1);
    mcX = mCameraMatrix.at<double>(0, 2);
    mcY = mCameraMatrix.at<double>(1, 2);
  }
  if (!mWarpMatrix.empty())
  {
    mWarpDepthMatrix = mWarpMatrix.clone();
  }
  //calculate warpmatrix for depth Image wich have a other resoulution

  double scaleFactor = mImgSize.width / DEPTH_IMAGE_WIDTH;
  if (!mWarpDepthMatrix.empty())
  {
    mWarpDepthMatrix.at<double>(0, 0) = mWarpDepthMatrix.at<double>(0, 0) * scaleFactor;
    mWarpDepthMatrix.at<double>(1, 0) = mWarpDepthMatrix.at<double>(1, 0) * scaleFactor;
    mWarpDepthMatrix.at<double>(2, 0) = mWarpDepthMatrix.at<double>(2, 0) * scaleFactor;
    mWarpDepthMatrix.at<double>(0, 1) = mWarpDepthMatrix.at<double>(0, 1) * scaleFactor;
    mWarpDepthMatrix.at<double>(1, 1) = mWarpDepthMatrix.at<double>(1, 1) * scaleFactor;
    mWarpDepthMatrix.at<double>(2, 1) = mWarpDepthMatrix.at<double>(2, 1) * scaleFactor;
  }

  mCameraPosX = Config::getDouble("BirdviewCal", "mCameraPosX", -0.055);
  mCameraPosY = Config::getDouble("BirdviewCal", "mCameraPosY", 0.295);
  mCameraPosZ = Config::getDouble("BirdviewCal", "mCameraPosZ", 0.225);
  mCameraPitch = Config::getDouble("BirdviewCal", "mCameraPitch", -0.05);

  setFirstAndLastRow();
}

CoordinateConverter::CoordinateConverter()
{
  // Fill with default values. Taken from Goffin's (Car 18) Bird View calibration file.
  mCoordinatesBirdView = ExtendedPose2d(-1.0980392694473267e+00, 2.4168626964092255e-01, 0);
  //mMetersPerPixel = 3.4313725490196082e-03;

  mCameraPosX = -0.055;
  mCameraPosY = 0.295;
  mCameraPosZ = 0.225;
  mCameraPitch = -0.05235987755982988731;

  // Adjusted to Jan's 'ideal images':
  //mMetersPerPixel = 1.0/334.0;
  //mMetersPerPixel = 1.0 / 284.0;
  mImgSize = cv::Size(640, 480);
//
  LOGGING_WARNING(utilLogger,
                  "WARNING! CoordinateConverter has been constructed without config file path."
                          << endl
                          << "\tUsing default configuration. May lead to wrong position conversions!"
                          << endl);
}

CoordinateConverter::~CoordinateConverter()
{
}

bool CoordinateConverter::insideBirdviewImage(const ExtendedPose2d &carPose, const ExtendedPose2d
                                                                &pose)
{
  cv::Point2f pixelPos = world2Pixel(carPose, pose);

  if (pixelPos.x >= 0 && pixelPos.x < mImgSize.width
          && pixelPos.y >= 0 && pixelPos.y <mImgSize.height)
  {
    return true;
  }
  return false;
}


bool CoordinateConverter::insideBirdviewImage(const lanedetection::OadrivePose &carPose,
                                              const lanedetection::OadrivePose &pose)
{
  cv::Point2f pixelPos = world2Pixel(carPose, pose);

  if (pixelPos.x >= 0 && pixelPos.x < mImgSize.width
      && pixelPos.y >= 0 && pixelPos.y <mImgSize.height)
  {
    return true;
  }
  return false;
}

bool CoordinateConverter::isPixel2CarCorrect(cv::Point2f pixelPos) {
    if (pixelPos.y > mImgSize.height) {
      return false;
    } 
    return true;
}

ExtendedPose2d CoordinateConverter::pixel2Car(cv::Point2f pixelPos)
{
  if (pixelPos.y > mImgSize.height)
  {
    LOGGING_WARNING(utilLogger, "!!!!!!!!!!!!!!!!!!!!![CoordinateConverter] Warning: "
            "pixelPos Y is higher than Pixel Height the Calculation is false" << endl);
  }

  double xCar = -pixelPos.y * mMetersPerPixelY + mCoordinatesBirdView.getX();
  double yCar = -pixelPos.x * mMetersPerPixelX + mCoordinatesBirdView.getY();

  return ExtendedPose2d(xCar, yCar, 0.f);
}

lanedetection::OadrivePose CoordinateConverter::pixel2Car2(cv::Point2f pixelPos)
{
  if (pixelPos.y > mImgSize.height)
  {
    LOGGING_WARNING(utilLogger, "!!!!!!!!!!!!!!!!!!!!![CoordinateConverter] Warning: "
            "pixelPos Y is higher than Pixel Height the Calculation is false" << endl);
  }

  double xCar = -pixelPos.y * mMetersPerPixelY + mCoordinatesBirdView.getX();
  double yCar = -pixelPos.x * mMetersPerPixelX + mCoordinatesBirdView.getY();

  return OadrivePose(xCar, yCar, 0.f);
}

cv::Point2f CoordinateConverter::car2Pixel(const ExtendedPose2d &pose)
{
  double xImage = -(pose.getY() - mCoordinatesBirdView.getY()) / mMetersPerPixelX;
  double yImage = -(pose.getX() - mCoordinatesBirdView.getX()) / mMetersPerPixelY;

  return cv::Point2f(xImage, yImage);
}

cv::Point2f CoordinateConverter::car2Pixel(const lanedetection::OadrivePose &pose)
{
  double xImage = -(pose.getY() - mCoordinatesBirdView.getY()) / mMetersPerPixelX;
  double yImage = -(pose.getX() - mCoordinatesBirdView.getX()) / mMetersPerPixelY;

  return cv::Point2f(xImage, yImage);
}

ExtendedPose2d
CoordinateConverter::car2World(const ExtendedPose2d &car, const ExtendedPose2d &point)
{
  double ang = car.getYaw();
  double x = point.getX();
  double y = point.getY();

  // Rotate point by angle of car pose:
  ExtendedPose2d rotated(x * cos(ang) - y * sin(ang), x * sin(ang) + y * cos(ang), 0);

  // Add car position:
  ExtendedPose2d pointWorld(rotated.getX() + car.getX(),
                            rotated.getY() + car.getY(),
                            fmod(point.getYaw() + car.getYaw(), 2 * M_PI));

  return pointWorld;
}

lanedetection::OadrivePose CoordinateConverter::car2World(const lanedetection::OadrivePose &car,
                                                          const lanedetection::OadrivePose &point)
{
  double ang = car.getYaw();
  double x = point.getX();
  double y = point.getY();

  // Rotate point by angle of car pose:
  OadrivePose rotated(x * cos(ang) - y * sin(ang), x * sin(ang) + y * cos(ang), 0);

  // Add car position:
  OadrivePose pointWorld(rotated.getX() + car.getX(),
                            rotated.getY() + car.getY(),
                            fmod(point.getYaw() + car.getYaw(), 2.f * M_PI));

  return pointWorld;
}

ExtendedPose2d
CoordinateConverter::world2Car(const ExtendedPose2d &car, const ExtendedPose2d &point)
{
  double ang = car.getYaw();

  // Subtract car position:
  double x = point.getX() - car.getX();
  double y = point.getY() - car.getY();

  // Rotate point by angle of car pose:
  ExtendedPose2d rotated(x * cos(ang) + y * sin(ang),
                         x * -sin(ang) + y * cos(ang),
                         mod(point.getYaw() - car.getYaw(), 2 * M_PI));

  return rotated;
}

lanedetection::OadrivePose CoordinateConverter::world2Car(const lanedetection::OadrivePose &car,
                                                          const lanedetection::OadrivePose &point)
{
  double ang = car.getYaw();

  // Subtract car position:
  double x = point.getX() - car.getX();
  double y = point.getY() - car.getY();

  // Rotate point by angle of car pose:
  OadrivePose rotated(x * cos(ang) + y * sin(ang),
                         x * -sin(ang) + y * cos(ang),
                         mod(point.getYaw() - car.getYaw(), float(2.f * M_PI)));

  return rotated;
}

ExtendedPose2d CoordinateConverter::pixel2World(const ExtendedPose2d &car, cv::Point2f pixelPos)
{
  // First, transform to car coordinates:
  ExtendedPose2d poseInCarCoords = pixel2Car(pixelPos);
  // ... and then convert those to global, world coordinates:
  return car2World(car, poseInCarCoords);
}


lanedetection::OadrivePose
CoordinateConverter::pixel2World(const lanedetection::OadrivePose &car, cv::Point2f pixelPos)
{
  // First, transform to car coordinates:
  OadrivePose poseInCarCoords = pixel2Car2(pixelPos);
  // ... and then convert those to global, world coordinates:
  return car2World(car, poseInCarCoords);
}

cv::Point2f CoordinateConverter::world2Pixel(const ExtendedPose2d &car, const ExtendedPose2d &point)
{
  ExtendedPose2d poseInCarCoords = world2Car(car, point);
  return car2Pixel(poseInCarCoords);
}

cv::Point2f CoordinateConverter::world2Pixel(const lanedetection::OadrivePose &car,
                                             const lanedetection::OadrivePose &point)
{
  OadrivePose poseInCarCoords = world2Car(car, point);
  return car2Pixel(poseInCarCoords);
}

cv::Point2f CoordinateConverter::car2VotingSpace(const core::ExtendedPose2d refPointInLocal,
                                                 const cv::Point2f refPointInVotingSpace,
                                                 const core::ExtendedPose2d pose, float scale)
{
  // taking into account that the image center is the top left corner. Swap x and y and negate them.
  double votingSpaceY = -(pose.getX() - refPointInLocal.getX()) / (mMetersPerPixelY / scale);
  double votingSpaceX = -(pose.getY() - refPointInLocal.getY()) / (mMetersPerPixelX / scale);

  cv::Point2f result(votingSpaceX + refPointInVotingSpace.x,
                     votingSpaceY + refPointInVotingSpace.y);
  return result;
}

cv::Point2f CoordinateConverter::car2VotingSpace(const lanedetection::OadrivePose refPointInLocal,
                                                 const cv::Point2f refPointInVotingSpace,
                                                 const lanedetection::OadrivePose pose, float scale)
{
  // taking into account that the image center is the top left corner. Swap x and y and negate them.
  double votingSpaceY = -(pose.getX() - refPointInLocal.getX()) / (mMetersPerPixelY / scale);
  double votingSpaceX = -(pose.getY() - refPointInLocal.getY()) / (mMetersPerPixelX / scale);

  cv::Point2f result(votingSpaceX + refPointInVotingSpace.x,
                     votingSpaceY + refPointInVotingSpace.y);
  return result;
}

ExtendedPose2d CoordinateConverter::votingSpace2Car(const core::ExtendedPose2d refPointInLocal,
                                                    const cv::Point2f refPointInVotingSpace,
                                                    cv::Point2f pose, float scale)
{
  double yCar = -(pose.x - refPointInVotingSpace.x) * (mMetersPerPixelX / scale);
  double xCar = -(pose.y - refPointInVotingSpace.y) * (mMetersPerPixelY / scale);

  core::ExtendedPose2d result(xCar + refPointInLocal.getX(),
                              yCar + refPointInLocal.getY(), 0.f);
  return result;
}

lanedetection::OadrivePose
CoordinateConverter::votingSpace2Car(const lanedetection::OadrivePose refPointInLocal,
                                     const cv::Point2f refPointInVotingSpace, cv::Point2f pose,
                                     float scale)
{
  double yCar = -(pose.x - refPointInVotingSpace.x) * (mMetersPerPixelX / scale);
  double xCar = -(pose.y - refPointInVotingSpace.y) * (mMetersPerPixelY / scale);

  OadrivePose result(xCar + refPointInLocal.getX(),
                              yCar + refPointInLocal.getY(), 0.f);
  return result;
}


float CoordinateConverter::getMetersPerPixelX(float scale)
{
  return (mMetersPerPixelX / scale);
}

float CoordinateConverter::getMetersPerPixelY(float scale)
{
  return (mMetersPerPixelY / scale);
}

void CoordinateConverter::readConfigFile(std::string path)
{
  cv::FileStorage fs2(path, cv::FileStorage::READ);
  if (fs2.isOpened())
  {
    LOGGING_INFO(utilLogger, "[CoordinateConverter] Opened camera calibration file"
            << endl << "\t" << path << endl);
    cv::FileNode nodeCoordinatesBirdView;

    nodeCoordinatesBirdView = fs2["CoordinatesBirdView"];
    if (nodeCoordinatesBirdView.empty())
    {
      LOGGING_WARNING(utilLogger,
                      "[CoordinateConverter] can't find node CoodrinatesBirdView Matrix. "
                              "Please check the file." << endl);
    }
    else
    {
      cv::Point2f tmp;
      fs2["CoordinatesBirdView"] >> tmp;
      mCoordinatesBirdView = ExtendedPose2d(tmp.x, tmp.y, 0);
      fs2["MPerPixelX"] >> mMetersPerPixelX;
      fs2["MPerPixelY"] >> mMetersPerPixelY;
      fs2["ImgSize"] >> mImgSize;
      fs2["warpMatrix"] >> mWarpMatrix;
      fs2["camera_matrix"] >> mCameraMatrix;
      mfX = mCameraMatrix.at<double>(0, 0);
      mfY = mCameraMatrix.at<double>(1, 1);
      mcX = mCameraMatrix.at<double>(0, 2);
      mcY = mCameraMatrix.at<double>(1, 2);
      mWarpDepthMatrix = mWarpMatrix.clone();
      //calculate warpmatrix for depth Image wich have a other resoulution

      double scaleFactor = mImgSize.width / DEPTH_IMAGE_WIDTH;
      mWarpDepthMatrix.at<double>(0, 0) = mWarpDepthMatrix.at<double>(0, 0) * scaleFactor;
      mWarpDepthMatrix.at<double>(1, 0) = mWarpDepthMatrix.at<double>(1, 0) * scaleFactor;
      mWarpDepthMatrix.at<double>(2, 0) = mWarpDepthMatrix.at<double>(2, 0) * scaleFactor;
      mWarpDepthMatrix.at<double>(0, 1) = mWarpDepthMatrix.at<double>(0, 1) * scaleFactor;
      mWarpDepthMatrix.at<double>(1, 1) = mWarpDepthMatrix.at<double>(1, 1) * scaleFactor;
      mWarpDepthMatrix.at<double>(2, 1) = mWarpDepthMatrix.at<double>(2, 1) * scaleFactor;

      fs2["mCameraPosX"] >> mCameraPosX;
      fs2["mCameraPosY"] >> mCameraPosY;
      fs2["mCameraPosZ"] >> mCameraPosZ;
      fs2["mCameraPitch"] >> mCameraPitch;

      setFirstAndLastRow();

    }
  }
  else
  {
    LOGGING_INFO(utilLogger,
                 "[CoordinateConverter] can't open complete Calibration File:" << endl << "\t"
                                                                               << path << endl);
  }
  fs2.release();
}

void CoordinateConverter::getConePoints(std::vector<cv::Point2f> &cornerPointsBirdview)
{
  // get inverse warp transform
  if (mWarpMatrix.empty())
  {
    LOGGING_INFO(utilLogger,
                 "[CoordinateConverter] warpMatrix not set, cone calculation impossible!" << endl);

    cornerPointsBirdview.clear();
  }
  else
  {
    cv::Mat mWarpMatrixInverse = mWarpMatrix.clone();
    cv::invert(mWarpMatrix, mWarpMatrixInverse);

    // get the y values in original image of y 20th and 50th row from bottom in bird view image
    std::vector<cv::Point2f> pointsBirdview;
    std::vector<cv::Point2f> pointsOriginalImage;
    pointsBirdview.push_back(cv::Point2f(mImgSize.width / 2, mImgSize.height - 20));
    pointsBirdview.push_back(cv::Point2f(mImgSize.width / 2, mImgSize.height - 50));
    pointsOriginalImage.resize(pointsBirdview.size());
    cv::perspectiveTransform(pointsBirdview, pointsOriginalImage, mWarpMatrixInverse);

    // get coordinates in bird view image from boundary points in original image
    std::vector<cv::Point2f> cornerPointsOriginalImage;
    cornerPointsOriginalImage.push_back(cv::Point2f(0, pointsOriginalImage[0].y));
    cornerPointsOriginalImage.push_back(cv::Point2f(0, pointsOriginalImage[1].y));
    cornerPointsOriginalImage.push_back(cv::Point2f(mImgSize.width - 1, pointsOriginalImage[1].y));
    cornerPointsOriginalImage.push_back(cv::Point2f(mImgSize.width - 1, pointsOriginalImage[0].y));
    cornerPointsBirdview.resize(cornerPointsOriginalImage.size());
    cv::perspectiveTransform(cornerPointsOriginalImage, cornerPointsBirdview, mWarpMatrix);
  }
}

void CoordinateConverter::setFirstAndLastRow()
{
  if (mWarpMatrix.empty())
  {
    LOGGING_INFO(utilLogger,
                 "[CoordinateConverter] warpMatrix not set, cone calculation impossible!" << endl);
  }
  else
  {
    cv::Mat mWarpMatrixInverse = mWarpMatrix.clone();
    cv::invert(mWarpMatrix, mWarpMatrixInverse);

    // get the y values in original image of y 20th and 50th row from bottom in bird view image
    std::vector<cv::Point2f> pointsBirdview;
    std::vector<cv::Point2f> pointsOriginalImage;
    pointsBirdview.push_back(cv::Point2f(mImgSize.width / 2, 0));
    pointsBirdview.push_back(cv::Point2f(mImgSize.width / 2, mImgSize.height));
    pointsOriginalImage.resize(pointsBirdview.size());
    cv::perspectiveTransform(pointsBirdview, pointsOriginalImage, mWarpMatrixInverse);
    mFirstRow = pointsOriginalImage[0].y;
    mLastRow = pointsOriginalImage[1].y;
  }
}

ExtendedPose2d CoordinateConverter::depthPixel2Car(cv::Point2f pixel, float depth)
{
  // Note: We found too little documentation on the depth image of the
  // xtion camera used. The way the ground plane is calculated here seems to be
  // precise enough for our purposes, but if you want to do things which depend on
  // a higher precision, please do not take for granted that this method will work!

  // See "focal length", wikipedia. And Strahlensatz.
  // Factor of 2 because other values here are from the RGB image, which is twice as large.
  float u = (2 * pixel.x - mcX);
  float v = -(2 * pixel.y - mcY);

  float X = u * depth / mfX;
  float Y = v * depth / mfY;
  float Z = depth;

  // Rotate because camera is pitched down:
  float YCar = Z * cos(mCameraPitch) - Y * sin(mCameraPitch);
  //float ZCar = Z*sin( mCameraPitch ) + Y*cos( mCameraPitch );
  float XCar = X;

  return ExtendedPose2d(XCar + mCameraPosX, YCar + mCameraPosY, 0);
}

float CoordinateConverter::calcZeroPlaneDepth(cv::Point2f pixel, float planeZcoord)
{
  // Calculates the depth of a pixel, when looking at an ideal ("imaginary") ground plane.
  // Note: We found too little documentation on the depth image of the
  // xtion camera used. The way the ground plane is calculated here seems to be
  // precise enough for our purposes, but if you want to do things which depend on
  // a higher precision, please do not take for granted that this method will work!

  // Pixel, relative to position:
  float v = (2 * pixel.y - mcY);

  float ang2Ground = M_PI_2 - atan(v / mfY) + mCameraPitch;

  float dist = (mCameraPosZ - planeZcoord) / cos(ang2Ground);

  return dist;
}

Polygon CoordinateConverter::scaleRectangle(cv::RotatedRect rectToScale, float scale)
{
  PointList polygonVertices(4);


  cv::Point2f vertices[4];
  rectToScale.points(vertices);

  for (int i = 0; i < 4; i++)
  {
    polygonVertices.at(i) = vertices[i] - rectToScale.center;

    polygonVertices.at(i).x /= getMetersPerPixelX(scale);
    polygonVertices.at(i).y /= getMetersPerPixelY(scale);

    polygonVertices.at(i) += rectToScale.center;
  }

  return Polygon(polygonVertices, rectToScale.center);
}

float CoordinateConverter::getBirdviewOffsetX()
{
  return (mCoordinatesBirdView2.getX() - mImgSize.height * mMetersPerPixelY);
}

}  // namespace
}  // namespace

