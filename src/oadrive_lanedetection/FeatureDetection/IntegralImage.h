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
 * \author  Jan-Markus Gomer <uecxl@student.kit.edu>
 * \date    2015-12-06
 *
 */
//----------------------------------------------------------------------

#ifndef INTEGRALIMAGE_H_
#define INTEGRALIMAGE_H_

#include <opencv2/imgproc/imgproc.hpp>

class IntegralImage {
public:
  void setIntegralImage(cv::Mat& integralImage);
  int boxSum(cv::Point2i position, cv::Point2i delta);
  virtual int boxSum(cv::Point2i (&points)[4]);
  virtual int boxSum(int positionY, int positionX, int deltaY, int deltaX);
  virtual cv::Point2i getPointWithOffset(cv::Point2i& vectorMiddleOfFeature, cv::Point2i& vectorToRotationPoint);
  virtual cv::Point2i rotatePoint(cv::Point2i& vectorToBeRotated);

protected:
  cv::Mat integralImage;
};

class IntegralImageRotated : public IntegralImage {
public:
  int boxSum(cv::Point2i (&points)[4]);
  int boxSum(int positionY, int positionX, int deltaY, int deltaX);
  cv::Point2i getPointWithOffset(cv::Point2i& vectorMiddleOfFeature, cv::Point2i& vectorToRotationPoint);
  cv::Point2i rotatePoint(cv::Point2i& vectorToBeRotated);

private:
  static const float ANGLE; // cos(Pi/4)=sin(Pi/4)

};

#endif /* INTEGRALIMAGE_H_ */
