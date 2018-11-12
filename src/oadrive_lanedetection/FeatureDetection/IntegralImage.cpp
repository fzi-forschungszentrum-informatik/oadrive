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

#include "IntegralImage.h"
#include <iostream>

using namespace std;
using namespace cv;

const float IntegralImageRotated::ANGLE = 0.7071067811865475244008443621048490392848359376884740;

void IntegralImage::setIntegralImage(Mat& integralImage) {
  this->integralImage = integralImage;
}

int IntegralImage::boxSum(Point2i position, Point2i delta) {
  return boxSum(position.y, position.x, delta.y, delta.x);
}

int IntegralImage::boxSum(int positionY, int positionX, int deltaY, int deltaX) {
  return ( 	integralImage.at<int>(positionY - deltaY, positionX - deltaX)
                + integralImage.at<int>(positionY + deltaY, positionX + deltaX)
                - integralImage.at<int>(positionY + deltaY, positionX - deltaX)
                - integralImage.at<int>(positionY - deltaY, positionX + deltaX)
                );
}

Point2i IntegralImage::getPointWithOffset(Point2i& vectorMiddleOfFeature, Point2i& vectorToRotationPoint) {
  return vectorMiddleOfFeature + vectorToRotationPoint;
}

Point2i IntegralImage::rotatePoint(Point2i& vectorToBeRotated) {
  return vectorToBeRotated;
}

int IntegralImageRotated::boxSum(int positionY, int positionX, int deltaY, int deltaX) {
  Point2i middlePoint(positionX, positionY);

  Point2i vectorToP1(deltaX, -deltaY);
  Point2i vectorToP2(-deltaX, deltaY);
  Point2i vectorToP3(-deltaX, -deltaY);
  Point2i vectorToP4(deltaX, deltaY);

  Point2i rotatedVectorToP1 = rotatePoint(vectorToP1);
  Point2i rotatedVectorToP2 = rotatePoint(vectorToP2);
  Point2i rotatedVectorToP3 = rotatePoint(vectorToP3);
  Point2i rotatedVectorToP4 = rotatePoint(vectorToP4);

  return ( 	integralImage.at<int>(middlePoint + rotatedVectorToP1)
                + integralImage.at<int>(middlePoint + rotatedVectorToP2)
                - integralImage.at<int>(middlePoint + rotatedVectorToP3)
                - integralImage.at<int>(middlePoint + rotatedVectorToP4)
                );
}

int IntegralImage::boxSum(Point2i (&points)[4]) {
  return(		integralImage.at<int>(points[0])
      - integralImage.at<int>(points[2])
      + integralImage.at<int>(points[1])
      - integralImage.at<int>(points[3])
      );
}

int IntegralImageRotated::boxSum(Point2i (&points)[4]) {
  return(		integralImage.at<int>(points[0])
      - integralImage.at<int>(points[2])
      - integralImage.at<int>(points[3])
      + integralImage.at<int>(points[1])
      );
}

Point2i IntegralImageRotated::rotatePoint(Point2i& vectorToBeRotated) {
  Point2i vectorRotated;
  vectorRotated.x = round( ANGLE * ( (float) vectorToBeRotated.x + (float) vectorToBeRotated.y ));
  vectorRotated.y = round( ANGLE * ( - (float) vectorToBeRotated.x + (float) vectorToBeRotated.y ));

  return vectorRotated;
}

Point2i IntegralImageRotated::getPointWithOffset(Point2i& vectorMiddleOfFeature, Point2i& vectorToRotationPoint) {
  Point2i rotatedVectorToRotationPoint;
  rotatedVectorToRotationPoint = rotatePoint(vectorToRotationPoint);

  return vectorMiddleOfFeature + rotatedVectorToRotationPoint;
}
