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
 * \author  Jan-Markus Gomer <uecxl@student.kit.edu>
 * \date    2015-11-01
 *
 */
//----------------------------------------------------------------------

#include "HaarFeature.h"

using namespace std;
using namespace cv;

void HaarFeature::setIntegralImage(IntegralImage* integralImage) {
  this->integralImage = integralImage;
}

void HaarFeature::updateMaximumResponse() {
  maximumResponse = 255 * ( 2 * ( deltaX / 2 ) ) * ( 2 * deltaY );
}

int HaarFeature::getMaximumResponse() {
  return maximumResponse;
}

void HaarFeature::setDeltaX(int deltaX) {
  this->deltaX = deltaX;
}

void HaarFeature::setDeltaY(int deltaY) {
  this->deltaY = deltaY;
}

void HaarFeature::setDeltaXY(int deltaX, int deltaY) {
  setDeltaX(deltaX);
  setDeltaY(deltaY);
  updateMaximumResponse();
}

void HaarFeature::setAngle(float angle) {
  this->angle = angle;
  this->sinAngle = sin(angle);
  this->cosAngle = cos(angle);
}

int HaarFeature::getDeltaX() {
  return deltaX;
}

int HaarFeature::getDeltaY() {
  return deltaY;
}

Point2i HaarFeature::rotatePoint(Point2i vectorToBeRotated, bool roundRotatedPoint) {
  Point2f temp;
  Point2i vectorRotated;

  temp.x = ( (float) vectorToBeRotated.x * cosAngle + (float) vectorToBeRotated.y * sinAngle );
  temp.y = (- (float) vectorToBeRotated.x * sinAngle + (float) vectorToBeRotated.y * cosAngle );

  if(roundRotatedPoint) {
    vectorRotated.x = round( temp.x );
    vectorRotated.y = round( temp.y );
  } else {
    vectorRotated.x = temp.x;
    vectorRotated.y = temp.y;
  }

  return vectorRotated;
}

Point2i HaarFeature::alignPointsOnLine(Point2i smallRectangle, Point2i bigRectangle) {
  int addValue;
  cv::Point2i differenceVector, absoluteDifferenceVector;
  cv::Point2i alignedPoint(bigRectangle);
  differenceVector = bigRectangle - smallRectangle;

  switch(abs(differenceVector.x) - abs(differenceVector.y)) {
    case -2:
      addValue = (int) copysign(1.0, differenceVector.x);
      alignedPoint.x += addValue;
      alignedPoint.y += addValue;
      break;
    case -1:
      addValue = (int) copysign(1.0, differenceVector.x);
      alignedPoint.x += addValue;
      break;
    case 0:
      // do nothing, since outer points already on line of inner box
      break;
    case 1:
      addValue = (int) copysign(1.0, differenceVector.y);
      alignedPoint.y += addValue;
      break;
    case 2:
      addValue = (int) copysign(1.0, differenceVector.y);
      alignedPoint.x += addValue;
      alignedPoint.y += addValue;
      break;
    default:
      // points too far away, let's warn the user that we didn't fix the problem
      std::cout << "WARNING: Rotated feature has accuracy problems!" << std::endl;
  }

  return alignedPoint;
}

int HaarFeature::getHeight() {
  return height;
}

int HaarFeature::getWidth() {
  return width;
}

void HaarFeature::updateHeightAndWidth() {
  height = max(max(max(abs(rotatedVectorToP1.y), abs(rotatedVectorToP2.y)), max(abs(rotatedVectorToP3.y), abs(rotatedVectorToP4.y))), max(max(abs(rotatedVectorToP5.y), abs(rotatedVectorToP6.y)), max(abs(rotatedVectorToP7.y), abs(rotatedVectorToP8.y))));
  width = max(max(max(abs(rotatedVectorToP1.x), abs(rotatedVectorToP2.x)), max(abs(rotatedVectorToP3.x), abs(rotatedVectorToP4.x))), max(max(abs(rotatedVectorToP5.x), abs(rotatedVectorToP6.x)), max(abs(rotatedVectorToP7.x), abs(rotatedVectorToP8.x))));
}

int HaarFeatureStraight::haarFeature(int positionY, int positionX) {
  Point2i middlePoint(positionX, positionY);

  Point2i boxes[2][4];
  getCalculationBoxes(middlePoint, boxes);

  return (	(normalizationFactor - 2 * normalizationFactorBlack) * integralImage->boxSum(boxes[0])
            + 2 * normalizationFactorBlack * integralImage->boxSum(boxes[1])
                );
}

void HaarFeatureStraight::initialize(float deltaX, float deltaY, float pixelsPerMeter, float angle) {
  setDeltaXY(round(deltaX * pixelsPerMeter), round(deltaY * pixelsPerMeter));
  setAngle(angle);

  /* case angle (mod Pi) between Pi/8 and 5/8*Pi
   *           P1
   *           /\
   *        P5/  \
   *         /\   \
   *        /  \  /P4
   *       /    \/
   *    P7/     /P8
   *     /\    /
   *  P3/  \  /
   *    \   \/
   *     \  /P6
   *      \/
   *      P2
   *
   * other case
   *
   * P1  P5       P7   P3
   *  ------------------
   *  |   |        |   |
   *  |   |        |   |
   *  ------------------
   * P4  P8       P6   P2
   */

  int factor = angle / M_PI;
  angle = angle - factor * M_PI;
  if(angle < 0) {
    angle += M_PI;
  }

  cv::Point2i vectorToP1;
  cv::Point2i vectorToP2;
  cv::Point2i vectorToP3;
  cv::Point2i vectorToP4;
  cv::Point2i vectorToP5;
  cv::Point2i vectorToP6;
  cv::Point2i vectorToP7;
  cv::Point2i vectorToP8;

  if(angle > M_PI_8 && angle < M_PI_2 + M_PI_8 ) {
    vectorToP1 = Point2i(this->deltaX, -this->deltaY);
    vectorToP2 = Point2i(-this->deltaX, this->deltaY);
    vectorToP3 = Point2i(this->deltaX, this->deltaY);
    vectorToP4 = Point2i(-this->deltaX, -this->deltaY);
    vectorToP5 = Point2i(this->deltaX/2, -this->deltaY);
    vectorToP6 = Point2i(-this->deltaX/2, this->deltaY);
    vectorToP7 = Point2i(this->deltaX/2, this->deltaY);
    vectorToP8 = Point2i(-this->deltaX/2, -this->deltaY);
  } else {
    vectorToP1 = Point2i(-this->deltaX, -this->deltaY);
    vectorToP2 = Point2i(this->deltaX, this->deltaY);
    vectorToP3 = Point2i(this->deltaX, -this->deltaY);
    vectorToP4 = Point2i(-this->deltaX, this->deltaY);
    vectorToP5 = Point2i(-this->deltaX/2, -this->deltaY);
    vectorToP6 = Point2i(this->deltaX/2, this->deltaY);
    vectorToP7 = Point2i(this->deltaX/2, -this->deltaY);
    vectorToP8 = Point2i(-this->deltaX/2, this->deltaY);
  }

  rotatedVectorToP1 = rotatePoint(vectorToP1);
  rotatedVectorToP2 = rotatePoint(vectorToP2);
  rotatedVectorToP3 = rotatePoint(vectorToP3);
  rotatedVectorToP4 = rotatePoint(vectorToP4);
  rotatedVectorToP5 = rotatePoint(vectorToP5);
  rotatedVectorToP6 = rotatePoint(vectorToP6);
  rotatedVectorToP7 = rotatePoint(vectorToP7);
  rotatedVectorToP8 = rotatePoint(vectorToP8);

  // is box rotated?
  factor = angle/M_PI_2;
  if(angle - factor * M_PI_2 > 0.1 ) {
    // put outer points on line of inner box (if needed)
    rotatedVectorToP1 = alignPointsOnLine(rotatedVectorToP5, rotatedVectorToP1);
    rotatedVectorToP2 = alignPointsOnLine(rotatedVectorToP6, rotatedVectorToP2);
    rotatedVectorToP3 = alignPointsOnLine(rotatedVectorToP7, rotatedVectorToP3);
    rotatedVectorToP4 = alignPointsOnLine(rotatedVectorToP8, rotatedVectorToP4);

    /* second reference points depends on the degree of rotation
                 * secondOuterPoint: outer point along width of filter corresponding to P3
                 * secondInnerPoint: inner point along border corresponding to P7 */
    Point2i secondOuterPoint = rotatedVectorToP1;
    Point2i secondInnerPoint = rotatedVectorToP6;
    if(factor%2 == 0) {
      secondOuterPoint = rotatedVectorToP2;
      secondInnerPoint = rotatedVectorToP5;
    }

    // calculate maximum filter response for rotated feature
    // calculate side lengths of filter mask (after rounding)
    float sideLength1 = abs( (float) (rotatedVectorToP3.y - secondOuterPoint.y) / cosAngle );
    float sideLength2 = abs( (float) (secondInnerPoint.y - rotatedVectorToP7.y) / cosAngle );
    int rotatedMaximumResponse = sideLength1 * sideLength2;
    normalizationFactor = (float) (4 * (this->deltaX / 2) * 2 * this->deltaY) / rotatedMaximumResponse;

    // calculate maximum black response
    float hypothenuseLengthBlack = sqrt( pow( (float) rotatedVectorToP3.x - rotatedVectorToP7.x, 2 ) + pow( (float) rotatedVectorToP3.y - rotatedVectorToP7.y, 2 ) );
    int rotatedMaximumBlackResponseOneSide = 255 * hypothenuseLengthBlack * abs( (float) (rotatedVectorToP7.y - secondInnerPoint.y) / cosAngle );
    int maximumBlackResponseOneSide = 255 * (this->deltaX - this->deltaX/2) * (2 * this->deltaY);

    normalizationFactorBlack = (float) maximumBlackResponseOneSide / rotatedMaximumBlackResponseOneSide;
  }
  else {
    normalizationFactor = 1.0;
    normalizationFactorBlack = 1.0;
  }

  updateHeightAndWidth();
}

void HaarFeature::getCalculationBoxes(Point2i middlePoint, Point2i (&boxes)[2][4]) {
  boxes[0][0] = (Point2i(middlePoint + rotatedVectorToP1));
  boxes[0][1] = (Point2i(middlePoint + rotatedVectorToP2));
  boxes[0][2] = (Point2i(middlePoint + rotatedVectorToP3));
  boxes[0][3] = (Point2i(middlePoint + rotatedVectorToP4));

  boxes[1][0] = (Point2i(middlePoint + rotatedVectorToP5));
  boxes[1][1] = (Point2i(middlePoint + rotatedVectorToP6));
  boxes[1][2] = (Point2i(middlePoint + rotatedVectorToP7));
  boxes[1][3] = (Point2i(middlePoint + rotatedVectorToP8));
}

void HaarFeature::getCalculationBoxes(Point2i middlePoint, Point2i (&boxes)[3][4]) {
  boxes[0][0] = (Point2i(middlePoint + rotatedVectorToP1));
  boxes[0][1] = (Point2i(middlePoint + rotatedVectorToP2));
  boxes[0][2] = (Point2i(middlePoint + rotatedVectorToP3));
  boxes[0][3] = (Point2i(middlePoint + rotatedVectorToP4));

  boxes[1][0] = (Point2i(middlePoint + rotatedVectorToP5));
  boxes[1][1] = (Point2i(middlePoint + rotatedVectorToP6));
  boxes[1][2] = (Point2i(middlePoint + rotatedVectorToP7));
  boxes[1][3] = (Point2i(middlePoint + rotatedVectorToP8));

  boxes[2][0] = (Point2i(middlePoint + rotatedVectorToP9));
  boxes[2][1] = (Point2i(middlePoint + rotatedVectorToP10));
  boxes[2][2] = (Point2i(middlePoint + rotatedVectorToP11));
  boxes[2][3] = (Point2i(middlePoint + rotatedVectorToP12));
}

void HaarFeatureParking::initialize(float widthSideLine, float widthParkingLine, float deltaX, float deltaY, float pixelsPerMeter) {
  pixelsSideLine = round(widthSideLine * pixelsPerMeter);
  pixelsParkingLine = round(widthParkingLine * pixelsPerMeter);
  pixelsTwoCentimeter = round(0.02 * pixelsPerMeter);

  setDeltaXY(round(deltaX * pixelsPerMeter), round(deltaY * pixelsPerMeter));

  // outer box
  rotatedVectorToP1 = Point2i(-this->deltaX, -this->deltaY);
  rotatedVectorToP2 = Point2i(this->deltaX, this->deltaY);
  rotatedVectorToP3 = Point2i(this->deltaX, -this->deltaY);
  rotatedVectorToP4 = Point2i(-this->deltaX, this->deltaY);

  // side line
  rotatedVectorToP5 = Point2i(-pixelsSideLine, -this->deltaY);
  rotatedVectorToP6 = Point2i(0, this->deltaY);
  rotatedVectorToP7 = Point2i(0, -this->deltaY);
  rotatedVectorToP8 = Point2i(-pixelsSideLine, this->deltaY);

  // parking line
  rotatedVectorToP9 = Point2i(pixelsTwoCentimeter, -this->deltaY/2);
  rotatedVectorToP10 = Point2i(this->deltaX, this->deltaY/2);
  rotatedVectorToP11 = Point2i(this->deltaX, -this->deltaY/2);
  rotatedVectorToP12 = Point2i(pixelsTwoCentimeter, this->deltaY/2);

  int areaOuterBox = (2 * this->deltaX) * (2 * this->deltaY);
  int areaLineBoxes = (pixelsSideLine * (2 * this->deltaY)) + ((this->deltaX - pixelsTwoCentimeter) * ((this->deltaY / 2) * 2));
  int areaBlackBoxes = areaOuterBox - areaLineBoxes;
  factorBlack = (float) areaOuterBox / (float) areaBlackBoxes;

  updateHeightAndWidth();
}

void HaarFeatureParking::updateMaximumResponse() {
  int areaSideLineBox = pixelsSideLine * (2 * this->deltaY);
  int areaParkingLineBox = (this->deltaX - pixelsTwoCentimeter) * ((this->deltaY / 2) * 2);

  maximumResponse = (areaSideLineBox + areaParkingLineBox) * 255;
}

int HaarFeatureParking::haarFeature(int positionY, int positionX) {
  Point2i middlePoint(positionX, positionY);

  Point2i boxes[3][4];
  getCalculationBoxes(middlePoint, boxes);

  int sumOuterBox = integralImage->boxSum(boxes[0]);
  int sumSideLineBox = integralImage->boxSum(boxes[1]);
  int sumParkingLineBox = integralImage->boxSum(boxes[2]);

  return (	sumOuterBox
                - factorBlack * (
                  sumOuterBox - sumSideLineBox - sumParkingLineBox
                  )
                );
}

int HaarFeatureParking::getPixelsSideLine() {
  return pixelsSideLine;
}

void HaarFeatureLineEnd::initialize(float widthLine, float deltaX, float deltaY, float pixelsPerMeter, float angle) {
  pixelsLine = round(widthLine * pixelsPerMeter);

  setDeltaXY(round(deltaX * pixelsPerMeter), round(deltaY * pixelsPerMeter));
  setAngle(angle);

  cv::Point2i vectorToP1;
  cv::Point2i vectorToP2;
  cv::Point2i vectorToP3;
  cv::Point2i vectorToP4;
  cv::Point2i vectorToP5;
  cv::Point2i vectorToP6;
  cv::Point2i vectorToP7;
  cv::Point2i vectorToP8;

  // calculate angle in range [0, 2*Pi)
  int factor = angle / (2 * M_PI);
  float angleMod2Pi = angle - factor * 2 * M_PI;

  // if angle positive, 2*Pi has to be added once to put into range [0, 2*Pi)
  if(angleMod2Pi < 0) {
    angleMod2Pi += 2 * M_PI;
  }

  if(angleMod2Pi <= M_PI_8 || angleMod2Pi > M_PI + M_PI_2 + M_PI_8) {
    vectorToP1 = Point2i(-this->deltaX, -this->deltaY);
    vectorToP2 = Point2i(this->deltaX, this->deltaY);
    vectorToP3 = Point2i(this->deltaX, -this->deltaY);
    vectorToP4 = Point2i(-this->deltaX, this->deltaY);
    vectorToP5 = Point2i(-pixelsLine/2, -this->deltaY);
    vectorToP6 = Point2i(pixelsLine/2, 0);
    vectorToP7 = Point2i(pixelsLine/2, -this->deltaY);
    vectorToP8 = Point2i(-pixelsLine/2, 0);
  }
  else if(angleMod2Pi <= (M_PI_2 + M_PI_8)) {
    vectorToP1 = Point2i(this->deltaX, -this->deltaY);
    vectorToP2 = Point2i(-this->deltaX, this->deltaY);
    vectorToP3 = Point2i(this->deltaX, this->deltaY);
    vectorToP4 = Point2i(-this->deltaX, -this->deltaY);
    vectorToP5 = Point2i(pixelsLine/2, -this->deltaY);
    vectorToP6 = Point2i(-pixelsLine/2, 0);
    vectorToP7 = Point2i(pixelsLine/2, 0);
    vectorToP8 = Point2i(-pixelsLine/2, -this->deltaY);
  }
  else if (angleMod2Pi <= (M_PI + M_PI_8)) {
    vectorToP1 = Point2i(this->deltaX, this->deltaY);
    vectorToP2 = Point2i(-this->deltaX, -this->deltaY);
    vectorToP3 = Point2i(-this->deltaX, this->deltaY);
    vectorToP4 = Point2i(this->deltaX, -this->deltaY);
    vectorToP5 = Point2i(pixelsLine/2, 0);
    vectorToP6 = Point2i(-pixelsLine/2, -this->deltaY);
    vectorToP7 = Point2i(-pixelsLine/2, 0);
    vectorToP8 = Point2i(pixelsLine/2, -this->deltaY);
  }
  else if (angleMod2Pi <= (M_PI + M_PI_2 + M_PI_8)) {
    vectorToP1 = Point2i(-this->deltaX, this->deltaY);
    vectorToP2 = Point2i(this->deltaX, -this->deltaY);
    vectorToP3 = Point2i(-this->deltaX, -this->deltaY);
    vectorToP4 = Point2i(this->deltaX, this->deltaY);
    vectorToP5 = Point2i(-pixelsLine/2, 0);
    vectorToP6 = Point2i(pixelsLine/2, -this->deltaY);
    vectorToP7 = Point2i(-pixelsLine/2, -this->deltaY);
    vectorToP8 = Point2i(pixelsLine/2, 0);
  }
  else {
    cout << "WARNING: Problem initializing HaarFeatureLineEnd" << endl << flush;
  }

  rotatedVectorToP1 = rotatePoint(vectorToP1);
  rotatedVectorToP2 = rotatePoint(vectorToP2);
  rotatedVectorToP3 = rotatePoint(vectorToP3);
  rotatedVectorToP4 = rotatePoint(vectorToP4);
  rotatedVectorToP5 = rotatePoint(vectorToP5);
  rotatedVectorToP6 = rotatePoint(vectorToP6);
  rotatedVectorToP7 = rotatePoint(vectorToP7);
  rotatedVectorToP8 = rotatePoint(vectorToP8);

  int areaOuterBox = (2 * this->deltaX) * (2 * this->deltaY);
  int areaLineBox = ((pixelsLine / 2) * 2) * this->deltaY;
  int areaBlackBox = areaOuterBox - areaLineBox;
  factorBlack = (float) areaOuterBox / (float) areaBlackBox;

  //TODO align points of 45° rotated patches!

  updateHeightAndWidth();
}

void HaarFeatureLineEnd::updateMaximumResponse() {
  maximumResponse = pixelsLine * this->deltaY * 255;
}

int HaarFeatureLineEnd::haarFeature(int positionY, int positionX) {
  Point2i middlePoint(positionX, positionY);

  Point2i boxes[2][4];
  getCalculationBoxes(middlePoint, boxes);

  int sumOuterBox = integralImage->boxSum(boxes[0]);
  int sumLineBox = integralImage->boxSum(boxes[1]);

  return (	sumOuterBox
                - factorBlack * (
                  sumOuterBox - sumLineBox
                  )
                );
}

void HaarFeatureCornerOneSide::initialize(float widthLine, float deltaX, float deltaY, float pixelsPerMeter, float angle) {
  pixelsLine = round(widthLine * pixelsPerMeter);

  setDeltaXY(round(deltaX * pixelsPerMeter), round(deltaY * pixelsPerMeter));
  setAngle(angle);

  cv::Point2i vectorToP1;
  cv::Point2i vectorToP2;
  cv::Point2i vectorToP3;
  cv::Point2i vectorToP4;
  cv::Point2i vectorToP5;
  cv::Point2i vectorToP6;
  cv::Point2i vectorToP7;
  cv::Point2i vectorToP8;

  //TODO cases if angle != 0
  vectorToP1 = Point2i(-this->deltaX, -this->deltaY);
  vectorToP2 = Point2i(this->deltaX, this->deltaY);
  vectorToP3 = Point2i(this->deltaX, -this->deltaY);
  vectorToP4 = Point2i(-this->deltaX, this->deltaY);
  vectorToP5 = Point2i(-this->deltaX + pixelsLine, -this->deltaY + pixelsLine);
  vectorToP6 = vectorToP2;
  vectorToP7 = Point2i(this->deltaX, -this->deltaY + pixelsLine);
  vectorToP8 = Point2i(-this->deltaX + pixelsLine, this->deltaY);

  rotatedVectorToP1 = rotatePoint(vectorToP1);
  rotatedVectorToP2 = rotatePoint(vectorToP2);
  rotatedVectorToP3 = rotatePoint(vectorToP3);
  rotatedVectorToP4 = rotatePoint(vectorToP4);
  rotatedVectorToP5 = rotatePoint(vectorToP5);
  rotatedVectorToP6 = rotatePoint(vectorToP6);
  rotatedVectorToP7 = rotatePoint(vectorToP7);
  rotatedVectorToP8 = rotatePoint(vectorToP8);

  int areaOuterBox = (2 * this->deltaX) * (2 * this->deltaY);
  int areaBlackBox = (2 * this->deltaX - pixelsLine) * (2 * this->deltaY - pixelsLine);
  factorBlack = (float) areaOuterBox / (float) areaBlackBox;

  updateHeightAndWidth();
}

void HaarFeatureCornerOneSide::updateMaximumResponse() {
  int outerBox = (2 * this->deltaX) * (2 * this->deltaY);
  int blackBox = (2 * this->deltaX - pixelsLine) * ( 2 * this->deltaY - pixelsLine);
  maximumResponse = (outerBox - blackBox) * 255;
}

int HaarFeatureCornerOneSide::haarFeature(int positionY, int positionX) {
  Point2i middlePoint(positionX, positionY);

  Point2i boxes[2][4];
  getCalculationBoxes(middlePoint, boxes);

  int sumOuterBox = integralImage->boxSum(boxes[0]);
  int sumBlackBox = integralImage->boxSum(boxes[1]);

  return (sumOuterBox - factorBlack * sumBlackBox);
}

void HaarFeatureCorner::initialize(float widthLine, float deltaX, float deltaY, float pixelsPerMeter, float angle) {
  pixelsLine = round(widthLine * pixelsPerMeter);

  setDeltaXY(round(deltaX * pixelsPerMeter), round(deltaY * pixelsPerMeter));
  setAngle(angle);

  cv::Point2i vectorToP1;
  cv::Point2i vectorToP2;
  cv::Point2i vectorToP3;
  cv::Point2i vectorToP4;
  cv::Point2i vectorToP5;
  cv::Point2i vectorToP6;
  cv::Point2i vectorToP7;
  cv::Point2i vectorToP8;
  cv::Point2i vectorToP9;
  cv::Point2i vectorToP10;
  cv::Point2i vectorToP11;
  cv::Point2i vectorToP12;

  // calculate angle in range [0, 2*Pi)
  int factor = angle / (2 * M_PI);
  float angleMod2Pi = angle - factor * 2 * M_PI;

  // if angle positive, 2*Pi has to be added once to put into range [0, 2*Pi)
  if(angleMod2Pi < 0) {
    angleMod2Pi += 2 * M_PI;
  }

  if(angleMod2Pi <= M_PI_8 || angleMod2Pi > M_PI + M_PI_2 + M_PI_8) {
    vectorToP1 = Point2i(-this->deltaX, -this->deltaY);
    vectorToP2 = Point2i(this->deltaX, this->deltaY);
    vectorToP3 = Point2i(this->deltaX, -this->deltaY);
    vectorToP4 = Point2i(-this->deltaX, this->deltaY);
    vectorToP5 = Point2i(-pixelsLine/2, -pixelsLine/2);
    vectorToP6 = vectorToP2;
    vectorToP7 = Point2i(this->deltaX, -pixelsLine/2);
    vectorToP8 = Point2i(-pixelsLine/2, this->deltaY);
    vectorToP9 = Point2i(pixelsLine/2, pixelsLine/2);
    vectorToP10 = vectorToP2;
    vectorToP11 = Point2i(this->deltaX, pixelsLine/2);
    vectorToP12 = Point2i(pixelsLine/2, this->deltaY);
  }
  else if(angleMod2Pi <= (M_PI_2 + M_PI_8)) {
    vectorToP1 = Point2i(this->deltaX, -this->deltaY);
    vectorToP2 = Point2i(-this->deltaX, this->deltaY);
    vectorToP3 = Point2i(this->deltaX, this->deltaY);
    vectorToP4 = Point2i(-this->deltaX, -this->deltaY);
    vectorToP5 = Point2i(this->deltaX, -pixelsLine/2);
    vectorToP6 = Point2i(-pixelsLine/2, this->deltaY);
    vectorToP7 = vectorToP3;
    vectorToP8 = Point2i(-pixelsLine/2, -pixelsLine/2);
    vectorToP9 = Point2i(this->deltaX, pixelsLine/2);
    vectorToP10 = Point2i(pixelsLine/2, this->deltaY);
    vectorToP11 = vectorToP3;
    vectorToP12 = Point2i(pixelsLine/2, pixelsLine/2);
  }
  else if (angleMod2Pi <= (M_PI + M_PI_8)) {
    vectorToP1 = Point2i(this->deltaX, this->deltaY);
    vectorToP2 = Point2i(-this->deltaX, -this->deltaY);
    vectorToP3 = Point2i(-this->deltaX, this->deltaY);
    vectorToP4 = Point2i(this->deltaX, -this->deltaY);
    vectorToP5 = vectorToP1;
    vectorToP6 = Point2i(-pixelsLine/2, -pixelsLine/2);
    vectorToP7 = Point2i(-pixelsLine/2, this->deltaY);
    vectorToP8 = Point2i(this->deltaX, -pixelsLine/2);
    vectorToP9 = vectorToP1;
    vectorToP10 = Point2i(pixelsLine/2, pixelsLine/2);
    vectorToP11 = Point2i(pixelsLine/2, this->deltaY);
    vectorToP12 = Point2i(this->deltaX, pixelsLine/2);
  }
  else if (angleMod2Pi <= (M_PI + M_PI_2 + M_PI_8)) {
    vectorToP1 = Point2i(-this->deltaX, this->deltaY);
    vectorToP2 = Point2i(this->deltaX, -this->deltaY);
    vectorToP3 = Point2i(-this->deltaX, -this->deltaY);
    vectorToP4 = Point2i(this->deltaX, this->deltaY);
    vectorToP5 = Point2i(-pixelsLine/2, this->deltaY);
    vectorToP6 = Point2i(this->deltaX, -pixelsLine/2);
    vectorToP7 = Point2i(-pixelsLine/2, -pixelsLine/2);
    vectorToP8 = vectorToP4;
    vectorToP9 = Point2i(pixelsLine/2, this->deltaY);
    vectorToP10 = Point2i(this->deltaX, pixelsLine/2);
    vectorToP11 = Point2i(pixelsLine/2, pixelsLine/2);
    vectorToP12 = vectorToP4;
  }
  else {
    cout << "WARNING: Problem initializing HaarFeatureCorner" << endl << flush;
  }


  rotatedVectorToP1 = rotatePoint(vectorToP1);
  rotatedVectorToP2 = rotatePoint(vectorToP2);
  rotatedVectorToP3 = rotatePoint(vectorToP3);
  rotatedVectorToP4 = rotatePoint(vectorToP4);
  rotatedVectorToP5 = rotatePoint(vectorToP5);
  rotatedVectorToP6 = rotatePoint(vectorToP6);
  rotatedVectorToP7 = rotatePoint(vectorToP7);
  rotatedVectorToP8 = rotatePoint(vectorToP8);
  rotatedVectorToP9 = rotatePoint(vectorToP9);
  rotatedVectorToP10 = rotatePoint(vectorToP10);
  rotatedVectorToP11 = rotatePoint(vectorToP11);
  rotatedVectorToP12 = rotatePoint(vectorToP12);

  int areaOuterBox = (2 * this->deltaX) * (2 * this->deltaY);
  int areaMiddleBoxWithCorner = (this->deltaX + pixelsLine/2) * (this->deltaY + pixelsLine/2);
  int areaSmallBlackBox = (this->deltaX - pixelsLine/2) * (this->deltaY - pixelsLine/2);
  int areaBlackBox = areaOuterBox - areaMiddleBoxWithCorner + areaSmallBlackBox;
  factorBlack = (float) areaOuterBox / (float) areaBlackBox;

  updateHeightAndWidth();
}

void HaarFeatureCorner::updateMaximumResponse() {
  int middleBoxWithCorner = (this->deltaX + pixelsLine/2) * (this->deltaY + pixelsLine/2);
  int smallBlackBox = (this->deltaX - pixelsLine/2) * (this->deltaY - pixelsLine/2);
  maximumResponse = (middleBoxWithCorner - smallBlackBox) * 255;
}

int HaarFeatureCorner::haarFeature(int positionY, int positionX) {
  Point2i middlePoint(positionX, positionY);

  Point2i boxes[3][4];
  getCalculationBoxes(middlePoint, boxes);

  int sumOuterBox = integralImage->boxSum(boxes[0]);
  int sumMiddleBoxWithCorner = integralImage->boxSum(boxes[1]);
  int sumSmallBlackBox = integralImage->boxSum(boxes[2]);

  return (sumOuterBox - factorBlack * (sumOuterBox - sumMiddleBoxWithCorner + sumSmallBlackBox));
}
