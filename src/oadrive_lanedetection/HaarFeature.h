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


#ifndef HAARFEATURE_H_
#define HAARFEATURE_H_

#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include "IntegralImage.h"
#include "StreetTypes.h"

// define PI/8 since it isn't implemented in C++ by default
#define M_PI_8 0.3926990816987241548078304229099378605246461749218882

class HaarFeature {
public:
  //! set pointer to integral image, must be set before evaluating feature using haarFeature() method
  void setIntegralImage(IntegralImage* integralImage);

  //! setter for width (deltaX) and height (deltaY) of unrotated feature
  void setDeltaXY(int deltaX, int deltaY);

  //! set angle of feature (not every feature is designed to be rotated, at most 45 degree steps are possible)
  void setAngle(float angle);

  //! getter for width of unrotated feature
  int getDeltaX();

  //! getter for height of unrotated feature
  int getDeltaY();

  //! returns the theoretically possible maximum response of the feature (= white area multiplied with 255)
  int getMaximumResponse();

  //! evaluate haar feature at the given position
  virtual int haarFeature(int positionY, int positionX) = 0;

  //! get height of feature calculated from center line to most outer y point
  int getHeight();

  //! get width of feature calculated from center line to most outer x point
  int getWidth();

  //! calculate width and height of feature
  void updateHeightAndWidth();

protected:
  //! saves the maximum response of the feature which is calculated by updateMaximumReponse() function
  int maximumResponse;

  //! factor for black boxes that the size is equal to the outer box
  float factorBlack;

  //! factor for rotated features since their response is smaller
  float normalizationFactor;

  //! factor for rotated features since their black response is different
  float normalizationFactorBlack;

  //! vectors calculated in initialize function to each evaluation point (origin lying in the middle point)
  cv::Point2i rotatedVectorToP1;
  cv::Point2i rotatedVectorToP2;
  cv::Point2i rotatedVectorToP3;
  cv::Point2i rotatedVectorToP4;
  cv::Point2i rotatedVectorToP5;
  cv::Point2i rotatedVectorToP6;
  cv::Point2i rotatedVectorToP7;
  cv::Point2i rotatedVectorToP8;
  cv::Point2i rotatedVectorToP9;
  cv::Point2i rotatedVectorToP10;
  cv::Point2i rotatedVectorToP11;
  cv::Point2i rotatedVectorToP12;

  //! gives half the width of the unrotated feature (sometimes it is also the line width, depends on the feature)
  int deltaX;

  //! gives half the height of the unrotated feature
  int deltaY;

  //! half width of feature (in pixel)
  int width;

  //! half height of feature (in pixel)
  int height;

  //! the angle of the feature of which it is rotated (some features can't be rotated, at most 45 degree steps make sense)
  float angle;

  //! cosine of the chosen angle (saved for performance reasons)
  float cosAngle;

  //! sine of the chosen angle (saved for performance reasons)
  float sinAngle;

  //! pointer to integral image on which calculation of features is performed
  IntegralImage *integralImage;

  //! setter for deltaX variable
  void setDeltaX(int deltaX);

  //! setter for deltaY variable
  void setDeltaY(int deltaY);

  //! calculates the maximum response (= size of white area) of a feature
  virtual void updateMaximumResponse();

  //! rotate a vector by the saved angle, if roundRotatedPoint is enabled, the point position is rounded after rotation, otherwise the position is just casted as integer
  cv::Point2i rotatePoint(cv::Point2i vectorToBeRotated, bool roundRotatedPoint = true);

  //! method used for strait line features to align rotated points on line, otherwise it can happen, that the filter response is negative
  cv::Point2i alignPointsOnLine(cv::Point2i smallRectangle, cv::Point2i bigRectangle);

  //! calculates and returns the coordinates of all boxes according to the given middle point
  void getCalculationBoxes(cv::Point2i middlePoint, cv::Point2i (&boxes)[2][4]);

  //! calculates and returns the coordinates of all boxes according to the given middle point
  void getCalculationBoxes(cv::Point2i middlePoint, cv::Point2i (&boxes)[3][4]);
};

class HaarFeatureStraight : public HaarFeature {
public:
  /* initialize haar feature of straight lines
         * more precisely the 8 coordinates of the points are calculated from the center point
         *
         *  |--|-----|--|
         *  |  |     |  |
         *  |  |     |  |
         *  |  |  X  |  |Λ
         *  |  |     |  ||
         *  |  |     |  || deltaY
         *  |--|-----|--|V
         *  <----->
         *  deltaX
         *
         *
         * P1 P5    P7 P3
         *  |--|-----|--|
         *  |  |     |  |
         *  |  |     |  |
         *  |  |  X  |  |
         *  |  |     |  |
         *  |  |     |  |
         *  |--|-----|--|
         * P4 P8    P6 P2
         *
         *  X in feature image is center point of feature
         */
  void initialize(float deltaX, float deltaY, float pixelsPerMeter, float angle);

  /*! evaluate feature on point (positionX, positionY)
         * ATTENTION: first y variable, second x variable! (following open cv standard)
         */
  int haarFeature(int positionY, int positionX);
};

class HaarFeatureParking : public HaarFeature {
public:
  /*! initialize haar feature of parking lot
   * more precisely the 12 coordinates of the points are calculated from the center point
   *                             deltaX
   *                         <------------>
   *         |-----|--------|--------------|
   *         |     |        |              |
   *         |     |        |      |-------|
   *         |     |        |      |       |Λ
   *         |     |        X      |       || widthParkingLine
   *        Λ|     |        |      |       |V
   * deltaY ||     |        |<2cm->|-------|
   *        ||     |        |              |
   *        V|-----|--------|--------------|
   *                <------>
   *             widthSideLine
   *
   *
   *        P1    P5       P7             P3
   *         |-----|--------|--------------|
   *         |     |        |     P9       |
   *         |     |        |      |-------|P11
   *         |     |        |      |       |
   *         |     |        X      |       |
   *         |     |        |      |       |
   *         |     |        |      |-------|P10
   *         |     |        |     P12      |
   *         |-----|--------|--------------|
   *        P4    P8       P6             P2
   *
   *  X in feature image is center point of feature
   */
  void initialize(float widthSideLine, float widthParkingLine, float deltaX, float deltaY, float pixelsPerMeter);

  /*! evaluate feature on point (positionX, positionY)
   * ATTENTION: first y variable, second x variable! (following open cv standard)
   */
  int haarFeature(int positionY, int positionX);

  //! getter for private variable pixelsSideLine
  int getPixelsSideLine();

protected:
  //! overwrite maximum response function since maximum response is calculated differently
  void updateMaximumResponse();

private:
  //! initialization parameters (set in initialize function)
  int pixelsSideLine;
  int pixelsParkingLine;
  int pixelsTwoCentimeter;
};

class HaarFeatureLineEnd : public HaarFeature {
public:
  /*! initialize haar feature of line end feature
   * more precisely the 8 coordinates of the points are calculated from the center point
   *
   *     widthLine
   *     <----->
   *  |--|-----|--|
   *  |  |     |  |
   *  |  |     |  |
   *  |  |--X--|  |Λ
   *  |           ||
   *  |           || deltaY
   *  |-----------|V
   *  <----->
   *  deltaX
   *
   *
   * P1 P5    P7 P3
   *  |--|-----|--|
   *  |  |     |  |
   *  |  |     |  |
   *  |  |--X--|  |
   *  | P8    P6  |
   *  |           |
   *  |-----------|
   * P4          P2
   *
   *  X in feature image is center point of feature
   */
  void initialize(float widthLine, float deltaX, float deltaY, float pixelsPerMeter, float angle);

  /*! evaluate feature on point (positionX, positionY)
   * ATTATION: first y variable, second x variable! (following open cv standard)
   */
  int haarFeature(int positionY, int positionX);

protected:
  //! overwrite maximum response function since maximum response is calculated differently
  void updateMaximumResponse();

private:
  //! initialization parameters (set in initialize function)
  int pixelsLine;
};

class HaarFeatureCornerOneSide : public HaarFeature {
public:
  /*! initialize haar feature for corners
   *
   *  |--------------------|Λ
   *  |                    ||
   *  |                    || widthLine
   *  |     |--------------|V
   *  |     |    X         |Λ
   *  |     |              ||
   *  |     |              || deltaY
   *  |-----|--------------|V
   *  <----->    <--------->
   *  widthLine   deltaX
   *
   * P1                   P3
   *  |--------------------|
   *  |                    |
   *  |    P5              |
   *  |     |--------------|P7
   *  |     |    X         |
   *  |     |              |
   *  |     |              |
   *  |-----|--------------|
   * P4    P8             P2=P6
   *
   * deltaX and deltaY are always calculated from middle Point X
   * the line is always on the left and top side => black box is only in the lower right corner
   */
  void initialize(float widthLine, float deltaX, float deltaY, float pixelsPerMeter, float angle);

  /*! evaluate feature on point (positionX, positionY)
   * WARNING: first y variable, second x variable! (following open cv standard)
   */
  int haarFeature(int positionY, int positionX);

protected:
  //! overwrite maximum response function since maximum response is calculated differently
  void updateMaximumResponse();

private:
  //! initialization parameters (set in initialize function)
  int pixelsLine;
};

class HaarFeatureCorner : public HaarFeature {
public:
  /*! initialize haar feature for corners
   *
   *                   deltaX
   *                   <--------->
   *         |--------------------|
   *         |                    |
   *         |                    |
   *         |     |--------------|Λ
   *         |     |              ||
   *         |     |   X          || widthLine
   *        Λ|     |     ---------|V
   *        ||     |     |        |
   * deltaY ||     |     |        |
   *        ||     |     |        |
   *        V|-----|-----|--------|
   *                <--->
   *                widthLine
   *
   * P1                   P3
   *  |--------------------|
   *  |                    |
   *  |    P5              |
   *  |     |--------------|P7
   *  |     |              |
   *  |     |   X P9       |
   *  |     |     ---------|P11
   *  |     |     |        |
   *  |     |     |        |
   *  |     |     |        |
   *  |-----|-----|--------|
   * P4    P8    P12      P2=P6=P10
   *
   * deltaX and deltaY are always calculated from middle Point X
   * the center of the corner is always the center of the feature => black parts are on both sides of the line/corner
   */
  void initialize(float widthLine, float deltaX, float deltaY, float pixelsPerMeter, float angle);

  /*! evaluate feature on point (positionX, positionY)
   * WARNING: first y variable, second x variable! (following open cv standard)
   */
  int haarFeature(int positionY, int positionX);

protected:
  //! overwrite maximum response function since maximum response is calculated differently
  void updateMaximumResponse();

private:
  //! initialization parameters (set in initialize function)
  int pixelsLine;

};

#endif
