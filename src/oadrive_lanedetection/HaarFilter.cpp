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
 * \author  Jan-Markus Gomer <uecxl@student.kit.edu>
 * \date    2015-11-01
 *
 */
//----------------------------------------------------------------------

#include "HaarFilter.h"
#include "lanedetectionLogging.h"
#include <opencv2/flann/timer.h>

using namespace std;
using namespace oadrive::util;

using icl_core::logging::endl;
using icl_core::logging::flush;

namespace oadrive {
namespace lanedetection {

HaarFilter::HaarFilter( CoordinateConverter* converter )
  : mCoordConverter( converter )
{
  LOGGING_INFO(lanedetectionLogger, "[HaarFilter] created."<<endl);
  startup();
}

void HaarFilter::startup() {
  // reset maximum offset
  maxOffsetX = 0;
  maxOffsetY = 0;

  maxParkingOffsetX = 0;
  maxParkingOffsetY = 0;

  pixelsPerMeter = mCoordConverter->getPixelsPerMeter();

  // initialize side line feature
  float haarFeatureSideStraightWidth = 0.035;
  float haarFeatureSideStraightHeight = 0.02;

  mHaarFeatureSideStraightY.initialize(haarFeatureSideStraightWidth, haarFeatureSideStraightHeight, pixelsPerMeter, 0.0);
  updateMaximumOffset(mHaarFeatureSideStraightY);
  mHaarFeatureSideStraightRotatedY.initialize(haarFeatureSideStraightWidth, haarFeatureSideStraightHeight, pixelsPerMeter, M_PI_4);
  updateMaximumOffset(mHaarFeatureSideStraightRotatedY);
  mHaarFeatureSideStraightX.initialize(haarFeatureSideStraightWidth, haarFeatureSideStraightHeight, pixelsPerMeter, M_PI_2);
  updateMaximumOffset(mHaarFeatureSideStraightX);
  mHaarFeatureSideStraightRotatedX.initialize(haarFeatureSideStraightWidth, haarFeatureSideStraightHeight, pixelsPerMeter, M_PI_2 + M_PI_4);
  updateMaximumOffset(mHaarFeatureSideStraightRotatedX);
  LOGGING_INFO(lanedetectionLogger, "[HaarFilter] side line features created with line width " << haarFeatureSideStraightWidth * pixelsPerMeter << "pixel" << endl);

  // initialize center line feature
  float haarFeatureCenterStraightWidth = 0.02;
  float haarFeatureCenterStraightHeight = 0.02;

  mHaarFeatureCenterStraightY.initialize(haarFeatureCenterStraightWidth, haarFeatureCenterStraightHeight, pixelsPerMeter, 0.0);
  updateMaximumOffset(mHaarFeatureCenterStraightY);
  mHaarFeatureCenterStraightRotatedY.initialize(haarFeatureCenterStraightWidth, haarFeatureCenterStraightHeight, pixelsPerMeter, M_PI_4);
  updateMaximumOffset(mHaarFeatureCenterStraightRotatedY);
  mHaarFeatureCenterStraightX.initialize(haarFeatureCenterStraightWidth, haarFeatureCenterStraightHeight, pixelsPerMeter, M_PI_2);
  updateMaximumOffset(mHaarFeatureCenterStraightX);
  mHaarFeatureCenterStraightRotatedX.initialize(haarFeatureCenterStraightWidth, haarFeatureCenterStraightHeight, pixelsPerMeter, M_PI_2 + M_PI_4);
  updateMaximumOffset(mHaarFeatureCenterStraightRotatedX);
  LOGGING_INFO(lanedetectionLogger, "[HaarFilter] center line features created with line width " << haarFeatureCenterStraightWidth * pixelsPerMeter << "pixel" << endl);

  // initialize crossroad line feature
  float haarFeatureCrossroadLineWidth = 0.055;
  float haarFeatureCrossroadLineHeight = 0.05;

  mHaarFeatureCrossroadLineY.initialize(haarFeatureCrossroadLineWidth, haarFeatureCrossroadLineHeight, pixelsPerMeter, 0.0);
  updateMaximumOffset(mHaarFeatureCrossroadLineY);
  mHaarFeatureCrossroadLineRotatedY.initialize(haarFeatureCrossroadLineWidth, haarFeatureCrossroadLineHeight, pixelsPerMeter, M_PI_4);
  updateMaximumOffset(mHaarFeatureCrossroadLineRotatedY);
  mHaarFeatureCrossroadLineX.initialize(haarFeatureCrossroadLineWidth, haarFeatureCrossroadLineHeight, pixelsPerMeter, M_PI_2);
  updateMaximumOffset(mHaarFeatureCrossroadLineX);
  mHaarFeatureCrossroadLineRotatedX.initialize(haarFeatureCrossroadLineWidth, haarFeatureCrossroadLineHeight, pixelsPerMeter, M_PI_2 + M_PI_4);
  updateMaximumOffset(mHaarFeatureCrossroadLineRotatedX);
  LOGGING_INFO(lanedetectionLogger, "[HaarFilter] cross road stop line features created with line width " << haarFeatureCrossroadLineWidth * pixelsPerMeter << "pixel" << endl);

  // initialize crossroad line feature
  float haarFeatureCrossroadLineWidthWashedOut = 0.08;
  float haarFeatureCrossroadLineHeightWashedOut = 0.05;

  mHaarFeatureCrossroadLineWashedOutRotatedY.initialize(haarFeatureCrossroadLineWidthWashedOut, haarFeatureCrossroadLineHeightWashedOut, pixelsPerMeter, M_PI_4);
  updateMaximumOffset(mHaarFeatureCrossroadLineWashedOutRotatedY);
  mHaarFeatureCrossroadLineWashedOutX.initialize(haarFeatureCrossroadLineWidthWashedOut, haarFeatureCrossroadLineHeightWashedOut, pixelsPerMeter, M_PI_2);
  updateMaximumOffset(mHaarFeatureCrossroadLineWashedOutX);
  mHaarFeatureCrossroadLineWashedOutRotatedX.initialize(haarFeatureCrossroadLineWidthWashedOut, haarFeatureCrossroadLineHeightWashedOut, pixelsPerMeter, M_PI_2 + M_PI_4);
  updateMaximumOffset(mHaarFeatureCrossroadLineWashedOutRotatedX);
  LOGGING_INFO(lanedetectionLogger, "[HaarFilter] washed out cross road stop line features created with line width " << haarFeatureCrossroadLineWidthWashedOut * pixelsPerMeter << "pixel" << endl);

  // initialize parking line feature
  float haarFeatureParkingLineDeltaX = 0.02;
  float haarFeatureParkingLineDeltaY = 0.02;

  mHaarFeatureParkingLineX.initialize(haarFeatureParkingLineDeltaX, haarFeatureParkingLineDeltaY, pixelsPerMeter, M_PI_2);
  updateMaximumParkingOffset(mHaarFeatureParkingLineX);
  LOGGING_INFO(lanedetectionLogger, "[HaarFilter] parking line feature created with parking line width " << haarFeatureParkingLineDeltaX * pixelsPerMeter << "pixel" << endl);

  // calculate factors between maximum responses to be able to compare responses (all scaled to one reference scale)
  factorSide2Center = (float) mHaarFeatureSideStraightY.getMaximumResponse() / (float) mHaarFeatureCenterStraightY.getMaximumResponse();
  factorParkingLine2Center = (float) mHaarFeatureParkingLineX.getMaximumResponse() / (float) mHaarFeatureCenterStraightY.getMaximumResponse();
  factorCrossroad2Center = (float) mHaarFeatureCrossroadLineY.getMaximumResponse() / (float) mHaarFeatureCenterStraightY.getMaximumResponse();
  factorCrossroadWashedOut2Center = (float) mHaarFeatureCrossroadLineWashedOutX.getMaximumResponse() / (float) mHaarFeatureCenterStraightY.getMaximumResponse();
  factorCrossroad2Side = (float) mHaarFeatureCrossroadLineY.getMaximumResponse() / (float) mHaarFeatureSideStraightY.getMaximumResponse();
  factorCrossroadWashedOut2Crossroad = (float) mHaarFeatureCrossroadLineWashedOutRotatedY.getMaximumResponse() / (float) mHaarFeatureCrossroadLineY.getMaximumResponse();

  // calculate bird view cone
  std::vector<cv::Point2f> conePoints;
  mCoordConverter->getConePoints(conePoints);

  if(conePoints.size() == 4) {
    cv::Point2i pointLeft1 = conePoints[0];
    cv::Point2i pointLeft2 = conePoints[1];
    cv::Point2i pointRight1 = conePoints[2];
    cv::Point2i pointRight2 = conePoints[3];

    mLeft = (float) (pointLeft2.y - pointLeft1.y) / (pointLeft2.x - pointLeft1.x);
    bLeft = pointLeft1.y - mLeft * pointLeft1.x;
    mRight = (float) (pointRight2.y - pointRight1.y) / (pointRight2.x - pointRight1.x);
    bRight = pointRight1.y - mRight * pointRight1.x;
  }
  else {
    // if cone points cannot be calculated, use dummy points outside of image => all points on image will be processed
    mLeft = 1;
    bLeft = 2000;
    mRight = 1;
    bRight = -2000;

    LOGGING_ERROR(lanedetectionLogger, "[HaarFilter] calculation of bird view cone unsuccessful, searching in whole image for features");
  }

  // reset initialization at startup
  resetInitialization();
}

HaarFilter::~HaarFilter() { }

void HaarFilter::setImage( cv::Mat& image ) {
  // save image size for feature detection
  mInputSize = image.size();

  // set input image (expected as gray image)
  mImage = image;

  cv::Mat imageWithBorder(image.rows + 2 * maxOffsetX, image.rows + 2 * maxOffsetY, image.type());

  // add border to Image
  cv::copyMakeBorder(image, imageWithBorder, maxOffsetY, maxOffsetY, maxOffsetX, maxOffsetX, cv::BORDER_REPLICATE);

  // calculate integral image and rotated integral image
  cv::integral( imageWithBorder, mIntegralImage, mSquaredIntegralImage, mRotatedIntegralImage);

  // pass calculated integral image to integral image object
  mIntegralImageObject.setIntegralImage(mIntegralImage);

  // pass calculated rotated integral image to integral image object
  mIntegralImageRotatedObject.setIntegralImage(mRotatedIntegralImage);

  // set proper integral image object to side straight features
  mHaarFeatureSideStraightY.setIntegralImage(&mIntegralImageObject);
  mHaarFeatureSideStraightRotatedY.setIntegralImage(&mIntegralImageRotatedObject);
  mHaarFeatureSideStraightX.setIntegralImage(&mIntegralImageObject);
  mHaarFeatureSideStraightRotatedX.setIntegralImage(&mIntegralImageRotatedObject);

  // set proper integral image object to center straight features
  mHaarFeatureCenterStraightY.setIntegralImage(&mIntegralImageObject);
  mHaarFeatureCenterStraightRotatedY.setIntegralImage(&mIntegralImageRotatedObject);
  mHaarFeatureCenterStraightX.setIntegralImage(&mIntegralImageObject);
  mHaarFeatureCenterStraightRotatedX.setIntegralImage(&mIntegralImageRotatedObject);

  // set proper integral image object to cross road line features
  mHaarFeatureCrossroadLineY.setIntegralImage(&mIntegralImageObject);
  mHaarFeatureCrossroadLineRotatedY.setIntegralImage(&mIntegralImageRotatedObject);
  mHaarFeatureCrossroadLineX.setIntegralImage(&mIntegralImageObject);
  mHaarFeatureCrossroadLineRotatedX.setIntegralImage(&mIntegralImageRotatedObject);

  // set proper integral image object to washed out cross road line features
  mHaarFeatureCrossroadLineWashedOutX.setIntegralImage(&mIntegralImageObject);
  mHaarFeatureCrossroadLineWashedOutRotatedX.setIntegralImage(&mIntegralImageRotatedObject);
  mHaarFeatureCrossroadLineWashedOutRotatedY.setIntegralImage(&mIntegralImageRotatedObject);

  // set proper integral image object to parking line feature
  mHaarFeatureParkingLineX.setIntegralImage(&mIntegralImageObject);

  // clear feature vector (such that only new detections are saved)
  mFeatureVector.clear();

  // reset probability (which is used to normalize probability)
  maximumProbability = 0;
}

void HaarFilter::calculateFeaturesPerLine(bool detectCrossroad /*= false*/, bool detectParkingLot /*= false*/) {
  //  cvflann::StartStopTimer timer;
  //  timer.start();

  // check if image is present, otherwise leave function
  if(mImage.empty()) {
    LOGGING_ERROR(lanedetectionLogger, "[HaarFilter] no valid image provided with setImage function");

    return;
  }

  int currentAmountResponsesSide = AMOUNT_RESPONSES_SIDE;
  int currentAmountResponsesCenter = AMOUNT_RESPONSES_CENTER;
  int currentAmountResponsesParkingLine = AMOUNT_RESPONSES_PARKING_LINE;

  // array for highest feature responses of side line
  int** highestResponsesSide= new int*[currentAmountResponsesSide];
  for(int i = 0; i < currentAmountResponsesSide; i++) {
    highestResponsesSide[i] = new int[4];
    for(int j = 0; j < 4; j++) {
      highestResponsesSide[i][j] = 0;
    }
  }

  // array for highest feature responses of center line
  int** highestResponsesCenter = new int*[currentAmountResponsesCenter];
  for(int i = 0; i < currentAmountResponsesCenter; i++) {
    highestResponsesCenter[i] = new int[4];
    for(int j = 0; j < 4; j++) {
      highestResponsesCenter[i][j] = 0;
    }
  }

  // array for highest feature responses of cross road line
  int** highestResponsesCrossroad = new int*[AMOUNT_RESPONSES_CROSSROAD];
  if(detectCrossroad) {
    for(int i = 0; i < AMOUNT_RESPONSES_CROSSROAD; i++) {
      highestResponsesCrossroad[i] = new int[4];
      for(int j = 0; j < 4; j++) {
        highestResponsesCrossroad[i][j] = 0;
      }
    }
  }

  // array for highest feature responses of parking lines
  int** highestResponsesParkingLine = new int*[currentAmountResponsesParkingLine];
  if(detectParkingLot) {
    for(int i = 0; i < currentAmountResponsesParkingLine; i++) {
      highestResponsesParkingLine[i] = new int[4];
      for(int j = 0; j < 4; j++) {
        highestResponsesParkingLine[i][j] = 0;
      }
    }
  }

  // initialize lots of temp variables used during detection
  int x, y, y1, y2, yMax, maxSide, maxCenter, weightCenterY, weightCenterRotatedX, weightCenterRotatedY, weightSideX, weightSideY, weightSideRotatedX, weightSideRotatedY, weightCrossroadY = 0, weightCrossroadX = 0, weightCrossroadRotatedY = 0, weightCrossroadRotatedX = 0, absoluteMaxSide, absoluteMaxCenter, absoluteMaxCrossroad, absoluteMax;

  //int totalMaxSide = 0, totalMaxCross = 0;

  // go through every STEPSIZE_X-th column and every STEPSIZE_Y-th row and detect features (horizontal scrolling through picture)
  for(int i = maxOffsetX, endI = maxOffsetX + mInputSize.width; i < endI; i += STEPSIZE_X ) {
    y1 = mLeft * (i - maxOffsetX) + bLeft;
    y2 = mRight * (i - maxOffsetX) + bRight;
    yMax = min(mInputSize.height, min(y1, y2)) + maxOffsetY;
    for (int j = maxOffsetY; j < yMax; j += STEPSIZE_Y ) {
      x = i;
      y = j;

      // temporarily save responses of side straight features and save maximum of all four responses
      weightSideY = mHaarFeatureSideStraightY.haarFeature(y, x);
      weightSideRotatedY = mHaarFeatureSideStraightRotatedY.haarFeature(y, x);
      weightSideRotatedX = mHaarFeatureSideStraightRotatedX.haarFeature(y, x);
      maxSide = max(weightSideY, max(weightSideRotatedY, weightSideRotatedX));

      // additional factor for center feature, don't know why this factor is needed, but experiments have shown, that it helps to detect center features better
      float factorCenter = 1.3;
      //temporarily save responses of center straight features and save maximum of all four responses
      weightCenterY = factorCenter * mHaarFeatureCenterStraightY.haarFeature(y, x);
      weightCenterRotatedY = factorCenter * mHaarFeatureCenterStraightRotatedY.haarFeature(y, x);
      weightCenterRotatedX = factorCenter * mHaarFeatureCenterStraightRotatedX.haarFeature(y, x);
      maxCenter = max(weightCenterY, max(weightCenterRotatedY, weightCenterRotatedX));

      // temporarily save responses of cross road vertical line features
      /*if(detectCrossroad && y > DETECT_CROSSROAD_Y_FROM_TOP) {
        weightCrossroadY = mHaarFeatureCrossroadLineY.haarFeature(y, x);
      } else {*/
        weightCrossroadY = 0;
      //}

      // normalize absolute responses such that they are comparable
      absoluteMaxSide = (float)maxSide / factorSide2Center;
      absoluteMaxCenter = maxCenter;
     // absoluteMaxCrossroad = (float)weightCrossroadY / factorCrossroad2Center;
      //absoluteMax = max(absoluteMaxCenter, max(absoluteMaxSide, absoluteMaxCrossroad));
      absoluteMax = max(absoluteMaxCenter, absoluteMaxSide);

      /*if(totalMaxSide < absoluteMaxSide) {
        totalMaxSide = absoluteMaxSide;
      }
      if(totalMaxCross < absoluteMaxCrossroad) {
        totalMaxCross = absoluteMaxCrossroad;
      }*/

      // only use feature with highest responses, all other results are discarded
      if(absoluteMax == absoluteMaxSide) {
        saveHighestResponses(highestResponsesSide, currentAmountResponsesSide, weightSideY, x, y, ORIENTATION_Y);
        saveHighestResponses(highestResponsesSide, currentAmountResponsesSide, weightSideRotatedY, x, y, ORIENTATION_Y_ROTATED);
        saveHighestResponses(highestResponsesSide, currentAmountResponsesSide, weightSideRotatedX, x, y, ORIENTATION_X_ROTATED);
      }
      /*else if(detectCrossroad && absoluteMax == absoluteMaxCrossroad) {
        saveHighestResponses(highestResponsesCrossroad, AMOUNT_RESPONSES_CROSSROAD, weightCrossroadY, x, y, ORIENTATION_Y);
      }*/
      else {
        saveHighestResponses(highestResponsesCenter, currentAmountResponsesCenter, weightCenterY, x, y, ORIENTATION_Y);
        saveHighestResponses(highestResponsesCenter, currentAmountResponsesCenter, weightCenterRotatedY, x, y, ORIENTATION_Y_ROTATED);
        saveHighestResponses(highestResponsesCenter, currentAmountResponsesCenter, weightCenterRotatedX, x, y, ORIENTATION_X_ROTATED);
      }
    }
  }

  // go through every STEPSIZE_X-th column and every STEPSIZE_Y-th row and detect features (vertical scrolling through picture)
  if(detectCrossroad) {
    for(int x = maxOffsetX, endX = mInputSize.width + maxOffsetX; x < endX; x += STEPSIZE_Y) {
      y1 = mLeft * (x - maxOffsetX) + bLeft;
      y2 = mRight * (x - maxOffsetX) + bRight;
      yMax = min(mInputSize.height, min(y1, y2)) + maxOffsetX;
      for (int y = maxOffsetY; y < yMax - 1; y += STEPSIZE_X ) {
        // washed out lines (horizontal lines are always bigger than in real)
        if(y < DETECT_CROSSROAD_Y_FROM_TOP) {
          // side line
         /* weightSideX = mHaarFeatureCrossroadLineX.haarFeature(y, x);
          weightSideRotatedY = mHaarFeatureCrossroadLineRotatedY.haarFeature(y, x);
          weightSideRotatedX = mHaarFeatureCrossroadLineRotatedX.haarFeature(y, x);
          absoluteMaxSide = (float)max(weightSideX, max(weightSideRotatedY, weightSideRotatedX)) / factorCrossroad2Center;
          weightSideX = weightSideX / factorCrossroad2Side;
          weightSideRotatedY = weightSideRotatedY / factorCrossroad2Side;
          weightSideRotatedX = weightSideRotatedX / factorCrossroad2Side;

          // cross road line
          weightCrossroadX = mHaarFeatureCrossroadLineWashedOutX.haarFeature(y, x);
          weightCrossroadRotatedY = mHaarFeatureCrossroadLineWashedOutRotatedY.haarFeature(y, x);
          weightCrossroadRotatedX = mHaarFeatureCrossroadLineWashedOutRotatedX.haarFeature(y, x);
          absoluteMaxCrossroad = (float)max(weightCrossroadX, max(weightCrossroadRotatedY, weightCrossroadRotatedX)) / factorCrossroadWashedOut2Center;
          weightCrossroadX = weightCrossroadX / factorCrossroadWashedOut2Crossroad;
          weightCrossroadRotatedY = weightCrossroadRotatedY / factorCrossroadWashedOut2Crossroad;
          weightCrossroadRotatedX = weightCrossroadRotatedX / factorCrossroadWashedOut2Crossroad;*/
        }
        else {
          // side line
          weightSideX = mHaarFeatureSideStraightX.haarFeature(y, x);
          weightSideRotatedY = mHaarFeatureSideStraightRotatedY.haarFeature(y, x);
          weightSideRotatedX = mHaarFeatureSideStraightRotatedX.haarFeature(y, x);
          absoluteMaxSide = (float)max(weightSideX, max(weightSideRotatedY, weightSideRotatedX)) / factorSide2Center;

          // cross road line
          weightCrossroadX = mHaarFeatureCrossroadLineX.haarFeature(y, x);
          weightCrossroadRotatedY = mHaarFeatureCrossroadLineRotatedY.haarFeature(y, x);
          weightCrossroadRotatedX = mHaarFeatureCrossroadLineRotatedX.haarFeature(y, x);
          absoluteMaxCrossroad = (float)max(weightCrossroadX, max(weightCrossroadRotatedY, weightCrossroadRotatedX)) / factorCrossroad2Center;
        }

        /*if(totalMaxSide < absoluteMaxSide) {
          totalMaxSide = absoluteMaxSide;
        }
        if(totalMaxCross < absoluteMaxCrossroad) {
          totalMaxCross = absoluteMaxCrossroad;
        }*/

        absoluteMax = max(absoluteMaxSide, absoluteMaxCrossroad);

        if(absoluteMax == absoluteMaxSide) {
          if(detectParkingLot) {
            saveHighestResponses(highestResponsesParkingLine, currentAmountResponsesParkingLine, weightSideX, x, y, ORIENTATION_X);
          } else {
            saveHighestResponses(highestResponsesSide, currentAmountResponsesSide, weightSideX, x, y, ORIENTATION_X);
          }
          saveHighestResponses(highestResponsesSide, currentAmountResponsesSide, weightSideRotatedY, x, y, ORIENTATION_Y_ROTATED);
          saveHighestResponses(highestResponsesSide, currentAmountResponsesSide, weightSideRotatedX, x, y, ORIENTATION_X_ROTATED);
        } else {
          saveHighestResponses(highestResponsesCrossroad, AMOUNT_RESPONSES_CROSSROAD, weightCrossroadX, x, y, ORIENTATION_X);
          saveHighestResponses(highestResponsesCrossroad, AMOUNT_RESPONSES_CROSSROAD, weightCrossroadRotatedY, x, y, ORIENTATION_Y_ROTATED);
          saveHighestResponses(highestResponsesCrossroad, AMOUNT_RESPONSES_CROSSROAD, weightCrossroadRotatedX, x, y, ORIENTATION_X_ROTATED);
        }
      }
    }
  }
/*std::cout << "max cross (vertical): " << totalMaxCross << std::endl;
std::cout << "max side (vertical): " << totalMaxSide << std::endl;*/
  // detection of parking lots only in special region (detection of cross road corners is similar, but so far only implemented for one corner type)
  if(detectParkingLot) {
    int offsetParkingX = 0;
    int offsetParkingY = 0;

    // counter used that only every PARKING_LINE_X_SKIP loop the line detection is active
    int counterParkingLine = 0;

    if(detectParkingLot) {
      offsetParkingX = maxParkingOffsetX;
      offsetParkingY = maxParkingOffsetY;
    }
    int offsetX = max(PARKING_MIN_X_OFFSET_RIGHT, offsetParkingX);
    int offsetY = offsetParkingY;
    int weightParkingLineX;



    for(int x = max(offsetX, maxOffsetX), endX = mInputSize.width + maxOffsetX - (offsetParkingX - maxOffsetX); x < endX; x += STEPSIZE_PARKING_X) {
      y1 = mLeft * (x - maxOffsetX) + bLeft;
      y2 = mRight * (x - maxOffsetX) + bRight;
      yMax = min(mInputSize.height, min(y1, y2)) + maxOffsetY - (offsetY - maxOffsetY);
      for (int y = max(offsetY, maxOffsetY); y < yMax; y += STEPSIZE_PARKING_Y) {
        // right part of image

        if(counterParkingLine == 0) {
          // orthogonal lines of parking lot (additional factor of two needed to get enough detections)
          weightParkingLineX = 2 * mHaarFeatureParkingLineX.haarFeature(y, x);
          saveHighestResponses(highestResponsesParkingLine, currentAmountResponsesParkingLine, weightParkingLineX, x, y, ORIENTATION_X);
        }

        counterParkingLine++;
        if(counterParkingLine == PARKING_LINE_X_SKIP) {
          counterParkingLine = 0;
        }
      }
    }
  }

  // calculate moving average by using historic minimum responses of side straight, center straight and cross road straight (if enabled) features
  updateMovingAverage(responsesSideStraight, SIDE_LINE);
  updateMovingAverage(responsesCenterStraight, CENTER_LINE);
  if(detectCrossroad) {
    updateMovingAverage(responsesCrossroadStraight, CROSSROAD_LINE);
    overallMovingAverage = max(movingAverages[CENTER_LINE], ((int) ((float) movingAverages[CROSSROAD_LINE] / factorCrossroad2Center)));
  }
  else {
    overallMovingAverage = movingAverages[CENTER_LINE];
  }
  overallMovingAverage = max(((int) ((float) movingAverages[SIDE_LINE] / factorSide2Center)), overallMovingAverage);

  // update minimum response for every feature type
  movingAverages[SIDE_LINE] = overallMovingAverage * factorSide2Center;
  movingAverages[CENTER_LINE] = overallMovingAverage;

  // calculate amount of columns, rows and cells of grid
  int gridColumns = ceil((float) mInputSize.width / GRID_X);
  int gridRows = ceil((float) mInputSize.height / GRID_Y);
  int gridCells = gridColumns * gridRows;

  // initialize array to sort features into grid and only use highest one
  float** gridResponses = new float*[gridCells];
  for(int i = 0; i < gridCells; i++) {
    /* for every grid response the following values are saved
                        0: probability
                        1: type
                        2: position x
                        3: position y
                        4: angle
                */
    gridResponses[i] = new float[5];
    for(int j = 0; j < 5; j++) {
      gridResponses[i][j] = 0;
    }
  }

  // put side straight features into grid
  extractStraightFeatures(highestResponsesSide, gridResponses, currentAmountResponsesSide, &responsesSideStraight, SIDE_LINE, &mHaarFeatureSideStraightY, &mHaarFeatureSideStraightX, &mHaarFeatureSideStraightRotatedY, &mHaarFeatureSideStraightRotatedX);
  // put center straight features into grid
  extractStraightFeatures(highestResponsesCenter, gridResponses, currentAmountResponsesCenter, &responsesCenterStraight, CENTER_LINE, &mHaarFeatureCenterStraightY, &mHaarFeatureCenterStraightX, &mHaarFeatureCenterStraightRotatedY, &mHaarFeatureCenterStraightRotatedX);

  // if detection enabled, update minimum response of cross road line and put features into grid
  if(detectCrossroad) {
    movingAverages[CROSSROAD_LINE] = overallMovingAverage * factorCrossroad2Center;
    extractStraightFeatures(highestResponsesCrossroad, gridResponses, AMOUNT_RESPONSES_CROSSROAD, &responsesCrossroadStraight, CROSSROAD_LINE, &mHaarFeatureCrossroadLineY, &mHaarFeatureCrossroadLineX, &mHaarFeatureCrossroadLineRotatedY, &mHaarFeatureCrossroadLineRotatedX);
  }

  // if detection enabled, update minimum response of parking lot and put features directly to feature list (because parking lot features should always be given if response big enough)
  if(detectParkingLot) {
    movingAverages[PARKING_LINE] = overallMovingAverage * factorParkingLine2Center;
    extractStraightFeatures(highestResponsesParkingLine, NULL, currentAmountResponsesParkingLine, NULL, PARKING_LINE, &mHaarFeatureParkingLineX, NULL, NULL, NULL);
  }

  // read grid and put every feature stored in grid, into feature list
  for(int i = 0; i < gridCells; i++) {
    if(gridResponses[i][0] > 0.00001) {
      Feature newFeature;

      newFeature.pose = mCoordConverter->pixel2Car( cv::Point2f( gridResponses[i][2] - maxOffsetX, gridResponses[i][3] - maxOffsetY ) );
      newFeature.pose.setYaw( gridResponses[i][4] );
      newFeature.type = (FeatureType) gridResponses[i][1];
      newFeature.probability = gridResponses[i][0];
      newFeature.frameCounter = 0;

      mFeatureVector.push_back( newFeature );
    }
  }

  // set filter to initialized if enough features are detected
  if(!initialized && mFeatureVector.size() >= AMOUNT_RESPONSES_INITIALISED) {
    initialized = true;
    LOGGING_INFO(lanedetectionLogger, "[HaarFilter] filter is now initialized" << endl);
  }

//  timer.stop();
//  std::cout << "feature calculation took " << timer.value << " seconds" << std::endl;
}

void HaarFilter::updateMovingAverage(list<int>& lastResponses, FeatureType type) {
  int movingAverage = 0;
  for (std::list<int>::const_iterator iterator = lastResponses.begin(), end = lastResponses.end(); iterator != end; ++iterator) {
    movingAverage += *iterator;
  }
  movingAverages[type] = movingAverage / lastResponses.size();
}

void HaarFilter::resetInitialization() {
  initialized = false;

  responsesSideStraight.clear();
  for(int i = 0; i < NUMBER_OF_POINTS_MOVING_AVERAGE; i++) {
    responsesSideStraight.push_back( mHaarFeatureSideStraightY.getMaximumResponse() );
  }

  responsesCenterStraight.clear();
  for(int i = 0; i < NUMBER_OF_POINTS_MOVING_AVERAGE; i++) {
    responsesCenterStraight.push_back( mHaarFeatureCenterStraightY.getMaximumResponse() );
  }

  responsesCrossroadStraight.clear();
  for(int i = 0; i < NUMBER_OF_POINTS_MOVING_AVERAGE; i++) {
    responsesCrossroadStraight.push_back( mHaarFeatureCrossroadLineY.getMaximumResponse() );
  }
}

bool HaarFilter::getInitialized() {
  return initialized;
}

void HaarFilter::extractStraightFeatures(int** highestResponses, float** gridResponses, int amountResponses, list<int>* lastResponses, FeatureType type, HaarFeature* mHaarFeatureStraightY, HaarFeature* mHaarFeatureStraightX, HaarFeature* mHaarFeatureStraightRotatedY, HaarFeature* mHaarFeatureStraightRotatedX) {
  float maximumResponse = (float)mHaarFeatureStraightY->getMaximumResponse();
  int cellX, cellY, gridKey;
  float probability, angle = 0;
  int gridColumns = ceil((float) mInputSize.width / GRID_X);

  for(int i = 0; i < amountResponses; i++) {
    if(highestResponses[i][FILTER_RESPONSE] > movingAverages[type]) {
      cellX = (highestResponses[i][POSITION_X] - maxOffsetX) / GRID_X;
      cellY = (highestResponses[i][POSITION_Y] - maxOffsetY) / GRID_Y;
      gridKey = cellX + cellY * gridColumns;

      probability = min( (float)1.0, (float)highestResponses[i][FILTER_RESPONSE] / maximumResponse );
      if(type == PARKING_LOT) {
        angle = ANGLE2;
      }
      else {
        angle = getAngle( (Orientations) highestResponses[i][FILTER_ORIENTATION], highestResponses[i][FILTER_RESPONSE], highestResponses[i][POSITION_X], highestResponses[i][POSITION_Y], mHaarFeatureStraightY, mHaarFeatureStraightX, mHaarFeatureStraightRotatedY, mHaarFeatureStraightRotatedX );
      }

      if(gridResponses != 0) {
        if(probability > gridResponses[gridKey][0]) {
          gridResponses[gridKey][0] = probability;
          gridResponses[gridKey][1] = type;
          gridResponses[gridKey][2] = highestResponses[i][POSITION_X];
          gridResponses[gridKey][3] = highestResponses[i][POSITION_Y];


          gridResponses[gridKey][4] = angle;

          if(maximumProbability < probability) {
            maximumProbability = probability;
          }
        }
      }
      else {
        Feature newFeature;

        newFeature.pose = mCoordConverter->pixel2Car( cv::Point2f( highestResponses[i][POSITION_X] - maxOffsetX, highestResponses[i][POSITION_Y] - maxOffsetY ) );
        newFeature.pose.setYaw( angle );
        newFeature.type = type;
        newFeature.probability = probability;
        newFeature.frameCounter = 0;

        mFeatureVector.push_back( newFeature );
      }
    }
  }

  if(lastResponses != 0) {
    lastResponses->push_front(highestResponses[0][FILTER_RESPONSE]);
    lastResponses->resize(NUMBER_OF_POINTS_MOVING_AVERAGE);
  }
}

float HaarFilter::getAngle(Orientations orientation, int response, int positionX, int positionY, HaarFeature* mHaarFeatureStraightY, HaarFeature* mHaarFeatureStraightX, HaarFeature* mHaarFeatureStraightRotatedY, HaarFeature* mHaarFeatureStraightRotatedX) {
  float angle = 0.0;
  HaarFeature *featureX = 0;
  HaarFeature *featureY = 0;

  float angleX = 0.0;
  float angleY = 0.0;

  switch(orientation) {
    case ORIENTATION_Y:
      angle = ANGLE2;
      angleX = (float) (ANGLE3 - M_PI);
      angleY = ANGLE4;
      featureX = mHaarFeatureStraightRotatedX;
      featureY = mHaarFeatureStraightRotatedY;
      break;
    case ORIENTATION_X:
      angle = ANGLE1;
      angleX = ANGLE3;
      angleY = ANGLE4;
      featureX = mHaarFeatureStraightRotatedX;
      featureY = mHaarFeatureStraightRotatedY;
      break;
    case ORIENTATION_Y_ROTATED:
      angle = ANGLE4;
      angleX = ANGLE1;
      angleY = ANGLE2;
      featureX = mHaarFeatureStraightX;
      featureY = mHaarFeatureStraightY;
      break;
    case ORIENTATION_X_ROTATED:
      angle = ANGLE3;
      angleX = ANGLE1;
      angleY = (float) (ANGLE2 + M_PI);
      featureX = mHaarFeatureStraightX;
      featureY = mHaarFeatureStraightY;
      break;
  }
  if (featureX != 0 && featureY != 0) {
    angle = interpolateAngle(response,
                             angle,
                             featureX->haarFeature(positionY, positionX),
                             angleX,
                             featureY->haarFeature(positionY, positionX),
                             angleY);
  }

  return angle;
}

void HaarFilter::saveHighestResponses(int** highestResponses, int amountResponses, int response, int positionX, int positionY, Orientations orientation) {
  if(response > highestResponses[0][FILTER_RESPONSE]) {
    int pos = amountResponses-1;

    for(int k = 0; k < amountResponses-1; k++) {
      if(response > highestResponses[k+1][FILTER_RESPONSE]) {
        highestResponses[k][FILTER_RESPONSE] = highestResponses[k+1][FILTER_RESPONSE];
        highestResponses[k][POSITION_X] = highestResponses[k+1][POSITION_X];
        highestResponses[k][POSITION_Y] = highestResponses[k+1][POSITION_Y];
        highestResponses[k][FILTER_ORIENTATION] = highestResponses[k+1][FILTER_ORIENTATION];
      } else {
        pos = k;
        break;
      }
    }
    if(pos > -1) {
      highestResponses[pos][FILTER_RESPONSE] = response;
      highestResponses[pos][POSITION_X] = positionX;
      highestResponses[pos][POSITION_Y] = positionY;
      highestResponses[pos][FILTER_ORIENTATION] = orientation;
    }
  }
}

FeatureVector* HaarFilter::getFeatures() {
  return &mFeatureVector;
}

cv::Mat HaarFilter::generateDebugImage() {
  cv::Mat output( mInputSize, CV_8UC4, cv::Scalar(0,0,0,0) );
ostringstream oss;
  for( size_t i = 0; i < mFeatureVector.size(); i++ )
  {
    Feature* f = &mFeatureVector[i];
    cv::Point2f pos = mCoordConverter->car2Pixel( f->pose );
    if(pos.x > 0 && pos.x < mInputSize.width && pos.y > 0 && pos.y < mInputSize.height) {
      cv::Point2f dist = cv::Point2f( -16.0*sin(f->pose.getYaw()), -16.0*cos( f->pose.getYaw() ) );
      oss.clear(); oss.str(""); oss << f->probability;
      cv::putText( output, oss.str(), pos, cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(0, 0, 255, 0));

      if(f->type == CENTER_LINE) {
        cv::line( output, pos, pos + dist, cv::Scalar( 255, 64, 32, 255 ), 1 );
        cv::circle( output, pos, 2, cv::Scalar( 255, 64, 32, 255 ) );
      }
      else if (f->type == CROSSROAD_LINE) {
        cv::line( output, pos, pos + dist, cv::Scalar( 0, 190, 0, 255 ), 1 );
        cv::circle( output, pos, 2, cv::Scalar( 0, 190, 0, 255 ) );
      }
      else if (f->type == CROSSROAD_CORNER) {
        cv::line( output, pos, pos + dist, cv::Scalar( 0, 130, 0, 255 ), 1 );
        cv::circle( output, pos, 2, cv::Scalar( 122, 0, 255, 255 ) );
      }
      else if (f->type == PARKING_LOT) {
        cv::line( output, pos, pos + dist, cv::Scalar( 0, 155, 255, 255 ), 1 );
        cv::circle( output, pos, 2, cv::Scalar( 0, 155, 255, 255 ) );
      }
      else if (f->type == PARKING_LINE) {
        cv::line( output, pos, pos + dist, cv::Scalar( 222, 255, 0, 255 ), 1 );
        cv::circle( output, pos, 2, cv::Scalar( 222, 255, 0, 255 ) );
      }
      else {
        cv::line( output, pos, pos + dist, cv::Scalar( 32, 64, 255, 255 ), 1 );
        cv::circle( output, pos, 2, cv::Scalar( 32, 64, 255, 255 ) );
      }
    }

    //output.at<cv::Vec3b>(pos) = cv::Vec3b( 255, 128, 0 );
  }


  cv::Vec4b colorLeft(183, 22, 247, 255);
  cv::Vec4b colorRight(108, 10, 147, 255);

  // draw calculated bird view cones
  for(int i = 0; i < mInputSize.height; i++ ) {
    int xConeLeft = ((float)i - bLeft) / mLeft;
    if(xConeLeft >= 0 && xConeLeft < mInputSize.width) {
      output.at<cv::Vec4b>(i, xConeLeft) = colorLeft;
    }
    int xConeRight = ((float)i - bRight) / mRight;
    if(xConeRight >= 0 && xConeRight < mInputSize.width) {
      output.at<cv::Vec4b>(i, xConeRight) = colorRight;
    }
  }

  return output;
}

float HaarFilter::interpolateAngle(int weight, float angle, int weightNeighbour1, float angleNeighbour1, int weightNeighbour2, float angleNeighbour2) {
  int factor = 3;

  weightNeighbour1 = max(0, weightNeighbour1 * factor);
  weightNeighbour2 = max(0, weightNeighbour2 * factor);

  return ( weight * angle +  weightNeighbour1 * angleNeighbour1 + weightNeighbour2 * angleNeighbour2 ) / ( weight + weightNeighbour1 + weightNeighbour2);
}

void HaarFilter::updateMaximumOffset(HaarFeature& haarFeature) {
  maxOffsetX = max(maxOffsetX, haarFeature.getWidth());
  maxOffsetY = max(maxOffsetY, haarFeature.getHeight());
}

void HaarFilter::updateMaximumParkingOffset(HaarFeature& haarFeature) {
  maxParkingOffsetX = max(maxParkingOffsetX, haarFeature.getWidth());
  maxParkingOffsetY = max(maxParkingOffsetY, haarFeature.getHeight());
}

cv::Mat HaarFilter::getROIs(cv::Mat& inputImage) {
  std::vector<float> array;
  array.assign(inputImage.datastart, inputImage.dataend);

  std::sort(array.begin(), array.end());
  
  int quant = (int) (array.size() * 0.95);

  float quantVal = array[quant];

  cv::Mat roiImage;

  cv::threshold(inputImage, roiImage, quantVal, 255 , cv::THRESH_BINARY);

  int erosion_size = 2;
  cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1),      cv::Point(erosion_size, erosion_size) );
  cv::erode( roiImage, roiImage, element );

  return roiImage;
}

}	// namespace
}	// namespace


