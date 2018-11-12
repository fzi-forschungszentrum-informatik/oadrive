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
 * \date    2015-11-01
 *
 */
//----------------------------------------------------------------------

#ifndef HAARFILTER_H_
#define HAARFILTER_H_

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <oadrive_util/CoordinateConverter.h>
#include <iostream>
#include "StreetTypes.h"
#include "HaarFeature.h"
#include "IntegralImage.h"
#include "oadrive_lanedetection/DebugViewRoadPerception.h"
#include <chrono>


namespace oadrive{

namespace lanedetection{

// possible orientations of lines
enum Orientations {
  ORIENTATION_Y,
  ORIENTATION_Y_ROTATED,
  ORIENTATION_X,
  ORIENTATION_X_ROTATED
};

/* Maximum features are defined for a bird view which has approximately a view 1.2m to the front.
 * If we look more into front, we also need to increase the amount of side line, center line
 * and parking spot features.
 */
#define AMOUNT_RESPONSES_VISUAL_RANGE 1.2
// maximum features of side straight features
#define AMOUNT_RESPONSES_SIDE 200
// maximum features of center straight features (should be around 2/3 of AMOUNT_RESPONSES_SIDE)
#define AMOUNT_RESPONSES_CENTER 125
// maximum amount of features of cross road line features
#define AMOUNT_RESPONSES_CROSSROAD 55
// maximum number of parking line features
#define AMOUNT_RESPONSES_PARKING_LINE 18

// minimum amount of features that must be detected that filter is set to initialized
#define AMOUNT_RESPONSES_INITIALISED ( ( AMOUNT_RESPONSES_CENTER + AMOUNT_RESPONSES_SIDE ) / 2 )

// defines how big the history of old responses is for moving average filter (higher values make the feature detection adapt slower to light change but it is more robust against other influences)
#define NUMBER_OF_POINTS_MOVING_AVERAGE 1

// define possible angle
#define ANGLE1 M_PI_2
#define ANGLE2 0
#define ANGLE3 M_PI_2 + M_PI_4
#define ANGLE4 M_PI_4

// define position of values in vector
#define FILTER_RESPONSE 0
#define POSITION_X 1
#define POSITION_Y 2
#define FILTER_ORIENTATION 3
#define FEATURE_TYPE 4

// size of grid for grid clustering
#define GRID_X 40
#define GRID_Y 30

// step size for feature calculation
#define STEPSIZE_X 3
#define STEPSIZE_Y 30

// step size for feature calculation of parking detection
#define STEPSIZE_PARKING_X 3
#define STEPSIZE_PARKING_Y 3

// detect parking lines only on every PARKING_LINE_X_SKIP*STEPSIZE_PARKING_X-th point in x direction
#define PARKING_LINE_X_SKIP 3

// defines offset to left and right border of image for parking detection (parking spots are normally just detected while driving by)
#define PARKING_MIN_X_OFFSET_LEFT 320
#define PARKING_MIN_X_OFFSET_RIGHT 100

// detect crossroad only in image
#define DETECT_CROSSROAD_Y_FROM_TOP 140

class HaarFilter {

friend class DebugViewRoadPerception;

public:
  //! constructor of haar filter initializes all features according to the provided meters per pixel value
  HaarFilter( util::CoordinateConverter* converter );

  //! destructor does nothing
  ~HaarFilter();

  //! setImage always has to be called before getting features
  void setImage( cv::Mat& image );

  /* calculate features in every few lines
   * \param detectCrossroad toggle cross road line detection
   * \param detectParkingLot toggle parking lot detection (T feature and line)
   */
  void calculateFeaturesPerLine(bool detectCrossroad = false, bool detectParkingLot = false);

  //! get features calculated by one of the feature calculation functions
  FeatureVector* getFeatures();

  //! returns true if filter is initialized, it can take some frames until the filter is initialized
  bool getInitialized();

  //! reset initialization of filter (it takes some frames until filter is initialized again)
  void resetInitialization();

  //! draws line features into given image
  cv::Mat generateDebugImage();

private:
  //! bird view image
  cv::Mat mImage;

  //! binary image with intersting regions
  cv::Mat mROIImage;

  //! integral image of bird view image
  cv::Mat mIntegralImage;

  //! squared integral image, not used (comes for free)
  cv::Mat mSquaredIntegralImage;

  //! integral image rotated by 45 degress
  cv::Mat mRotatedIntegralImage;

  //! object of integral image class (used to abstract feature evaluation)
  IntegralImage mIntegralImageObject;

  //! object of rotated integral image class (used to abstract feature evaluation)
  IntegralImageRotated mIntegralImageRotatedObject;

  //! declaration of out straight haar feature objects
  HaarFeatureStraight mHaarFeatureSideStraightY;
  HaarFeatureStraight mHaarFeatureSideStraightRotatedY;
  HaarFeatureStraight mHaarFeatureSideStraightX;
  HaarFeatureStraight mHaarFeatureSideStraightRotatedX;

  //! list of last responses of the straight haar feature objects, used to calculate the moving average
  std::list<int> responsesSideStraight;

  //! declaration of center straight haar feature objects
  HaarFeatureStraight mHaarFeatureCenterStraightY;
  HaarFeatureStraight mHaarFeatureCenterStraightRotatedY;
  HaarFeatureStraight mHaarFeatureCenterStraightX;
  HaarFeatureStraight mHaarFeatureCenterStraightRotatedX;

  //! list of last responses of the center straight haar feature objects, used to calculate the moving average
  std::list<int> responsesCenterStraight;

  //! declaration of crossroad line feature objects
  HaarFeatureStraight mHaarFeatureCrossroadLineY;
  HaarFeatureStraight mHaarFeatureCrossroadLineRotatedY;
  HaarFeatureStraight mHaarFeatureCrossroadLineX;
  HaarFeatureStraight mHaarFeatureCrossroadLineRotatedX;

  HaarFeatureStraight mHaarFeatureCrossroadLineWashedOutRotatedY;
  HaarFeatureStraight mHaarFeatureCrossroadLineWashedOutX;
  HaarFeatureStraight mHaarFeatureCrossroadLineWashedOutRotatedX;

  //! list of last responses of the crossroad line haar feature objects, used to calculate the moving average
  std::list<int> responsesCrossroadStraight;

  //! declaration of the parking line feature object
  HaarFeatureStraight mHaarFeatureParkingLineX;

  //! stores current minimum response calculated with moving average filter
  int overallMovingAverage;

  //! sotres current minimum reponse scaled for several feature types
  int movingAverages[numFeatureTypes];

  //! vector which stores all detected features
  FeatureVector mFeatureVector;

  //! image size
  cv::Size mInputSize;

  //! contain the pixels per meter for the current bird view config
  float pixelsPerMeter;

  //! pointer to bird view position converter, used to get pixels per meter value and transformation to world coordinates
  util::CoordinateConverter* mCoordConverter;

  // save maximum filter size as offset
  int maxOffsetX;
  int maxOffsetY;

  // save maximum filter size of parking features as offset
  int maxParkingOffsetX;
  int maxParkingOffsetY;

  // variables for detection if out of birdview cone
  float mLeft;
  float bLeft;
  float mRight;
  float bRight;

  // maximum probability of feature (used for normalization)
  float maximumProbability;

  // factors to compare responses of two different feature types
  float factorSide2Center;
  float factorParkingLine2Center;
  float factorCrossroad2Center;
  float factorCrossroadWashedOut2Center;
  float factorCrossroadCorner2Center;
  float factorCrossroad2Side;
  float factorCrossroadWashedOut2Crossroad;

  // flag if filter is initialized
  bool initialized;

  //! set up all features and variables
  void startup();

  //! approximate angle of feature by using different feature responses (uses interpolateAngle function)
  float getAngle(Orientations orientation, int response, int positionX, int positionY, HaarFeature* mHaarFeatureStraightY, HaarFeature* mHaarFeatureStraightX, HaarFeature* mHaarFeatureStraightRotatedY, HaarFeature* mHaarFeatureStraightRotatedX);

  //! approximate angle of feature by using different feature responses
  float interpolateAngle(int weight, float angle, int weightNeighbour1, float angleNeighbour1, int weightNeighbour2, float angleNeighbour2);

  //! saves response if big enough in ordered (by value) vector of size amountResponses
  void saveHighestResponses(int highestResponses[][4], int amountResponses, int response,
                            int positionX, int positionY, Orientations orientation);

  //! function to filter features in grids, only highest feature in grid is used if gridResponses pointer given, otherwise features are directly put into detected feature list
  void extractStraightFeatures(int highestResponses[][4], float** gridResponses,
                               int  amountResponses, std::list<int>* lastResponses,
                               FeatureType type, HaarFeature* mHaarFeatureStraightY,
                               HaarFeature* mHaarFeatureStraightX,
                               HaarFeature* mHaarFeatureStraightRotatedY,
                               HaarFeature* mHaarFeatureStraightRotatedX);

  //! function to recalculate the moving average according to the last feature responses
  void updateMovingAverage(std::list<int>& lastResponses, FeatureType type);

  //! calculates the maximum offset/border that needs to be kept to the image border
  void updateMaximumOffset(HaarFeature& haarFeature);

  //! calculates the maximum offset/border that needs to be kept to the image border for parking detection
  void updateMaximumParkingOffset(HaarFeature& haarFeature);

  //! calculate region of interest (= bright spots) on input image
  cv::Mat getROIs(cv::Mat& inputImage);

};

}	// namespace
}	// namespace

#endif
