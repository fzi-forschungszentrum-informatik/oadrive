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
 * \author  Fabian Dürr
 * \date    2017-9-26
 * 
 * \author  Simon Roesler <simon.roesler@student.kit.edu>
 * \date    2018
 * 
 * \author  Mark Hueneberg <hueneber@fzi.de>
 * \date    2018
 */
//----------------------------------------------------------------------

#ifndef OADRIVE_LANEDETECTION_FEATURECLASSIFICATION_H
#define OADRIVE_LANEDETECTION_FEATURECLASSIFICATION_H

#include "oadrive_lanedetection/FeatureDetection/StreetTypes.h"
#include "StreetPatcher.h"

namespace oadrive
{
namespace lanedetection
{

class FeatureClassification
{

public:

  /*!
   * Creates a new feature classifier, given the birdview size and a coordinate converter.
   * @param birdviewSize the size of the birdview image in pixel.
   * @param coordConverter for transformations between world, car and birdview.
   */
  FeatureClassification(cv::Size birdviewSize, util::CoordinateConverter* coordConverter);

  /*!
  * Classifies the given features in right, center and left line based on the relative location
  * to the patch lying behind of the features.
  * @param features which should be classified.
  */
  void classifyFeatures(FeaturePointerVector features,
                        PatchHypothesis currentRefHypothesis,
                        PatchHypothesisList currentPatchHypotheses,
                        bool reset = true);

  /*!
   * Adjusts the lateral position of the patches by estimating the location of the center, left and
   * right line determined by the features in featurePointer.
   * @param featurePointer containing the features which are used to estimate the center, left
   * and right line.
   * @param currentRefPatchHyp the current reference patch hypothesis.
   * @param currentPatchHyps the patches currently in the birdview.
   */
  void adjustPatchPositions(const FeaturePointerVector &featurePointer,
                            PatchHypothesis currentRefPatchHyp,
                            PatchHypothesisList currentPatchHyps);


  /*!
 * Check if the feature lies on the engine cover or very close to the cone border.
 * @param feature which is checked;
 */
  bool insideInvalidMask(const Feature &feature);

  void setMergeMode(bool mergeModeActive);

private:

  /*!
   * Transforms the position of the features into the local coordinate system of the patch given by
   * its pose.
   * @param featurePointer the features which will be transformed.
   * @param refPose specifying the coordinate system of the patch.
   */
  void transformFeatures(FeaturePointerVector &featurePointer,
                         OadrivePose refPose, bool reset);

  /*!
   * Classifies all features whose distance is smaller than range in left, right or center line.
   * The classification is based on the feature's local y-location (referring to the patch lying
   * behind them).
   * @param featurePointer containing the features.
   * @param range in x-direction. All features closer than range will be classified.
   * @return a vector containing the classified features.
   */
  FeaturePointerVector classifyNearestFeatures(FeaturePointerVector &featurePointer, float range);

  /*!
   * Calculates the optimal y-offset for the patch, to match the patch's position with the
   * positions of the street lines determined by the left, center and right line.
   * @param leftLine features forming the left line.
   * @param centerLine features forming the center line.
   * @param rightLine features forming the right line.
   * @return the calculated offset.
   */
  float calculatePositionOffset(FeaturePointerVector &leftLine,
                                FeaturePointerVector &centerLine,
                                FeaturePointerVector &rightLine) const;

  /*!
   * Assuming the features in featurePointer form a street line, calculate the mean of
   * the y location of that line by calculating the mean of the y locations of the
   * features.
   * @param featurePointer containing the features.
   * @param intervalSize only features with its y location in that interval are considered.
   * @return the calculated mean.
   */
  float calculateFeatureYMean(FeaturePointerVector &featurePointer,
                              float intervalSize) const;

  /*!
   * Assuming the features in featurePointer form a street line, calculate the standard deviation of
   * the y location of that line by calculating the standard deviation of the y locations of the
   * features.
   * @param featurePointer containing the features.
   * @param mean the mean of the features' y location.
   * @return the calculated standard deviation.
   */
  float calculateFeatureYStdDev(const FeaturePointerVector &featurePointer, float mean) const;

  /*!
   * Calculates the y-mean and y-standard deviation of the features in line. If the standard
   * deviation is smaller a given threshold mean and standard deviation are added to the respective
   * list.
   * @param line containing the features forming a street line (left, center, right).
   * @param validMeans if mean is valid (this means stdDev < threshold) it's added to this list.
   * @param validStdDevs if stdDev is valid (this means stdDev < threshold) it's added to this list.
   * @param sizes the number of valid features.
   * @return if the calculated mean and stdDev have been valid and were added to the list.
   */
  bool addMeanAndStdDev(FeaturePointerVector &line, std::vector<float> &validMeans,
                        std::vector<float> &validStdDevs, std::vector<uint> &sizes) const;


/******************************************************************************/
/**************************** class attributes ********************************/
/******************************************************************************/

  util::CoordinateConverter* mCoordConverter;

  StreetPatcher mClassificationPatcher;
  
  bool mMergeMode;

};

}
}


#endif //OADRIVE_LANEDETECTION_FEATURECLASSIFICATION_H
