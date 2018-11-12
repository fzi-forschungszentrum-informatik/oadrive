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

#ifndef OADRIVE_LANEDETECTION_ROADPERCEPTION_H
#define OADRIVE_LANEDETECTION_ROADPERCEPTION_H

#include <opencv2/imgproc/imgproc.hpp>
#include <icl_core_logging/LoggingManager.h>

#include <oadrive_util/CoordinateConverter.h>
#include <oadrive_core/ExtendedPose2d.h>
#include <oadrive_world/TrafficSign.h>

#include "oadrive_lanedetection/FeatureDetection/StreetTypes.h"
#include "oadrive_lanedetection/FeatureDetection/HaarFilter.h"
#include "oadrive_lanedetection/DebugViewRoadPerception.h"
#include "StreetPatcher.h"
#include "IntersectionPatcher.h"
#include "MapPatcher.h"
#include "PatchHypothesis.h"
#include "FeatureClassification.h"
#include "RoadHypothesis.h"


namespace oadrive
{
namespace lanedetection
{

class RoadPerception
{
  INIT_MEASURE_EXEC_TIME

#ifdef DEBUG_ACTIVE
  friend class DebugViewRoadPerception;
#endif

public:

  /*!
   * Create a new road perception object from the given data.
   * @param birdViewSize the size of the birdview in pixel.
   * @param coordConverter for transformations between world, car and image.
   * @param searchForCrossSections determines if the road perception searches for intersections.
   */
  RoadPerception(cv::Size birdViewSize, util::CoordinateConverter* coordConverter,
                 bool searchForIntersection);

  /*!
   * Updates the car pose to the given new one.
   * @param carPose the new car pose.
   */
  void updateCarPose(const OadrivePose &carPose);

  /*!
   * Executes the feature detection and classification on the birdview image. The result are
   * feature (hopefully) located on the street lines which are classified in left, center and
   * right line. The features are saved internally.
   * @param birdviewImage
   */
  void executeFeatureDetection(const cv::Mat &birdviewImage);


  /*!
   * Executes the road perception, should be called after executeFeatureDetection because it uses
   * the features detected by this method. It calculates patches by searching for maxima in the
   * voting space. The features vote for patch locations in that voting space. In the second step
   * the method searches for intersection patches, using also the segmentation of the birdview
   * image and traffic signs if available.
   * @param neuralNetOutput the segmentation of the birdview image currently calculated by a FCN.
   * @param trafficSign optional traffic sign.
   */
  void executeRoadPerception(const cv::Mat &neuralNetOutput, const world::TrafficSign* trafficSign);

  /*!
   * Executes the road perception, should be called after executeFeatureDetection because it uses
   * the features detected by this method. It calculates patches by searching for maxima in the
   * voting space. The features vote for patch locations in that voting space. In the second step
   * the method searches for intersection patches, using the available map data
   * @param worldPose Position of the car in the world
   */
  void executeRoadPerception(const OadrivePose worldPose);

  /*!
   * Configueres the road perception to detect / not detect cross sections.
   * @param searchForCrossSections for enable/disable the search for cross sections.
   */
  void configureRoadPerception(bool searchForCrossSections);

  /*!
   * Set a pose where the first patch should approximatly be.
   * @param pose the estimated pose in car coordinates of the first patch.
   */
  void setEstimatedFirstPatchPose(const core::ExtendedPose2d &pose);

  /*!
   * Returns the current road consisting of patches (street and cross sections).
   * @return
   */
  const PatchHypothesisList & getRoad();

  void enableCrossSectionDetection();

  void disableCrossSectionDetection();

  /*!
   * Adds the next driving direction for intersections (left, straight, right).
   * Given this the road perception has to search only in this direction for new patches.
   * @param driveDir the next driving direction (left, straight, right)
   */
  void addNextDrivingDirection(PatchHypothesis::DrivingDirection driveDir);

  /*!
   * Resets the road perception to its initial state. Needs a first guess where the road should be.
   * @param estimatedPatchPose the estimated pose in car coordinates of the first patch.
   */
  void reset(const OadrivePose &estimatedPatchPose);

  void setMergeMode(bool mergeModeActive);

  void loadMap(std::string map);

  // Debug
  cv::Mat generateDebugImage();

  cv::Mat* generateStreetVoteDebugImage();

private:

  void processFeatures(FeatureVector* features);

  void findRoadPatches();

  void findIntersections(const world::TrafficSign* trafficSign);

  void findIntersections(const OadrivePose worldPose);

  /*!
   * The position of the traffic sign is given in the synchronized coordinate system.
   * The RoadPerception works with the drifting. So the traffic sign position has
   * to be transformed from the synchronized to the drifting.
   * @param trafficSign whose position is transformed.
   */
  world::TrafficSign transformTrafficSign(const world::TrafficSign &trafficSign);

  /*!
   * Iterates through the found features looking for crossroad features. Criterion is the
   * relative angle to the current patches. (Should be around +/- Pi/2)
   * @param features
   * @return a vector containing the found crossroad features.
   */
  FeaturePointerVector searchForCrossRoadFeatures(FeaturePointerVector features);

  /*!
   * Transform the features in the coordinate system given by refPose
   * @param features which should be transformed.
   * @param refPose pose specifying the coordinate system.
   */
  void transformFeatures(FeaturePointerVector features, OadrivePose refPose);

/******************************************************************************/
/**************************** class attributes ********************************/
/******************************************************************************/

  // Current birdview image used for feature detection.
  cv::Mat mCurrentImage;

  // The haar filter which is responsible for feature detection.
  HaarFilter mHaarFilter;

  // The coordinate converter used for transformation between world, car and image.
  util::CoordinateConverter* mCoordConverter;

  // The street patcher which is responsible for detecting street patches on the basis of features.
  StreetPatcher mStreetPatcher;

  // The intersection patcher which is responsible for detecting intersection patches on the basis
  // of patches, birdview segmentation and traffic signs.
  IntersectionPatcher mIntersectionPatcher;

  // The map patcher which is responsible for detecting intersection patches based on the map
  MapPatcher mMapPatcher;

  // Feature classficator which classifies the features in left, center and right.
  FeatureClassification mFeatureClassificator;

  // The current hypothesis of the road consisting of patch hypotheses.
  RoadHypothesis mRoadHyp;

  // The features of the current image.
  FeaturePointerVector mCurrentFeatures;

  world::TrafficSign* mCurrentTrafficSign = nullptr;
  bool mTrafficSignValid;

  // Current and previous pose of the car in global coordinates
  OadrivePose mCurrentCarPose;
  OadrivePose mPrevCarPose;

  // Determines if the road perception searches for crossings
  bool mSearchForIntersections;

  // Determines if the features are classified based on their location relative to patches.
  bool mEnableClassification;

  // Determines if the patch positions are adjusted to correct errors in the odometry position.
  bool mEnablePositionAdjustment;

  // Used for the detection of the first call to set a priori vote to make the detection of the
  // first patches easier.
  bool mFirstCall;

  // if true the road perception is in merging mode
  bool mMergeMode;

  icl_core::logging::LoggingManager &loggingManager = icl_core::logging::LoggingManager::instance();

#ifdef DEBUG_ACTIVE
  cv::Mat dbgFeatureImage;
#endif

public:
  // use a proper alignment when calling the constructor.
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW


};

}
}
#endif //OADRIVE_LANEDETECTION_ROADPERCEPTION_H
