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

#include <chrono>

#include <boost/make_shared.hpp>

#include "RoadPerception.h"
#include "oadrive_lanedetection/lanedetectionLogging.h"
#include "oadrive_util/Config.h"

using oadrive::core::ExtendedPose2d;
using oadrive::util::CoordinateConverter;
using oadrive::util::Config;
using icl_core::logging::endl;
using icl_core::logging::flush;

using namespace std::chrono;

namespace oadrive
{
namespace lanedetection
{


RoadPerception::RoadPerception(cv::Size birdViewSize, CoordinateConverter* coordConverter,
                               bool searchForCrossSections) :
        mHaarFilter(coordConverter),
        mCoordConverter(coordConverter),
        mStreetPatcher(birdViewSize, coordConverter),
        mIntersectionPatcher(birdViewSize, coordConverter),
        mMapPatcher(birdViewSize, coordConverter, Config::getString("Map", "Path", "")),
        mFeatureClassificator(birdViewSize, coordConverter),
        mRoadHyp(coordConverter),
        mTrafficSignValid(false),
        mCurrentCarPose(0.f, 0.f, 0.f),
        mSearchForIntersections(searchForCrossSections),
        mEnableClassification(true),
        mEnablePositionAdjustment(false),
        mFirstCall(true),
        mMergeMode(false)
{
  loggingManager.initialize();
  loggingManager.setLogLevel(icl_core::logging::LogLevel::eLL_WARNING);
  lanedetectionLogger::create();

#ifdef DEBUG_ACTIVE
  DebugViewRoadPerception::setRoadPerception(this);
#endif
}

/******************************************************************************/
/***************************** public methods *********************************/
/******************************************************************************/


void RoadPerception::updateCarPose(const OadrivePose &carPose)
{
  if (mFirstCall)
  {
    mPrevCarPose = carPose;
    mCurrentCarPose = carPose;
    mFirstCall = false;
  }

  mPrevCarPose = mCurrentCarPose;
  mCurrentCarPose = carPose;
}


void
RoadPerception::executeFeatureDetection(const cv::Mat &birdviewImage)
{
  mCurrentFeatures.clear();
  mCurrentImage = birdviewImage;


  /******************* feature detection ***********************/
  START_CLOCK
  mHaarFilter.setImage(mCurrentImage);
  mHaarFilter.calculateFeaturesPerLine(true, false);
  FeatureVector* features = mHaarFilter.getFeatures();
  END_CLOCK("Feature Detection")


  /******************* feature classification ***********************/
  START_CLOCK
  processFeatures(features);
  END_CLOCK("Feature Classification")
}

void RoadPerception::executeRoadPerception(const cv::Mat &neuralNetOutput,
                                           const world::TrafficSign* trafficSign)
{
  /******************* Patch detection ***********************/
  START_CLOCK
  findRoadPatches();
  END_CLOCK("Find Patches")


  /******************* Intersection detection ***********************/
  if (mSearchForIntersections)
  {
    START_CLOCK
    // findIntersections(neuralNetOutput, trafficSign);
    END_CLOCK("Find Intersections")
  }
}

void RoadPerception::executeRoadPerception(const OadrivePose worldPose)
{
  /******************* Patch detection ***********************/
  START_CLOCK
  findRoadPatches();
  END_CLOCK("Find Patches")


  /******************* Intersection detection ***********************/
  if (mSearchForIntersections)
  {
    START_CLOCK
    // findIntersections(nullptr);
    findIntersections(worldPose);
    END_CLOCK("Find Intersections")
  }
}

void RoadPerception::configureRoadPerception(bool searchForIntersections)
{
  mSearchForIntersections = searchForIntersections;
}

const PatchHypothesisList &RoadPerception::getRoad()
{
  return mRoadHyp.getRoadHypothesis();
}

void RoadPerception::enableCrossSectionDetection()
{
  mSearchForIntersections = true;
}

void RoadPerception::disableCrossSectionDetection()
{
  mSearchForIntersections = false;
}

void RoadPerception::setEstimatedFirstPatchPose(const ExtendedPose2d &pose)
{
  OadrivePose estimatedInitalPatchPose = OadrivePose(pose.getX() - PATCH_LENGTH_STRAIGHT,
                                                     pose.getY(),
                                                     pose.getYaw());
  mRoadHyp.setInitialPatchPose(estimatedInitalPatchPose);
}


void RoadPerception::addNextDrivingDirection(PatchHypothesis::DrivingDirection driveDir)
{
  mRoadHyp.addNextDrivingDirection(driveDir);
}

void RoadPerception::reset(const OadrivePose &estimatedPatchPose)
{
  mFirstCall = true;
  mTrafficSignValid = false;
  mRoadHyp.reset(estimatedPatchPose);
}

/******************************************************************************/
/***************************** private methods ********************************/
/******************************************************************************/

void RoadPerception::processFeatures(FeatureVector* features)
{
  for (size_t i = 0; i < features->size(); i++)
  {
    if (!mFeatureClassificator.insideInvalidMask(features->at(i)))
    {
      mCurrentFeatures.push_back(&features->at(i));
    }
  }

#ifdef DEBUG_ACTIVE
  dbgFeatureImage = DebugViewRoadPerception::generateFeatureDebugImage(mCurrentFeatures);
#endif


  if (mEnableClassification)
  {
    mFeatureClassificator.classifyFeatures(mCurrentFeatures, mRoadHyp.getCurrentRefHyp(), mRoadHyp.getRoadHypothesis());

    auto nextIntersection = mRoadHyp.getCurrentIntersection();
    if (nextIntersection) {
      mFeatureClassificator.classifyFeatures(mCurrentFeatures, *nextIntersection, mRoadHyp.getSortedRoadHypothesis(), false);
      // Without reset to not destroy prev. classifications, and also sorted is required, to prevent prev/next problems
    }
  }

#ifdef DEBUG_ACTIVE
  dbgFeatureImage = DebugViewRoadPerception::generateFeatureDebugImage(mCurrentFeatures);
#endif

  if (mEnablePositionAdjustment)
  {
    mFeatureClassificator.adjustPatchPositions(mCurrentFeatures,
                                               mRoadHyp.getCurrentRefHyp(),
                                               mRoadHyp.getRoadHypothesis());
  }
}


void RoadPerception::findRoadPatches()
{
  mRoadHyp.updatePatches(mCurrentCarPose, mPrevCarPose);

  mStreetPatcher.update(mRoadHyp.getCurrentRefHyp(), mRoadHyp.getAngleHistory());
  PatchHypothesisList foundPatches =
          mStreetPatcher.generatePatches(mCurrentFeatures,
                                         mRoadHyp.getRoadHypothesis());

  mRoadHyp.addPatches(foundPatches);
}

void RoadPerception::findIntersections(const OadrivePose worldPose)
{

  PatchHypothesisList foundIntersections =
          mMapPatcher.generateIntersectionPatches(worldPose, mRoadHyp.getRoadHypothesis());

  /* Add the found intersections to the current road hypthesis. */
  if (foundIntersections.size() > 0)
  {
    PatchHypothesis intersection = foundIntersections.front();

    mRoadHyp.addIntersection(intersection, PatchHypothesisList());
  }
}

void RoadPerception::findIntersections(const world::TrafficSign* trafficSign = nullptr)
{
  /* Search for cross features */
  FeaturePointerVector crossRoadFeatures = searchForCrossRoadFeatures(mCurrentFeatures);

  /* Search for cross patches based on the cross features and neural net ouput */
  PatchHypothesisList foundCrossPatches =
          mStreetPatcher.generateCrossPatches(crossRoadFeatures,
                                              mRoadHyp.getRoadHypothesis());

  #ifdef DEBUG_ACTIVE
    DebugViewRoadPerception::visualizePatches(foundCrossPatches);
    //DebugViewRoadPerception::visualizePatches(mRoadHyp.getRoadHypothesis());
  #endif

  /* Search for intersections based on the cross patches and, if availabe, traffic signs */
  const world::TrafficSign* currentTrafficSign;
  if (trafficSign)
  {
    if (!mCurrentTrafficSign)
    {
      // TODO: check for memory leak.
      mCurrentTrafficSign = new world::TrafficSign(transformTrafficSign(*trafficSign));
    }
    else
    {
      *mCurrentTrafficSign = transformTrafficSign(*trafficSign);
    }
    currentTrafficSign = mCurrentTrafficSign;
  }
  else
  {
    currentTrafficSign = trafficSign;
  }

#ifdef DEBUG_ACTIVE
  dbgFeatureImage = DebugViewRoadPerception::generateFeatureDebugImage(crossRoadFeatures);
#endif

  PatchHypothesisList foundIntersections =
          mIntersectionPatcher.generateIntersectionPatches(mCurrentCarPose,
                                                           mRoadHyp.getRoadHypothesis(),
                                                           foundCrossPatches,
                                                           currentTrafficSign);

  /* Add the found intersections to the current road hypthesis. */
  if (foundIntersections.size() > 0)
  {
    PatchHypothesis intersection = foundIntersections.front();

    // if the found intersection is based on the traffic sign set the yaw of the intersection to
    // the yaw of the traffic sign
    if (trafficSign && intersection.getBasedOn() == PatchHypothesis::BasedOn::TRAFFIC_SIGN)
    {
      intersection.setYaw(mCurrentTrafficSign->getYaw());
    }

    mRoadHyp.addIntersection(intersection, foundCrossPatches);
  }
}

FeaturePointerVector RoadPerception::searchForCrossRoadFeatures(FeaturePointerVector features)
{
  float range = 0.4;
  FeaturePointerVector crossRoadFeatures;
  for (const PatchHypothesis patchHyp : mRoadHyp.getRoadHypothesis())
  {
    if (patchHyp.getPatchType() == PatchHypothesis::Type::INTERSECTION)
    {
      break;
    }

    // transform features in the coordinate system of the current patch
    transformFeatures(features, patchHyp.getPose());

    for (auto featureIt = features.begin(); featureIt != features.end();)
    {
      // if the feature is to far away no reliable prediction based on the local angle can be made
      // because the direction of the road can change between the patch and the feature.
      if ((*featureIt)->poseRelPatch.getX() > range)
      {
        featureIt++;
      }
      else
      {
        // if the feature is near enough and the absolut angle is between 45°-135° or 225°-315°
        // the feature is considered as cross feature
        float localYawAbs = std::abs((*featureIt)->poseRelPatch.getYaw());
        if ((localYawAbs > M_PI_4 && localYawAbs < 3 * M_PI_4) || (localYawAbs > 5 * M_PI_4 &&
                                                                   localYawAbs < 7.f * M_PI_4))
        {
          crossRoadFeatures.push_back((*featureIt));
        }

        featureIt = features.erase(featureIt);
      }
    }
  }

  return crossRoadFeatures;
}

void RoadPerception::transformFeatures(FeaturePointerVector features, OadrivePose refPose)
{
  for (uint i = 0; i < features.size(); i++)
  {
    features.at(i)->poseRelPatch = mCoordConverter->world2Car(refPose, features.at(i)->localPose);
  }
}

world::TrafficSign RoadPerception::transformTrafficSign(const world::TrafficSign &trafficSign)
{
  ExtendedPose2d carPose(mCurrentCarPose.getX(), mCurrentCarPose.getY(),
                                 mCurrentCarPose.getYaw());
  ExtendedPose2d newTrafficSignPose = mCoordConverter->world2Car(carPose,
                                                                 trafficSign.getPose());

  return world::TrafficSign(trafficSign.getType(), newTrafficSignPose);
}

/******************************************************************************/
/****************************** debug methods *********************************/
/******************************************************************************/

cv::Mat RoadPerception::generateDebugImage()
{
  // Get image of the detected features:
  cv::Mat image = DebugViewRoadPerception::generateFeatureDebugImage(mCurrentFeatures);
  // cv::Mat image(mCurrentImage.size(), CV_8UC4, cv::Scalar(0, 0, 0, 0));

  PatchHypothesisList::const_reverse_iterator it;
  for (it = mRoadHyp.getRoadHypothesis().rbegin(); it != mRoadHyp.getRoadHypothesis().rend(); it++)
  {
    cv::Point2f center = mCoordConverter->car2Pixel((*it).getPose());
    cv::Size2f size = (*it).getPatchSize();
    double angle = (*it).getYaw();
    cv::RotatedRect rRect(center, size, -angle * 180.0 / M_PI);

    cv::Point2f dist = cv::Point2f(-50.0 * sin(angle), -50.0 * cos(angle));

    util::PointList vertices = mCoordConverter->scaleRectangle(rRect, 1.0).vertices;

    cv::Scalar col(0, 0, 255);
    for (uint i = 0; i < vertices.size(); i++)
    {
      cv::line(image, vertices[i], vertices[(i + 1) % vertices.size()], col, 1);
    }
    cv::line(image, center, center + dist, col, 1);
    cv::circle(image, center, 4, col, 2);

    cv::putText(image, std::to_string((*it).getPatchID()), center, cv::FONT_HERSHEY_SIMPLEX,
                0.7, cv::Scalar(255));

  }

  if (mCurrentTrafficSign)
  {
    cv::Point2f center = mCoordConverter->car2Pixel(mCurrentTrafficSign->getPose());

    cv::Scalar col(147, 20, 255);
    cv::circle(image, center, 5, col, 2);
  }

  return image;
}

cv::Mat* RoadPerception::generateStreetVoteDebugImage()
{
  return nullptr;
}

void RoadPerception::setMergeMode(bool mergeModeActive) {
  mMergeMode = mergeModeActive;
  mFeatureClassificator.setMergeMode(mergeModeActive);
}

void RoadPerception::loadMap(std::string map) {
  mMapPatcher.loadMap(map);
}



}
}
