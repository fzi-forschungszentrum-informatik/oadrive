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
 * \author  Micha Pfeiffer <ueczz@student.kit.edu>
 * \date    2015-11-01
 * 
 * \author  Simon Roesler <simon.roesler@student.kit.edu>
 * \date    2018
 * 
 * \author  Mark Hueneberg <hueneber@fzi.de>
 * \date    2018
 *
 * Street-Detection.
 * - Uses the HaarFilter's responses to vote for Street Patches.
 *   These patches are then added to the Environment.
 *
 */
//----------------------------------------------------------------------

#include "StreetPatcher.h"

#include "opencv2/opencv.hpp"
#include "FeatureClassification.h"
#include <chrono>

#include <oadrive_lanedetection/lanedetectionLogging.h>
#include <oadrive_util/Config.h>
#include <boost/make_shared.hpp>

using oadrive::util::ImageHelper;
using icl_core::logging::endl;
using icl_core::logging::flush;

namespace oadrive
{

namespace lanedetection
{

StreetPatcher::StreetPatcher(cv::Size2i birdviewSize, util::CoordinateConverter* coordConverter) :
        RoadPatcher(birdviewSize, coordConverter, 10, 5),
        mVotingSpace(coordConverter, OadrivePose(0.f, 0.f, 0.f), birdviewSize),
        mNumOfPatches(3),
        mNNetOutputProcessed(false)
{
  LOGGING_INFO(lanedetectionLogger, "Generating Streetpatcher: "
          << "\tMinimum Vote: " << mMinimumVote << endl);


  initFeatureVotes();
  initPatchVotes();
  initCrossPatchVotes();

#ifdef DEBUG_ACTIVE
  dbgVisualizationEnabled = true;
#endif
}

StreetPatcher::~StreetPatcher()
{
}

/******************************************************************************/
/***************************** public methods *********************************/
/******************************************************************************/

PatchHypothesisList
StreetPatcher::generatePatches(const FeaturePointerVector &features,
                               const PatchHypothesisList &currentPatchHyps)
{
  // Remove old features and old patches
  mVotingSpace.clear();
  clearPatches();

  // Add the new features
  addFeatures(features);

  // ... and search for new ones:
  findPatches(currentPatchHyps);

  return mPatchHypotheses;
}

PatchHypothesisList
StreetPatcher::generateCrossPatches(const FeaturePointerVector &features,
                                    const PatchHypothesisList &currentPatchHyps)
{
  // Remove old features and old patches
  mVotingSpace.clear();
  clearPatches();
  mNNetOutputProcessed = false;

  // Add the new features
  addFeatures(features);

  // ... and search for new ones:
  findCrossPatches(currentPatchHyps);

  return mPatchHypotheses;
}

void StreetPatcher::setNumberOfPatches(int numOfPatches)
{
  mNumOfPatches = numOfPatches;
}

void
StreetPatcher::update(const PatchHypothesis &currentRefHyp, const std::list<float> angleHistory)
{
  mCurrentRefHyp = currentRefHyp;
  mAngleHistory = angleHistory;
}


/******************************************************************************/
/***************************** private methods ********************************/
/******************************************************************************/

void StreetPatcher::addFeatures(const FeaturePointerVector& features)
{
  for (const Feature* f : features)
  {
    addFeature(f);
  }
}

void StreetPatcher::addFeature(const Feature* f)
{
  if (!f)
  {
    return;
  }

  for (const Vote &vote : mVotes.at(f->type))
  {
    mVotingSpace.addFeatureVote(f->localPose, vote);
  }
}

void StreetPatcher::addPatchHypothesis(const PatchHypothesis &patchHyp)
{
  if (patchHyp.getPatchType() < mPatchHypVotes.size()) {
    for (unsigned int i = 0; i < mPatchHypVotes.at(patchHyp.getPatchType()).size(); i++)
    {
      Vote vote = mPatchHypVotes.at(patchHyp.getPatchType()).at(i);
      mVotingSpace.addPatchHypVote(patchHyp.getPose(), vote);
    }
  } else {
    std::cout << "SOMETHING IS VERY WRONG! " << patchHyp.getPatchType() << std::endl;
  }
}

void StreetPatcher::findPatches(const PatchHypothesisList &currentPatchHyps)
{
  mVotingSpace.setThresholds(STREETPATCH_MIN_VOTE_COMBINED, STREETPATCH_MIN_VOTE_FEATURE_ONLY);

  PatchHypothesis localRefHyp = mCurrentRefHyp;
  for (int numFoundPatches = 0; numFoundPatches < mNumOfPatches; numFoundPatches++)
  {
#ifdef DEBUG_ACTIVE
    dbgCurrentRefPatch = localRefHyp;
    dbgFoundNewPatch = false;
#endif

    addPatchHypothesis(localRefHyp);
    RegionOfInterest roi = localRefHyp.getROIForNextPatch();

    int valueOfMax = 0;
    OadrivePose locationOfMax;
    locationOfMax = mVotingSpace.searchForMaximum(roi, &valueOfMax, false);

    // If the value is greater zero a maximum has been found and a new patch is created for that.
    // This new patch is also the reference patch for the next iteration.
    if (valueOfMax > 0)
    {
      PatchHypothesis newPatch = createPatchHypothesis(locationOfMax, valueOfMax,
                                                       localRefHyp.getPatchID() + 1,
                                                       PatchHypothesis::Type::ROAD);
      bool replaced = false;
      // if we found a street patch with an ID equal to the ID of an intersection, the patch is
      // discared. No need for patches at the position of intersections.
      for (PatchHypothesis patchHyp : currentPatchHyps)
      {
        if (patchHyp.getPatchID() == newPatch.getPatchID())
        {
          if (patchHyp.getPatchType() == PatchHypothesis::Type::INTERSECTION)
          {
            newPatch = patchHyp;
            replaced = true;
            break;
          }
        }
      }

#ifdef DEBUG_ACTIVE
      dbgNewPatch = newPatch;
      dbgFoundNewPatch = true;
#endif
      if (!replaced)
      {
        mPatchHypotheses.push_back(newPatch);
      }
      localRefHyp = newPatch;
    }

      // No maximum has been found. In that case if we already have a patch hypothesis for the
      // ID under consideration we take this as next reference patch. In not we predict the next
      // reference patch based on the current reference patch.
    else
    {
      int nextID = localRefHyp.getPatchID() + 1;

      bool found = false;
      PatchHypothesis tmpLocalPatch;
      // Check if we can use an already existing patch as next reference patch.
      for (PatchHypothesis patch : currentPatchHyps)
      {
        if (patch.getPatchID() == nextID)
        {
          tmpLocalPatch = patch;
          tmpLocalPatch.setBasedOn(PatchHypothesis::BasedOn::PREVIOUS_HYP);
          found = true;
        }
      }

      // Predict a patch for the next reference patch.
      if (!found)
      {
        OadrivePose pose = roi.center;
        pose.setYaw(predictAngle(pose.getYaw()));
        tmpLocalPatch = PatchHypothesis(pose, nextID, 0.f,
                                        PatchHypothesis::Type::ROAD,
                                        PatchHypothesis::BasedOn::PREDICTION);

        mPatchHypotheses.push_back(tmpLocalPatch);
      }

#ifdef DEBUG_ACTIVE
      dbgNewPatch = tmpLocalPatch;
      dbgFoundNewPatch  = true;
#endif

      localRefHyp = tmpLocalPatch;
    }

#ifdef DEBUG_ACTIVE
    dbgCurrentIt = numFoundPatches;
    dbgVisualizeVotingSpace();
#endif
  }
}

void StreetPatcher::findCrossPatches(const PatchHypothesisList &currentPatchHyps)
{
  mVotingSpace.setThresholdsFeatureOnly(CROSSPATCH_MIN_VOTE_FEATURE_ONLY);

  for (const PatchHypothesis &patchHyp : currentPatchHyps)
  {
#ifdef DEBUG_ACTIVE
    dbgFoundNewPatch  = false;
    dbgCurrentRefPatch = patchHyp;
#endif

    std::vector<RegionOfInterest> roiList = patchHyp.getROIForCrossPatches();

    for (RegionOfInterest &roi : roiList)
    {
      int valueOfMax = 0;
      OadrivePose locationOfMax;

      locationOfMax = mVotingSpace.searchForMaximum(roi, &valueOfMax, true);

      // If the value is greater zero a maximum has been found and a new patch is created for that.
      if (valueOfMax > 0)
      {
        PatchHypothesis newPatch = createPatchHypothesis(locationOfMax, valueOfMax,
                                                         patchHyp.getPatchID() + 2,
                                                         PatchHypothesis::Type::CROSSING_ROAD);
        mPatchHypotheses.push_back(newPatch);
        #ifdef DEBUG_ACTIVE
          dbgNewPatch = newPatch;
          dbgFoundNewPatch = true;
        #endif
      }

#ifdef DEBUG_ACTIVE
      dbgVisualizeVotingSpace();
#endif
    }
  }
}

float StreetPatcher::predictAngle(float baseAngle)
{
  // At the beginning if the history isn't filled skip.
  if (mAngleHistory.size() < 6)
  {
    return baseAngle;
  }

  // Count the angle deltas with positive and negative sign.
  std::list<float> positives, negatives;
  float sumPositives = 0.f, sumNegatives = 0.f;
  for (auto angleIt = mAngleHistory.begin(); angleIt != mAngleHistory.end();)
  {
    auto predecessor = angleIt;
    angleIt++;

    if (angleIt != mAngleHistory.end())
    {
      float angleDiff = -((*angleIt) - (*predecessor));

      if (angleDiff < 0.f)
      {
        negatives.push_back(angleDiff);
        sumNegatives += angleDiff;
      }

      if (angleDiff > 0.f)
      {
        positives.push_back(angleDiff);
        sumPositives += angleDiff;
      }
    }
  }

  // If most of the angle deltas have the same sign, add the mean of them.
  if (positives.size() >= mAngleHistory.size() - 2)
  {
    return baseAngle + (sumPositives / float(positives.size()));
  }
  else if (negatives.size() >= mAngleHistory.size() - 2)
  {
    return baseAngle + (sumNegatives / float(negatives.size()));
  }
  else
  {
    return baseAngle;
  }
}

bool StreetPatcher::verifyCrossPatch(const cv::Mat &neuralNetOutput,
                                     const PatchHypothesis &crossPatchHyp)
{
  if (!mNNetOutputProcessed)
  {
    // Convert every label in a street label. Centerline and intersection aren't used so far.
    cv::threshold(neuralNetOutput, mNeuralNetOutput, 0.1, 1.0, 0);

    // Resize to fit to the voting space
    cv::resize(mNeuralNetOutput, mNeuralNetOutput, mVotingSpace.getVotingSpaceSize(), 0, 0,
               CV_INTER_LINEAR);

    mNNetOutputProcessed = true;
  }

  RegionOfInterest roi;
  roi.center = crossPatchHyp.getPose();
  roi.width = PATCH_WIDTH_STRAIGHT;
  roi.length = PATCH_LENGTH_STRAIGHT;

  cv::Mat extractedRegion;
  cv::Size2f areaSize;
  util::Polygon roiAsPolygon = mVotingSpace.convertRoiToPolygon(roi);
  // Extract the region of the patch and count the street labels. If number is creater than
  // threshold accept the patch
  if (ImageHelper::extractRegionOfInterest(mNeuralNetOutput, roiAsPolygon, extractedRegion, &areaSize))
  {

    double max;
    cv::minMaxLoc(extractedRegion, NULL, &max, NULL, NULL);
    cv::Vec4f test = cv::sum(extractedRegion);
    float score = test[0] / (areaSize.width * areaSize.height);
    LOGGING_INFO(lanedetectionLogger, "SCORE " << score << endl);

    if (score >= CROSSPATCH_MIN_OVERLAP)
    {
      return true;
    }
  }

  return false;
}


void StreetPatcher::initFeatureVotes()
{

  ////////////////////////////////////////////
  // RIGHT_SIDE_LINE:
  //      ||              ||
  //      ||      |       ||
  //      ||      |<------||
  //      ||      |       ||
  //      ||              ||
  ////////////////////////////////////////////
  cv::Size2f voteSize(0.03, 0.4);
  cv::Point2f votePosition(0.f, HALF_ROAD_WIDTH);
  Vote vote(votePosition, voteSize, 10 * mMinimumVote, 0.f);
  mVotes[RIGHT_SIDE_LINE].push_back(vote);

  ////////////////////////////////////////////
  // LEFT_SIDE_LINE:
  //      ||              ||
  //      ||      |       ||
  //      ||----->|       ||
  //      ||      |       ||
  //      ||              ||
  ////////////////////////////////////////////
  votePosition = cv::Point2f(0.f, -HALF_ROAD_WIDTH);
  vote = Vote(votePosition, voteSize, 10 * mMinimumVote, 0.f);
  mVotes[LEFT_SIDE_LINE].push_back(vote);

  ////////////////////////////////////////////
  // CENTER_LINE:
  //      ||              ||
  //      ||      |       ||
  //      ||     >|<      ||
  //      ||      |       ||
  //      ||              ||
  ////////////////////////////////////////////
  votePosition = cv::Point2f(0.f, 0.f);
  vote = Vote(votePosition, voteSize, 10 * mMinimumVote, 0.f);
  mVotes[CENTER_LINE].push_back(vote);

  ////////////////////////////////////////////
  // UNKNOW STREET LINE:
  //      ||              ||
  //      ||      |       ||
  //      ||<---->|<----->||
  //      ||      |       ||
  //      ||              ||
  ////////////////////////////////////////////
  votePosition = cv::Point2f(0.f, 0.f);
  vote = Vote(votePosition, voteSize, mMinimumVote, 0.f);
  mVotes[STREET_LINE].push_back(vote);
  votePosition = cv::Point2f(0.f, -HALF_ROAD_WIDTH);
  vote = Vote(votePosition, voteSize, mMinimumVote, 0.f);
  mVotes[STREET_LINE].push_back(vote);
  votePosition = cv::Point2f(0.f, HALF_ROAD_WIDTH);
  vote = Vote(votePosition, voteSize, mMinimumVote, 0.f);
  mVotes[STREET_LINE].push_back(vote);
}

void StreetPatcher::initPatchVotes()
{
  cv::Size2f voteSize(0.3, 0.03);
  cv::Point2f votePosition(PATCH_LENGTH_STRAIGHT, 0.f);

  ////////////////////////////////////////////
  // Vote for patch in front with same angle and one angle region left and right:
  //      ||...............||
  //      ||.      +      .||
  //      ||.......|.......||
  //      ||-------|-------||
  //      ||       |       ||
  //      ||---------------||
  ////////////////////////////////////////////
  Vote vote(votePosition, voteSize, 25 * mMinimumVote, 0.f);
  mPatchHypVotes[PatchHypothesis::Type::ROAD].push_back(vote);

  vote = Vote(votePosition, voteSize, 25 * mMinimumVote, mVotingSpace.getAngleRegionSize());
  mPatchHypVotes[PatchHypothesis::Type::ROAD].push_back(vote);

  vote = Vote(votePosition, voteSize, 25 * mMinimumVote, -mVotingSpace.getAngleRegionSize());
  mPatchHypVotes[PatchHypothesis::Type::ROAD].push_back(vote);
}

void StreetPatcher::initCrossPatchVotes()
{
  cv::Size2f voteSize(0.3, 0.03);
  cv::Point2f votePosition(0.5 * PATCH_LENGTH_INTERSECTION + 0.5 * PATCH_LENGTH_STRAIGHT, 0.f);
  const int voteFactor = 20;

  ////////////////////////////////////////////
  // Vote for patch in front with same angle and one angle region left and right:
  //      ||...............||
  //      ||.      +      .||
  //      ||.......|.......||
  //      ||-------|-------||
  //      ||       |       ||
  //      ||---------------||
  ////////////////////////////////////////////
  Vote vote(votePosition, voteSize, voteFactor * mMinimumVote, 0.f);
  mPatchHypVotes[PatchHypothesis::Type::INTERSECTION].push_back(vote);

  vote = Vote(votePosition, voteSize, voteFactor * mMinimumVote, mVotingSpace.getAngleRegionSize());
  mPatchHypVotes[PatchHypothesis::Type::INTERSECTION].push_back(vote);

  vote = Vote(votePosition, voteSize, voteFactor * mMinimumVote, -mVotingSpace.getAngleRegionSize());
  mPatchHypVotes[PatchHypothesis::Type::INTERSECTION].push_back(vote);
}




/******************************************************************************/
/****************************** debug methods *********************************/
/******************************************************************************/

#ifdef DEBUG_ACTIVE

void StreetPatcher::dbgVisualizeVotingSpace()
{
  if (dbgVisualizationEnabled)
  {
    cv::Mat* debugImage = DebugViewRoadPerception::generateHoughSpaceDebugImage(&mVotingSpace,
                                                                                this);
    imshow("DebugImage", *debugImage);
    cv::waitKey(STEPPING_DISABLED);
  }
}

#endif

}   // namespace
}   // namespace

