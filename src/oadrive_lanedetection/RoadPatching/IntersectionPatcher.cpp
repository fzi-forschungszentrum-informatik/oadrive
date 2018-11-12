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

#include "IntersectionPatcher.h"
#include <oadrive_lanedetection/lanedetectionLogging.h>

using oadrive::util::ImageHelper;
using oadrive::world::TrafficSign;
using icl_core::logging::endl;
using icl_core::logging::flush;

namespace oadrive
{
namespace lanedetection
{


IntersectionPatcher::IntersectionPatcher(cv::Size2i birdviewSize,
                                         util::CoordinateConverter* coordConverter) :
        RoadPatcher(birdviewSize, coordConverter, 10, 5),
        mVotingSpace(coordConverter, birdviewSize)
{

  LOGGING_INFO(lanedetectionLogger, "Generating IntersectionPatcher: " << endl);

  initPatchVotes();
  initTrafficSignVotes();

#ifdef DEBUG_ACTIVE
  dbgVisualizationEnabled = true;
#endif
}

/******************************************************************************/
/***************************** public methods *********************************/
/******************************************************************************/

PatchHypothesisList
IntersectionPatcher::generateIntersectionPatches(const OadrivePose currentCarPose,
                                                 const PatchHypothesisList &currentPatchHyps,
                                                 const PatchHypothesisList &currentCrossPatchHyps,
                                                 const TrafficSign* currentTrafficSign = nullptr)
{
  // Remove old features and old patches
  mVotingSpace.clear();
  clearPatches();

  // Add the new features
  for (const PatchHypothesis &patchHyp : currentPatchHyps)
  {
    addPatchHypothesis(patchHyp);
  }

  for (const PatchHypothesis &patchHyp : currentCrossPatchHyps)
  {
    addPatchHypothesis(patchHyp);
  }


  addTrafficSign(currentTrafficSign);

  // ... and search for new ones:
  findIntersectionPatch();

  return mPatchHypotheses;
}

/******************************************************************************/
/***************************** private methods ********************************/
/******************************************************************************/

void IntersectionPatcher::addPatchHypothesis(const PatchHypothesis &patchHyp)
{
  for (unsigned int i = 0; i < mPatchHypVotes.at(patchHyp.getPatchType()).size(); i++)
  {
    Vote vote = mPatchHypVotes.at(patchHyp.getPatchType()).at(i);
    if (patchHyp.getPatchType() == PatchHypothesis::CROSSING_ROAD)
    {
      mVotingSpace.addCrossPatchVote(patchHyp.getPose(), vote);
    }
    else
    {
      mVotingSpace.addPatchVote(patchHyp.getPose(), vote);
    }
  }
}

void IntersectionPatcher::addTrafficSign(const TrafficSign* trafficSign)
{
  if (!trafficSign || !trafficSign->isIntersectionTrafficSign())
  {
    return;
  }

  for (unsigned int i = 0; i < mPatchHypVotes.at(PatchHypothesis::Type::TRAFFICSIGN).size(); i++)
  {
    Vote vote = mPatchHypVotes.at(PatchHypothesis::Type::TRAFFICSIGN).at(i);
    OadrivePose pose(trafficSign->getX(), trafficSign->getY(), trafficSign->getYaw());
    mVotingSpace.addTrafficSignVote(pose, vote);
  }
}

void IntersectionPatcher::findIntersectionPatch()
{
  int valueOfMax = 0;
  OadrivePose locationOfMax;

  bool basedOnTrafficSign = false;
  locationOfMax = mVotingSpace.searchForMaximum(&valueOfMax, &basedOnTrafficSign);

  // If the value is greater zero a maximum has been found and a new patch hypothesis is created
  // for that.
  if (valueOfMax > 0)
  {
    PatchHypothesis newPatch = createPatchHypothesis(locationOfMax, valueOfMax, -1,
                                                     PatchHypothesis::Type::INTERSECTION);
    if (basedOnTrafficSign)
    {
      newPatch.setBasedOn(PatchHypothesis::BasedOn::TRAFFIC_SIGN);
    }
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

PatchHypothesis
IntersectionPatcher::searchIntersectionInNNSegmentation(const cv::Mat &neuralNetOutput,
                                                        const PatchHypothesisList &currentPatchHyps)
{
  // Resize to fit to the voting space
  cv::resize(neuralNetOutput, neuralNetOutput, mVotingSpace.getVotingSpaceSize(), 0, 0,
             CV_INTER_LINEAR);

  for (const PatchHypothesis &patchHyp : currentPatchHyps)
  {
    std::vector<RegionOfInterest> roiList =
            patchHyp.getROIsForIntersection(mVotingSpace.getAngleRegionSize());

    for (RegionOfInterest &roi : roiList)
    {
      cv::Mat extractedRegion;
      cv::Size2f areaSize;
      util::Polygon roiAsPolygon = mVotingSpace.convertRoiToPolygon(roi);
      if (ImageHelper::extractRegionOfInterest(neuralNetOutput, roiAsPolygon, extractedRegion,
                                               &areaSize))
      {
        double max;
        cv::minMaxLoc(extractedRegion, NULL, &max, NULL, NULL);
        cv::Vec4f test = cv::sum(extractedRegion);
        float score = test[0] / (areaSize.width * areaSize.height);
        LOGGING_INFO(lanedetectionLogger, "SCORE Intersection " << score << endl);

        if (score >= INTERSECTION_MIN_OVERLAP)
        {
          PatchHypothesis newPatch = createPatchHypothesis(roi.center, 0.01,
                                                           patchHyp.getPatchID() + 1,
                                                           PatchHypothesis::Type::INTERSECTION);
        }
      }
    }
  }
}

void IntersectionPatcher::initPatchVotes()
{

  ////////////////////////////////////////////
  ///
  //      ||...............||
  //      ||.             .||
  //      ||.             .||
  //      ||.             .||
  //      ||.      +      .||
  //      ||.      |      .||
  //      ||.......|.......||
  //      ||-------|-------||
  //      ||       |       ||
  //      ||---------------||
  ////////////////////////////////////////////
  cv::Size2f voteSize(0.05, 0.8);
  cv::Point2f votePosition(PATCH_LENGTH_STRAIGHT * 0.5f + PATCH_LENGTH_INTERSECTION * 0.5f, 0.f);
  Vote vote(votePosition, voteSize, 10 * mMinimumVote, 0.f);
  mPatchHypVotes[PatchHypothesis::Type::ROAD].push_back(vote);
  vote = Vote(votePosition, voteSize, 10 * mMinimumVote, -mVotingSpace.getAngleRegionSize());
  mPatchHypVotes[PatchHypothesis::Type::ROAD].push_back(vote);
  vote = Vote(votePosition, voteSize, 10 * mMinimumVote, -2 * mVotingSpace.getAngleRegionSize());
  mPatchHypVotes[PatchHypothesis::Type::ROAD].push_back(vote);
  vote = Vote(votePosition, voteSize, 10 * mMinimumVote, mVotingSpace.getAngleRegionSize());
  mPatchHypVotes[PatchHypothesis::Type::ROAD].push_back(vote);
  vote = Vote(votePosition, voteSize, 10 * mMinimumVote, 2 * mVotingSpace.getAngleRegionSize());
  mPatchHypVotes[PatchHypothesis::Type::ROAD].push_back(vote);

  votePosition.x *= -1;
  vote = Vote(votePosition, voteSize, 5 * mMinimumVote, M_PI_2);
  mPatchHypVotes[PatchHypothesis::Type::CROSSING_ROAD].push_back(vote);
  vote = Vote(votePosition, voteSize, 5 * mMinimumVote, M_PI_2 - mVotingSpace.getAngleRegionSize());
  mPatchHypVotes[PatchHypothesis::Type::CROSSING_ROAD].push_back(vote);
  vote = Vote(votePosition, voteSize, 5 * mMinimumVote, M_PI_2 - 2 * mVotingSpace.getAngleRegionSize
          ());
  mPatchHypVotes[PatchHypothesis::Type::CROSSING_ROAD].push_back(vote);
  vote = Vote(votePosition, voteSize, 5 * mMinimumVote, M_PI_2 + mVotingSpace.getAngleRegionSize());
  mPatchHypVotes[PatchHypothesis::Type::CROSSING_ROAD].push_back(vote);
  vote = Vote(votePosition, voteSize, 5 * mMinimumVote, M_PI_2 + 2 * mVotingSpace.getAngleRegionSize
          ());
  mPatchHypVotes[PatchHypothesis::Type::CROSSING_ROAD].push_back(vote);

}

void IntersectionPatcher::initTrafficSignVotes()
{
  // In this case the first one is interpreted as width and the second as angle because we have
  // a circular voting shape.
  cv::Size2f voteSize(0.05, M_PI / 3.f);
  float distIntersectionCornerToTrafficSignX = 0.325f;
  float distIntersectionCornerToTrafficSignY = 0.15f;

  cv::Point2f votePosition(-PATCH_LENGTH_INTERSECTION * 0.5f - distIntersectionCornerToTrafficSignX,
                           -PATCH_WIDTH_INTERSECTION * 0.5f - distIntersectionCornerToTrafficSignY);
  Vote vote(votePosition, voteSize, 10 * mMinimumVote, 0.f);
  mPatchHypVotes[PatchHypothesis::Type::TRAFFICSIGN].push_back(vote);

  vote = Vote(votePosition, voteSize, 10 * mMinimumVote, -mVotingSpace.getAngleRegionSize());
  mPatchHypVotes[PatchHypothesis::Type::TRAFFICSIGN].push_back(vote);

  vote = Vote(votePosition, voteSize, 10 * mMinimumVote, mVotingSpace.getAngleRegionSize());
  mPatchHypVotes[PatchHypothesis::Type::TRAFFICSIGN].push_back(vote);

  vote = Vote(votePosition, voteSize, 10 * mMinimumVote, -2 * mVotingSpace.getAngleRegionSize());
  mPatchHypVotes[PatchHypothesis::Type::TRAFFICSIGN].push_back(vote);

  vote = Vote(votePosition, voteSize, 10 * mMinimumVote, 2 * mVotingSpace.getAngleRegionSize());
  mPatchHypVotes[PatchHypothesis::Type::TRAFFICSIGN].push_back(vote);
}

#ifdef DEBUG_ACTIVE
void IntersectionPatcher::dbgVisualizeVotingSpace()
{
  if (dbgVisualizationEnabled)
  {
    cv::Mat* debugImage = DebugViewRoadPerception::generateHoughSpaceDebugImage(&mVotingSpace,
                                                                                this);
    imshow("Intersection", *debugImage);
    cv::waitKey(STEPPING_DISABLED);
  }
}
#endif

}
}