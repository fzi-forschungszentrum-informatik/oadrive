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
 */
//----------------------------------------------------------------------

#include "StreetVotingSpace.h"

using icl_core::logging::endl;
using icl_core::logging::flush;

namespace oadrive
{
namespace lanedetection
{

StreetVotingSpace::StreetVotingSpace(util::CoordinateConverter* coordConverter,
                                     OadrivePose centerPose, cv::Size2i size) :
        VotingSpace(coordConverter, size),
        mHoughSpaceFeatures(mNumAngleRegions, cv::Size2i(mCols, mRows)),
        mHoughSpacePatches(mNumAngleRegions, cv::Size2i(mCols, mRows))
{
#ifdef DEBUG_ACTIVE
  dbgRegionsToDebug.push_back(&mHoughSpacePatches);
  dbgRegionsToDebug.push_back(&mHoughSpaceFeatures);

  dbgHasROI = true;
#endif
}



/******************************************************************************/
/***************************** public methods *********************************/
/******************************************************************************/


void StreetVotingSpace::addFeatureVote(const OadrivePose &featurePose, const Vote &vote)
{
  addVote(featurePose, vote, &mHoughSpaceFeatures);
}

void StreetVotingSpace::addPatchHypVote(const OadrivePose &patchHypPose, const Vote &vote)
{
  addVote(patchHypPose, vote, &mHoughSpacePatches);
}


OadrivePose StreetVotingSpace::searchForMaximum(const RegionOfInterest &regionOfInterest,
                                                int* valueOfMaximum,
                                                bool clear)
{
  *valueOfMaximum = -1;

  int refPatchAngleRegion = angle2AngleRegionID(regionOfInterest.center.getYaw());
#ifdef DEBUG_ACTIVE
  dbgCurrentAngleRegion = refPatchAngleRegion;
  dbgCurrentRegion = regionOfInterest;
#endif

  MaxVote maximum;
  if (mCombinedSearch)
  {
    // Consider features votes as well as patch votes
    maximum = extractMaximum(regionOfInterest, refPatchAngleRegion, true);
    if (maximum.value >= mMinScoreCombined)
    {
      (*valueOfMaximum) = maximum.value;
    }
  }

  if (mFeatureOnlySearch && maximum.value < mMinScoreCombined)
  {
    // Only consider features votes
    maximum = extractMaximum(regionOfInterest, refPatchAngleRegion, false);
    if (maximum.value > mMinScoreFeatureOnly)
    {
      (*valueOfMaximum) = maximum.value;
    }
  }

#ifdef DEBUG_ACTIVE
  dbgValueOfMax = maximum.value;
  dbgMaxAngleRegion = refPatchAngleRegion;
#endif

  // If at least one region was found where the highest score is greater than zero
  if ((*valueOfMaximum) > 0)
  {
    cv::Point2f imgPositionMax(maximum.location.x, maximum.location.y);
    imgPositionMax = transformToGlobalVotingSpace(imgPositionMax, regionOfInterest);
#ifdef DEBUG_ACTIVE
    dbgPosOfMax = imgPositionMax;
#endif
    OadrivePose localPatchPose = mCoordConverter->votingSpace2Car(mCenterPose, mCenterPixel,
                                                                  imgPositionMax, mScale);

    double angle = calculateAngle(mHoughSpaceFeatures.regions, refPatchAngleRegion, imgPositionMax);
    localPatchPose.setYaw(angle);
    // Determines if a region around the found patch is cleared in the voting space.
    if (clear)
    {
      setRegion(angle2AngleRegionID(angle), imgPositionMax,
                cv::Size2i(0.25 * mMeters2Pixels.x, 0.1 * mMeters2Pixels.y), 0);
    }

    return localPatchPose;
  }

  return OadrivePose();
}

void StreetVotingSpace::clear()
{
  mHoughSpacePatches.reset();
  mHoughSpaceFeatures.reset();
}

void
StreetVotingSpace::setThresholds(unsigned int minScoreCombined, unsigned int minScoreFeatureOnly)
{
  mMinScoreCombined = minScoreCombined;
  mMinScoreFeatureOnly = minScoreFeatureOnly;
  mFeatureOnlySearch = true;
  mCombinedSearch = true;
}

void StreetVotingSpace::setThresholdsFeatureOnly(unsigned int minScoreFeatureOnly)
{
  mMinScoreFeatureOnly = minScoreFeatureOnly;
  mMinScoreCombined = std::numeric_limits<unsigned int>::max();
  mFeatureOnlySearch = true;
  mCombinedSearch = false;
}

void StreetVotingSpace::setThresholdsCombinedOnly(unsigned int minScoreCombined)
{
  mMinScoreCombined = minScoreCombined;
  mMinScoreFeatureOnly = std::numeric_limits<unsigned int>::max();
  mFeatureOnlySearch = false;
  mCombinedSearch = true;
}


/******************************************************************************/
/**************************** private methods *********************************/
/******************************************************************************/

VotingSpace::MaxVote StreetVotingSpace::extractMaximum(RegionOfInterest roi,
                                                       int refPatchAngleRegion,
                                                       bool useCombined)
{
  // Sum over the relevant voting spaces which are the CONSIDERED_NEIGBOUR_REGIONS neighbours
  // left and right.
  cv::Mat summedFeatureVotingSpaces;
  cv::Mat summedPatchVotingSpaces;
  for (int i = -CONSIDERED_NEIGBOUR_REGIONS; i <= +CONSIDERED_NEIGBOUR_REGIONS; i++)
  {
    int angleReg = getNeighbourAngleRegion(refPatchAngleRegion, i);

    cv::Mat relevantVotingSpace;
    if (!calculateRelevantVotingSpace(mHoughSpaceFeatures.regions.at(angleReg), roi,
                                      relevantVotingSpace))
    {
      break;
    }

    // first iteration: initilize data structures.
    if (i == -CONSIDERED_NEIGBOUR_REGIONS)
    {
      summedFeatureVotingSpaces = cv::Mat(relevantVotingSpace.rows, relevantVotingSpace.cols,
                                          CV_16UC1,
                                          cv::Scalar(0));
      summedPatchVotingSpaces = cv::Mat(relevantVotingSpace.rows, relevantVotingSpace.cols,
                                        CV_16UC1,
                                        cv::Scalar(0));
    }
    summedFeatureVotingSpaces += relevantVotingSpace;

    // If both voting spaces should be used add also the patch voting space;
    if (useCombined)
    {
      if (!calculateRelevantVotingSpace(mHoughSpacePatches.regions.at(angleReg), roi,
                                        relevantVotingSpace))
      {
        break;
      }

      summedPatchVotingSpaces += relevantVotingSpace;
    }
  }
  double max = 0;
  cv::Point maxLoc;

  MaxVote maximumVote;
  if (useCombined)
  {
    // find the maximum over the minimums over both voting spaces
    maximumVote = findMaxVote(summedFeatureVotingSpaces, summedPatchVotingSpaces);
  }
  else
  {
    // find the maximum in the feature voting space.
    cv::minMaxLoc(summedFeatureVotingSpaces, NULL, &max, NULL, &maxLoc);
    maximumVote.value = ushort(max);
    maximumVote.location = maxLoc;
  }

  return maximumVote;
}

void StreetVotingSpace::setRegion(int angleRegion, cv::Point p, cv::Size size, int newAmount)
{
  cv::Rect r(p - cv::Point(size.width * 0.5, size.height * 0.5), size);
  for (int i = -CONSIDERED_NEIGBOUR_REGIONS; i <= +CONSIDERED_NEIGBOUR_REGIONS; i++)
  {
    int angleReg = getNeighbourAngleRegion(angleRegion, i);
    cv::rectangle(mHoughSpaceFeatures.regions.at(angleReg), r, cv::Scalar(newAmount), CV_FILLED);
    cv::rectangle(mHoughSpacePatches.regions.at(angleReg), r, cv::Scalar(newAmount), CV_FILLED);
  }
}

}
}