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

#include "IntersectionVotingSpace.h"

using oadrive::util::ImageHelper;
using icl_core::logging::endl;
using icl_core::logging::flush;

namespace oadrive
{
namespace lanedetection
{

IntersectionVotingSpace::IntersectionVotingSpace(util::CoordinateConverter* coordConverter,
                                                 cv::Size2i size) :
        VotingSpace(coordConverter, size),
        mHoughSpacePatches(mNumAngleRegions, cv::Size2i(mCols, mRows)),
        mHoughSpaceCrossPatches(mNumAngleRegions, cv::Size2i(mCols, mRows)),
        mHoughTrafficSigns(mNumAngleRegions, cv::Size2i(mCols, mRows))
{
#ifdef DEBUG_ACTIVE
  dbgRegionsToDebug.push_back(&mHoughSpacePatches);
  dbgRegionsToDebug.push_back(&mHoughSpaceCrossPatches);
  dbgRegionsToDebug.push_back(&mHoughTrafficSigns);
  dbgHasROI = false;
#endif
}


/******************************************************************************/
/***************************** public methods *********************************/
/******************************************************************************/


void IntersectionVotingSpace::addCrossPatchVote(const OadrivePose &featurePose,
                                                const Vote &vote)
{
  addVote(featurePose, vote, &mHoughSpaceCrossPatches);
}

void IntersectionVotingSpace::addPatchVote(const OadrivePose &patchPose, const Vote &vote)
{
  addVote(patchPose, vote, &mHoughSpacePatches);
}

void IntersectionVotingSpace::addTrafficSignVote(const OadrivePose &trafficSignLocalPose,
                                                 const Vote &vote)
{
  // rotate by 180° because the orientation of a traffic sign is pointing towards us.
  float angle = trafficSignLocalPose.getYaw() + vote.angleOffset;

  // Determine which angle map this patch should vote into:
  uint angleRegion = angle2AngleRegionID(angle);

  // discretize the pose position and place it into the corresponding pixel:
  cv::Point2f imagePos = mCoordConverter->car2VotingSpace(mCenterPose, mCenterPixel,
                                                          trafficSignLocalPose, mScale);

  float distance = sqrt(pow(vote.pos.x, 2.f) + pow(vote.pos.y, 2.f));
  float startAngle = atan2(vote.pos.y, vote.pos.x);

  util::Polygon poly(util::PointList(0), imagePos);
  float stepsize = 5.f * M_PI / 180.f; // 5°
  float distanceOuter = distance + vote.size.width * 0.5;
  float distanceInner = distance - vote.size.width * 0.5;

  // The vote size height is used as angle in this special case.
  // Outer vertices
  for (float angle = -vote.size.height / 2.f; angle < vote.size.height / 2.f; angle += stepsize)
  {
    cv::Point2f newPointOut(cos(startAngle + angle) * distanceOuter *
                            mCoordConverter->getPixelsPerMeterX(mScale),
                            sin(startAngle + angle) * distanceOuter *
                            mCoordConverter->getPixelsPerMeterY(mScale));
    poly.vertices.push_back(newPointOut + poly.refPoint);
  }

  // Inner vertices
  for (float angle = vote.size.height / 2.f; angle > -vote.size.height / 2.f; angle -= stepsize)
  {
    cv::Point2f newPointIn(
            cos(startAngle + angle) * distanceInner * mCoordConverter->getPixelsPerMeterX(mScale),
            sin(startAngle + angle) * distanceInner * mCoordConverter->getPixelsPerMeterY(mScale));

    poly.vertices.push_back(newPointIn + poly.refPoint);
  }

  // Add vote
  if (angleRegion >= 0 && angleRegion < mHoughTrafficSigns.regions.size())
  {
    ImageHelper::addPolygon2(mHoughTrafficSigns.regions.at(angleRegion), poly, vote.weight);
    mHoughTrafficSigns.regionWasUsed.at(angleRegion) = true;
  }
}

OadrivePose IntersectionVotingSpace::searchForMaximum(int* valueOfMaximum, bool* basedOnTafficSign)
{
  *valueOfMaximum = -1;
  bool foundMaximum = false;


  MaxVote maximum = extractMaximum(mHoughSpacePatches, mHoughSpaceCrossPatches);
  if (maximum.value >= INTERSECTION_PATCH_BASED_MIN_VOTE)
  {
    (*valueOfMaximum) = maximum.value;
     *basedOnTafficSign = false;
    foundMaximum = true;
  }

  if (!foundMaximum)
  {
    maximum = extractMaximum(mHoughSpacePatches, mHoughTrafficSigns);
    if (maximum.value >= INTERSECTION_TRAFFIC_BASED_MIN_VOTE)
    {
      (*valueOfMaximum) = maximum.value;
      *basedOnTafficSign = true;
    }
  }

#ifdef DEBUG_ACTIVE
  dbgCurrentAngleRegion = angle2AngleRegionID(maximum.angle);
  dbgValueOfMax = maximum.value;
  dbgMaxAngleRegion = maximum.angle;
#endif

  // If at least one region was found where the highest vote is higher than required min:
  if ((*valueOfMaximum) > 0)
  {
    cv::Point2f imgPositionMax(maximum.location.x, maximum.location.y);

#ifdef DEBUG_ACTIVE
    dbgPosOfMax = imgPositionMax;
#endif

    OadrivePose localPatchPose = mCoordConverter->votingSpace2Car(mCenterPose, mCenterPixel,
                                                                  imgPositionMax, mScale);

    localPatchPose.setYaw(maximum.angle);
    return localPatchPose;
  }

  return OadrivePose();
}

void IntersectionVotingSpace::clear()
{
  mHoughSpacePatches.reset();
  mHoughSpaceCrossPatches.reset();
  mHoughTrafficSigns.reset();
}

/******************************************************************************/
/**************************** private methods *********************************/
/******************************************************************************/

VotingSpace::MaxVote
IntersectionVotingSpace::extractMaximum(const VotingContainer &votingSpaceA,
                                        const VotingContainer &votingSpaceB)
{
  // If both voting spaces should be used add also the patch voting space;
  VotingSpace::MaxVote maximumVote, tempMaxVote;
  maximumVote.value = 0;
  cv::Mat combinedVotingSpace(mRows, mCols, CV_16UC1);
  int totalScore = 0;
  for (int i = 0; i < mNumAngleRegions; i++)
  {
    if (!votingSpaceA.regionWasUsed.at(i) || !votingSpaceB.regionWasUsed.at(i))
    {
      continue;
    }

    tempMaxVote = findMaxVote(votingSpaceA.regions.at(i), votingSpaceB.regions.at(i));

    if (tempMaxVote.value > maximumVote.value)
    {
      maximumVote = tempMaxVote;
      maximumVote.angle = angleRegion2Angle(i);
      totalScore = maximumVote.value;
    }
    else if (tempMaxVote.value > 0 &&
             tempMaxVote.value == maximumVote.value &&
             tempMaxVote.location == maximumVote.location)
    {
      float tempAngle = angleRegion2Angle(i);
      maximumVote.angle = (maximumVote.angle * totalScore + tempAngle * tempMaxVote.value) /
                          (totalScore + tempMaxVote.value);
      totalScore += tempMaxVote.value;
    }
  }
  return maximumVote;
}

}

}