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

#include "PatchHypothesis.h"
#include "VotingSpace.h"

using icl_core::logging::endl;
using icl_core::logging::flush;

using namespace oadrive::core;

namespace oadrive
{

namespace lanedetection
{

float PatchHypothesis::ROI_LENGTH = 0.15f;
float PatchHypothesis::ROI_WIDTH = 0.5f;


PatchHypothesis::PatchHypothesis(OadrivePose pose, uint ID, float probability,
                                 PatchHypothesis::Type type, PatchHypothesis::BasedOn based) :
        mPose(pose),
        mPatchID(ID),
        mProbability(probability),
        mPatchType(type),
        mBasedOn(based)
{

}

void PatchHypothesis::mergeHypothesis(const PatchHypothesis &other)
{
  // if this is a intersection and the oder is a straight patch nothing happens.
  if (mPatchType != other.getPatchType())
  {
    return;
  }

  float angle2 = other.getPose().getYaw();
  if (other.getPatchType() == PatchHypothesis::Type::INTERSECTION)
  {
    switch (mDrivingDirection)
    {
      case DrivingDirection::STRAIGHT:
        break;
      case DrivingDirection::LEFT:
        angle2+= M_PI_2;
        break;
      case DrivingDirection::RIGHT:
        angle2 -= M_PI_2;
        break;
    }
  }

  // update the reference patch by also taking into account the new patch:
  Position2d p1( mPose.getX(), mPose.getY());
  Position2d p2(other.getPose().getX(), other.getPose().getY());
  Position2d unit1, unit2, yawVec;
  float angle1 = getPose().getYaw();

  unit1(0) = cos(angle1);
  unit1(1) = sin(angle1);
  unit2(0) = cos(angle2);
  unit2(1) = sin(angle2);
  yawVec = unit1 * mProbability + unit2 * other.getProbability();

  float weight = other.getProbability() /
                 (other.getProbability() + mProbability);

  Position2d p = p1 + (p2 - p1) * (weight);

  // TODO: Test different logic for patches derived from the map as they usually get better with time!
  if (other.getBasedOn() == PatchHypothesis::BasedOn::MAP) {
    const float keepAlpha = 0.7;
    p = keepAlpha * p1 + (1 - keepAlpha) * p2;
  }

  mPose = OadrivePose(p(0), p(1), atan2(yawVec(1), yawVec(0)));
  mProbability += other.getProbability();
}

int PatchHypothesis::getPatchID() const
{
  return mPatchID;
}

OadrivePose PatchHypothesis::getPose() const
{
  return mPose;
}

void PatchHypothesis::setPose(OadrivePose pose)
{
  if (pose.getYaw() > M_PI)
  {
    pose.setYaw(pose.getYaw() - 2 * M_PI);
  }
  if (pose.getYaw() <= -M_PI)
  {
    pose.setYaw(pose.getYaw() + 2 * M_PI);
  }

  mPose = pose;
}


PatchHypothesis::Type PatchHypothesis::getPatchType() const
{
  return mPatchType;
}

PatchHypothesis::BasedOn PatchHypothesis::getBasedOn() const
{
  return mBasedOn;
}

float PatchHypothesis::getProbability() const
{
  return mProbability;
}

float PatchHypothesis::getX() const
{
  return mPose.getX();
}

void PatchHypothesis::setX(float x)
{
  mPose.setX(x);
}

float PatchHypothesis::getY() const
{
  return mPose.getY();
}

void PatchHypothesis::setY(float y)
{
  mPose.setY(y);
}

float PatchHypothesis::getYaw() const
{
  return mPose.getYaw();
}

void PatchHypothesis::setYaw(float yaw)
{
  if (yaw > M_PI)
  {
    yaw -= 2 * M_PI;
  }
  if (yaw <= -M_PI)
  {
    yaw += 2 * M_PI;
  }

  mPose.setYaw(yaw);
}

void PatchHypothesis::setBasedOn(PatchHypothesis::BasedOn basedOn)
{
  mBasedOn = basedOn;
}

RegionOfInterest PatchHypothesis::getROIForNextPatch() const
{
  RegionOfInterest regionOfInterest;

  float roiOffsetX = 0.f;
  float roiOffsetY = 0.f;
  float angle = 0.f;
  if (mPatchType == ROAD)
  {
    roiOffsetX = PATCH_LENGTH_STRAIGHT;
    roiOffsetY = 0.f;
    angle = 0.f;
  }
  else if (mPatchType == INTERSECTION)
  {
    roiOffsetX = (PATCH_WIDTH_INTERSECTION * 0.5 + PATCH_LENGTH_STRAIGHT * 0.5);
    roiOffsetY = 0.f;
  }

  regionOfInterest.center = calculatePoseWithOffset(roiOffsetX, roiOffsetY, angle, angle);
  regionOfInterest.width = ROI_WIDTH;
  regionOfInterest.length = ROI_LENGTH;

  return regionOfInterest;
}

std::vector<RegionOfInterest> PatchHypothesis::getROIForCrossPatches() const
{
  std::vector<RegionOfInterest> regionOfInterestList(2);
  RegionOfInterest regionOfInterest;

  if (mPatchType == Type::ROAD)
  {
    float roiOffsetX = (PATCH_LENGTH_STRAIGHT * 0.5 + PATCH_LENGTH_INTERSECTION * 0.5);
    float roiOffsetY = (PATCH_WIDTH_INTERSECTION * 0.5 + PATCH_LENGTH_STRAIGHT);
    regionOfInterest.center = calculatePoseWithOffset(roiOffsetX, roiOffsetY, 0.f, M_PI_2);
    regionOfInterest.width = ROI_WIDTH;
    regionOfInterest.length = ROI_LENGTH * 2.5f;
    regionOfInterestList.at(0) = regionOfInterest;

    roiOffsetX = (PATCH_LENGTH_STRAIGHT * 0.5 + PATCH_LENGTH_INTERSECTION * 0.5);
    roiOffsetY = -(PATCH_WIDTH_INTERSECTION * 0.5 + PATCH_LENGTH_STRAIGHT);
    regionOfInterest.center = calculatePoseWithOffset(roiOffsetX, roiOffsetY, 0.f, -M_PI_2);
    regionOfInterest.width = ROI_WIDTH;
    regionOfInterest.length = ROI_LENGTH * 2.5f;
    regionOfInterestList.at(1) = regionOfInterest;
  }

  return regionOfInterestList;
}

std::vector<RegionOfInterest> PatchHypothesis::getROIsForIntersection(float angleRegionSize) const
{
  std::vector<RegionOfInterest> regionOfInterestList(3);
  RegionOfInterest regionOfInterest;

  if (mPatchType == Type::ROAD)
  {
    float roiOffsetX = (PATCH_LENGTH_STRAIGHT * 0.5 + PATCH_LENGTH_INTERSECTION * 0.5);
    float roiOffsetY = 0.f;
    regionOfInterest.width = PATCH_WIDTH_INTERSECTION;
    regionOfInterest.length = PATCH_LENGTH_INTERSECTION;

    regionOfInterest.center = calculatePoseWithOffset(roiOffsetX, roiOffsetY, 0.f, 0.f);
    regionOfInterestList.at(0) = regionOfInterest;

    regionOfInterest.center = calculatePoseWithOffset(roiOffsetX, roiOffsetY, angleRegionSize, 0.f);
    regionOfInterestList.at(1) = regionOfInterest;

    regionOfInterest.center = calculatePoseWithOffset(roiOffsetX, roiOffsetY, -angleRegionSize, 0.f);
    regionOfInterestList.at(2) = regionOfInterest;
  }

  return regionOfInterestList;
}


RegionOfInterest PatchHypothesis::getROIForParkingSpot() const
{
  RegionOfInterest regionOfInterest;

  if (mPatchType == Type::ROAD)
  {
    float roiOffsetX = 0.f;
    float roiOffsetY = (PATCH_WIDTH_STRAIGHT * 0.5 + PATCH_LENGTH_PARKINGSPOT * 0.5);
    regionOfInterest.width = 0.3;
    regionOfInterest.length = 0.5;

    regionOfInterest.center = calculatePoseWithOffset(roiOffsetX, roiOffsetY, 0.f, 0.f);
  }
}


OadrivePose
PatchHypothesis::calculatePoseWithOffset(float offsetX, float offsetY, float rotateOffset,
                                         float offsetYaw) const
{
  float refPatchYaw = this->getYaw() + rotateOffset;

  float rotatedOffsetX = offsetX * cos(refPatchYaw) - offsetY * sin(refPatchYaw);
  float rotatedOffsetY = offsetX * sin(refPatchYaw) + offsetY * cos(refPatchYaw);

   OadrivePose globalPose = OadrivePose(this->getX() + rotatedOffsetX,
                                             this->getY() + rotatedOffsetY,
                                             this->getYaw() + offsetYaw);


  return globalPose;
}

void PatchHypothesis::setPatchID(int id)
{
  mPatchID = id;
}

cv::Size2f PatchHypothesis::getPatchSize() const
{
  cv::Size2f patchSize;

  if (mPatchType == ROAD || mPatchType == CROSSING_ROAD)
  {
    patchSize.width = PATCH_WIDTH_STRAIGHT;
    patchSize.height = PATCH_LENGTH_STRAIGHT;
  }
  else
  {
    patchSize.width = PATCH_WIDTH_INTERSECTION;
    patchSize.height = PATCH_LENGTH_INTERSECTION;
  }

  return patchSize;
}

PatchHypothesis::DrivingDirection PatchHypothesis::getDrivingDirection() const
{
  if (mPatchType == Type::ROAD || mPatchType == Type::CROSSING_ROAD)
  {
    return DrivingDirection::STRAIGHT;
  }
  else
  {
    return mDrivingDirection;
  }
}

void PatchHypothesis::setDrivingDirection(PatchHypothesis::DrivingDirection drivingDirection)
{
  if (mPatchType == Type::INTERSECTION)
  {
    mDrivingDirection = drivingDirection;

    switch (mDrivingDirection)
    {
      case DrivingDirection::STRAIGHT:
        break;
      case DrivingDirection::LEFT:
        mPose.setYaw(mPose.getYaw() + M_PI_2);
        break;
      case DrivingDirection::RIGHT:
        mPose.setYaw(mPose.getYaw() - M_PI_2);
        break;
    }
  }
}

void PatchHypothesis::discountProbability(float factor)
{
  mProbability *= factor;
}


std::ostream &operator<<(std::ostream &os, const PatchHypothesis &patchHyp)
{
  os << "PatchHyp: " << patchHyp.getPatchType()
 //    << " | " << patchHyp.getPose()
     << " | Prob: " << patchHyp.getProbability() << std::endl;
  return os;
}

icl_core::logging::ThreadStream &
operator<<(icl_core::logging::ThreadStream &os, const PatchHypothesis &patchHyp)
{
  os << "PatchHyp: " << patchHyp.getPatchType()
 //    << " | " << patchHyp.getPose()
     << " | Prob: " << patchHyp.getProbability()
     << icl_core::logging::endl;
  return os;
}

}
}
