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
 * \author  David Zimmerer <dzimmerer@gmail.com>
 * \author  Micha Pfeiffer <ueczz@student.kit.edu>
 * \date    2015-12-23
 * 
 * \author  Simon Roesler <simon.roesler@student.kit.edu>
 * \date    2018
 * 
 * \author  Mark Hueneberg <hueneber@fzi.de>
 * \date    2018
 *
 */
//----------------------------------------------------------------------

#include "Patch.h"
#include <iostream>
#include <limits>
#include <sstream>
#include <iomanip>
#include "TrajectoryDatabase.h"
#include "TrajectoryFactory.h"
#include "oadrive_util/Config.h"

using namespace oadrive::core;

#include "worldLogging.h"

using icl_core::logging::endl;
using icl_core::logging::flush;

namespace oadrive
{
namespace world
{

Patch::Patch(PatchType type, ExtendedPose2d pose) :
        EnvObject(pose, PATCH_WIDTHS[type], PATCH_LENGTHS[type]),
        mOriginalPose(pose.getX(), pose.getY(), pose.getYaw()),
        mPatchType(type),
        mAction(DD_STRAIGHT),
        mNumAngleRegions(4),
        mHasTrafficSign(false),
        mSwitch(false),
        mIsFixed(false)
{

  // Ugly Fix because we get too many
  if (type == PARKING)
  {
    setWidth(PATCH_WIDTHS[type] - 0.10);
    setHeight(PATCH_LENGTHS[type] - 0.10);
  }

  setType(PATCH);
  //updateTrajectory();

  mPoseVotes.push_back(pose);

  mLane = LANE_RIGHT;
  mMagicStreetOffset = oadrive::util::Config::getDouble("Magic", "MagicStreetOffset", 0.02);

  #ifdef EXP_MOVING_AVERAGE_PATCH_POSE
  // init moving avg
  mExpMovingAvgPose.x = this->getX();
  mExpMovingAvgPose.y = this->getY();
  mExpMovingAvgPose.yaw = this->getYaw();
  #endif
}

Patch::~Patch() {}

void Patch::mergeFrom(const PatchPtr newPatch) 
{
  // EnvObject merges position and some other stuff
  EnvObject::mergeFrom(newPatch);
  this->mPatchType = newPatch->mPatchType;
  this->mAction = newPatch->mAction;
  
  #ifdef EXP_MOVING_AVERAGE_PATCH_POSE
    const double alpha = EXP_MOVING_AVERAGE_PATCH_POSE;
    auto newPose = this->getPose();

    mExpMovingAvgPose.x = alpha * newPose.getX() + (1 - alpha) * mExpMovingAvgPose.x;
    mExpMovingAvgPose.y = alpha * newPose.getY() + (1 - alpha) * mExpMovingAvgPose.y;
    mExpMovingAvgPose.yaw = alpha * newPose.getYaw() + (1 - alpha) * mExpMovingAvgPose.yaw;

    newPose.setX(mExpMovingAvgPose.x);
    newPose.setY(mExpMovingAvgPose.y);
    newPose.setYaw(mExpMovingAvgPose.yaw);

    this->setPose(newPose);
  #endif
}

Trajectory2d* Patch::getTrajectoryFromMC(ExtendedPose2d &startPose, LaneType lane)
{
  return getTrajectory(startPose, mAction, lane);
}

Trajectory2d* Patch::getTrajectory(ExtendedPose2d &startPose, drivingDirection dd, LaneType lane)
{
  ExtendedPose2d center = getPose();
  double angle = center.getYaw();

  // UnitX points into the direction of the patch, UnitY points towards the left of the patch:
  Position2d unitX, unitY;
  unitX(0) = cos(angle);
  unitX(1) = sin(angle);
  unitY(0) = cos(angle + M_PI * 0.5);
  unitY(1) = sin(angle + M_PI * 0.5);

  mCurrentTrajectory.clear();

  if (mPatchType == STRAIGHT) {

    if (mLane == LANE_RIGHT) {
      Position2d lanePoint = center.getPosition() - unitY * (STREET_SIDE_TO_MID_LANE);
      mCurrentTrajectory.push_back(ExtendedPose2d(lanePoint[0], lanePoint[1], angle));
    }
    else if (mLane == LANE_LEFT) {
      Position2d lanePoint = center.getPosition() + unitY * 1.5 * (STREET_SIDE_TO_MID_LANE);
      mCurrentTrajectory.push_back(ExtendedPose2d(lanePoint[0], lanePoint[1], angle + M_PI));
    }
    else if (mLane == RESCUE_LANE) {
      Position2d lanePoint = center.getPosition() - unitY * 2.5 * (STREET_SIDE_TO_MID_LANE);
      mCurrentTrajectory.push_back(ExtendedPose2d(lanePoint[0], lanePoint[1], angle));
    }
    else if (mLane == SWITCH_TARGET_LANE) {
      Position2d lanePoint = center.getPosition() + unitY * 2 * (STREET_SIDE_TO_MID_LANE);
      mCurrentTrajectory.push_back(ExtendedPose2d(lanePoint[0], lanePoint[1], angle + M_PI));
    }
    else if (mLane == SWITCH_BACK_TARGET_LANE) {
      Position2d lanePoint = center.getPosition() - 2 * unitY * (STREET_SIDE_TO_MID_LANE);
      mCurrentTrajectory.push_back(ExtendedPose2d(lanePoint[0], lanePoint[1], angle));
    }
    else if (mLane == LANE_CENTER) {
      Position2d lanePoint = center.getPosition();
      mCurrentTrajectory.push_back(ExtendedPose2d(lanePoint[0], lanePoint[1], angle));
    }
  }
  else if (mPatchType == CROSS_SECTION)
  {
    float yOffset = -STREET_SIDE_TO_MID_LANE;
    
    if (mLane == LANE_LEFT) {
      yOffset = 1.5 * STREET_SIDE_TO_MID_LANE;
    } else if (mLane == SWITCH_TARGET_LANE) {
      yOffset = 2 * STREET_SIDE_TO_MID_LANE;
    } else if (mLane == LANE_CENTER) {
      yOffset = 0;
    }
    // Position2d startpoint = center.getPosition()
    //                         - unitX * (PATCH_LENGTHS[CROSS_SECTION] * 0.5)
    //                         - unitY * (STREET_SIDE_TO_MID_LANE + 0.01);

    // Position2d startpoint = center.getPosition() - unitX * (PATCH_LENGTHS[CROSS_SECTION] * 0.5) - unitY * STREET_SIDE_TO_MID_LANE;
    Position2d startpoint = center.getPosition() - unitX * (PATCH_LENGTHS[CROSS_SECTION] * 0.5) + unitY * yOffset;


    ExtendedPose2d start(startpoint[0], startpoint[1], angle);

    if (mAction == drivingDirection::DD_STRAIGHT)
    {
      Trajectory2d trajCrossStraight =
              TrajectoryFactory::getJunctionTrajectory(TurnDirection::STRAIGHT_TURN, mLane);
      mCurrentTrajectory = rotateAndMoveTrajectory(trajCrossStraight, start);
    }
    else if (mAction == drivingDirection::DD_LEFT)
    {
      // Trajectory2d trajCrossLeft = TrajectoryDatabase::getSingleTrajectory("cross_left");
      Trajectory2d trajCrossLeft = TrajectoryFactory::getJunctionTrajectory(TurnDirection::LEFT_TURN, mLane);
      mCurrentTrajectory = rotateAndMoveTrajectory(trajCrossLeft, start);
    }
    else if (mAction == drivingDirection::DD_RIGHT)
    {
      // Trajectory2d trajCrossRight = TrajectoryDatabase::getSingleTrajectory("cross_right");
      Trajectory2d trajCrossRight = TrajectoryFactory::getJunctionTrajectory(TurnDirection::RIGHT_TURN, mLane);
      mCurrentTrajectory = rotateAndMoveTrajectory(trajCrossRight, start);
    }
  }
  return &mCurrentTrajectory;
}


Trajectory2d Patch::rotateAndMoveTrajectory(Trajectory2d &traj, ExtendedPose2d pose)
{
  Trajectory2d rotated;
  for (size_t i = 0; i < traj.size(); i++)
  {
    // Rotate the trajecotry point acoording to "pose"'s yaw:
    double x = traj[i].getX() * cos(pose.getYaw()) - traj[i].getY() * sin(pose.getYaw());
    double y = traj[i].getX() * sin(pose.getYaw()) + traj[i].getY() * cos(pose.getYaw());

    // Append position:
    ExtendedPose2d newPose(pose.getX() + x, pose.getY() + y, pose.getYaw() + traj[i].getYaw());
    rotated.push_back(newPose);

  }
  rotated.isForwardTrajectory() = traj.isForwardTrajectory();
  return rotated;
}

void Patch::setPose(const ExtendedPose2d &pose)
{
  EnvObject::setPose(pose);
  //updateTrajectory();
}

PatchPtr Patch::getSuccessor()
{
  return mSuccessor;
}

void Patch::setSuccessor(PatchPtr successor)
{
  mSuccessor = successor;
}


bool Patch::setAction(drivingDirection dir)
{

  if (dir != DD_STRAIGHT && mPatchType != CROSS_SECTION)
  {
    return false;
  }

  mAction = dir;

  return true;
}

drivingDirection Patch::getAction()
{
  return mAction;
}

void Patch::removeSuccessor()
{
  mSuccessor = nullptr;
}

bool sortByAngle(const ExtendedPose2d &p1, const ExtendedPose2d &p2)
{
  return p1.getYaw() < p2.getYaw();
}

bool Patch::isSwitch()
{
  return mSwitch;
}

void Patch::setSwitch(bool is_switch)
{
  mSwitch = is_switch;
}

LaneType Patch::getLane()
{
  return mLane;
}

void Patch::setLane(LaneType lane)
{
  mLane = lane;
}

void Patch::setRecommendedLane(LaneType lane)
{
  mRecommendedLane = lane;
}

LaneType Patch::getRecommendedLane() {
  return mRecommendedLane;
}

std::string Patch::toJson()
{

  std::stringstream sstr;
  sstr << std::setprecision(2);
  sstr << std::fixed;
  sstr << "{" <<
       "\"type\": \"patch\"," <<
       "\"id\": " << this->getId() << "," <<
       "\"patchType\": " << this->getPatchType() << "," <<
       "\"x\": " << this->getX() << "," <<
       "\"y\": " << this->getY() << "," <<
       "\"yaw\": " << this->getYaw() <<
       "} ";
  return sstr.str();

}

void Patch::setPatchID(int patchID)
{
  mPatchID = patchID;
}

int Patch::getPatchID()
{
  return mPatchID;
}

void Patch::connectTrafficSign(const boost::shared_ptr<TrafficSign> trafficSign)
{
  assert(mPatchType == CROSS_SECTION);

  mTrafficSign = trafficSign;
  mHasTrafficSign = true;
}

const TrafficSignPtr Patch::getTrafficSign()
{
  return mTrafficSign;
}

bool Patch::hasTrafficSign()
{
  return mHasTrafficSign;
}

}  // namespace
}  // namespace

