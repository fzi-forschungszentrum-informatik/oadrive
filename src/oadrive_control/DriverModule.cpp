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
 * \date    2015-12-04
 *
 * \author  Robin Andlauer <robin.andlauer@student.kit.edu>
 * \date    2017-7-01
 * 
 * \author  Simon Roesler <simon.roesler@student.kit.edu>
 * \date    2018
 */
//----------------------------------------------------------------------

#include "DriverModule.h"
#include <iostream>
#include <iomanip>
#include "controlLogging.h"
#include <oadrive_util/Config.h>
//#include <oadrive_util/Broker.h>
using icl_core::logging::endl;
using icl_core::logging::flush;
using namespace oadrive::world;

namespace oadrive {
namespace control {

DriverModule::DriverModule()
  : mMaxSteering((float)oadrive::util::Config::getDouble("Driver", "MaxSteering", 29.9))
  , mCurrentTrajectoryIndex(0)
  , mCurrentSteering(0.0)
  , mTargetSpeed(0)
  , mMaxSpeed(1.0)
  , mDriving(false)
  , mBraking(false)
  , mInitializedTrajectory(false)
  , mTrajectoryEndReached(false)
  , mCarFollowingDistance((float)oadrive::util::Config::getDouble("Driver", "CarFollowingDistance", 1.2))
  , mCarFollowingP((float)oadrive::util::Config::getDouble("Driver", "CarFollowingP", 0.2))
  , mCarFollowingMaxSpeed((float)oadrive::util::Config::getDouble("Driver", "CarFollowingMaxSpeed", 0.5))
  , mLateralController(mMaxSteering)
  , mMinSteeringToCap((float)oadrive::util::Config::getDouble("Driver", "MinSteeringToCap", 10))
  , mMaxSteeringSpeed((float)oadrive::util::Config::getDouble("Driver", "MaxSteeringSpeed", 0.6))
  , mMinSteeringSpeed((float)oadrive::util::Config::getDouble("Driver", "MinSteeringSpeed", 0.4))
  , mRatioMinMaxSteeringSpeedToMinMaxSteering((mMaxSteeringSpeed - mMinSteeringSpeed) / (mMaxSteering - mMinSteeringToCap))
{
}

DriverModule::~DriverModule()
{
}

///////////////////////////////////////////////////////////////
// Public functions

bool DriverModule::setMultiTrajectory(const MultiTrajectory& traj)
{
  mTrajectoryEndReached = false;
  mInitializedTrajectory = false;

  mMultiTrajectory = traj;
  if (traj.trajectories.size() > 0)
  {
    mCurrentTrajectoryIndex = 0;
    bool result = setTrajectory(mMultiTrajectory.trajectories[mCurrentTrajectoryIndex]);
    return result;
  }
  return false;
}

void DriverModule::reset()
{
  halt(false);
  mTrajectoryEndReached = false;
  mInitializedTrajectory = false;
  mMultiTrajectory = MultiTrajectory();
  mTrajectory = Trajectory2d ();
  mLateralController.setTrajectory(mTrajectory);
  mCurrentTrajectoryIndex = 0;
}

void DriverModule::halt(const bool &active_brake)
{
  mDriving = false;
  #if ACTIVE_BRAKING
  mBraking = active_brake;
  #endif
}

void DriverModule::drive()
{
  mDriving = true;
  mBraking = false;
}

void DriverModule::update(const ExtendedPose2d &carPose)
{
  // lateral controlling:
  if (mInitializedTrajectory)
  {
    mCurrentSteering = mLateralController.calculateSteering(carPose.getPose());

    // check if traj has finished
    if (mLateralController.hasReached())
    {
      mInitializedTrajectory = false;
      // If we're driving a multi trajectory and still have more trajectories to go:
      if(mMultiTrajectory.trajectories.size() > mCurrentTrajectoryIndex + 1)
      {
        mCurrentTrajectoryIndex ++;
        setTrajectory(mMultiTrajectory.trajectories[mCurrentTrajectoryIndex]);
      } 
      else 
      {
        // Otherwise let the mission control know we're done.
        LOGGING_INFO(controlLogger, "[DriverModule] Reached end of trajectory." << endl);
        mTrajectoryEndReached = true;
#if ACTIVE_BRAKING
        // Uncomment if you want to brake after trajectory was reached
        mBraking = true;
#endif
      }
    }
  }

  mTargetSpeed = mDriving ? calculateTargetSpeed(carPose) : 0.0;
}

float DriverModule::calculateTargetSpeed(const ExtendedPose2d &carPose) 
{
  if(mBraking) {
    // TODO mCurrentDirection * carPose.getVelocity() <= 0.0f ||  would improve it
    if(fabs(carPose.getVelocity()) < 0.05f) {
      mBraking = false;
    }

    return 0.0;
  } else {
    // calculate desired speed: Call calculate speed or follow car
    if(mCarFollowing)
    {
      return calculateFollowCarSpeed(carPose);
    }
    else if (mInitializedTrajectory)
    {
      return calculateTrajectorySpeed();
    }
  }

  return 0.0;
}

float DriverModule::calculateTrajectorySpeed()
{
  // calculate recommended speed given by trajectory
  // float recommendedTrajectorySpeed = FLT_MAX;
  // if(mTrajectory.velocityAvailable())
  // {
  //   unsigned int currentTrajIndex = mLateralController.getIndexOfProjectedPose();
  //   if(currentTrajIndex < mTrajectory.size()-1)
  //   {
  //     double speed1 = mTrajectory[currentTrajIndex].getVelocity();
  //     double speed2 = mTrajectory[currentTrajIndex+1].getVelocity();
  //     double ratio = mLateralController.getRatioToNextPoint();
  //     recommendedTrajectorySpeed = (1.0-ratio) * speed1 + ratio*speed2;
  //   }
  //   else
  //   {
  //     recommendedTrajectorySpeed = mTrajectory.back().getVelocity();
  //   }
  // }

  // // calculate the recommended speed if the steering angle exceeds mMinSteeringToCap
  // float recommendedCurveSpeed = FLT_MAX;
  // if(std::abs(mCurrentSteering) > mMinSteeringToCap)
  // {
  //   recommendedCurveSpeed = (mMaxSteering - std::abs(mCurrentSteering)) * mRatioMinMaxSteeringSpeedToMinMaxSteering + mMinSteeringSpeed;
  // }

  // // final speed value is the minimum of the three recommended speed values
  // float recommendedSpeed =  std::min(std::min(recommendedTrajectorySpeed, recommendedCurveSpeed), mMaxSpeed);

  float recommendedSpeed = mMaxSpeed;
  // invert speed if we drive backwards
  if(!mTrajectory.isForwardTrajectory())
  {
    recommendedSpeed = -recommendedSpeed;
  }
  return recommendedSpeed;
}

float DriverModule::calculateFollowCarSpeed(const ExtendedPose2d &egoCarPose)
{
  const ExtendedPose2d &trackedCarPose = mTrackedCar.object.pose;
  const ExtendedPose2d &trackedCarSpeed = mTrackedCar.speed;

  // Pre controll
  float absTrackedCarSpeed = sqrt(trackedCarSpeed.getX()*trackedCarSpeed.getX()
                                  +trackedCarSpeed.getY()*trackedCarSpeed.getY());
  // P controller
  float dx = trackedCarPose.getX() - egoCarPose.getX();
  float dy = trackedCarPose.getY() - egoCarPose.getY();
  float distanceBetweenCars = sqrt(dx*dx+dy*dy);
  float recommendedSpeed = absTrackedCarSpeed + mCarFollowingP * (distanceBetweenCars - mCarFollowingDistance);

  //security check / speed cap
  if(recommendedSpeed > mCarFollowingMaxSpeed)
      recommendedSpeed = mCarFollowingMaxSpeed;
  if(recommendedSpeed < 0)
      recommendedSpeed = 0;
  std::cout<< "Car following: " << recommendedSpeed << std::endl;
  return recommendedSpeed;
}

float DriverModule::getSteeringAngle() const
{
  return mCurrentSteering;
}

float DriverModule::getSpeed() const
{
  return mTargetSpeed;
}

void DriverModule::setMaxSpeed(float speed)
{
  mMaxSpeed = speed;
}

void DriverModule::enableCarFollowing(oadrive::obstacle::TrackedCar trackedCar)
{
  mCarFollowing = true;
  mBraking = false;
  mTrackedCar = trackedCar;
}

void DriverModule::disableCarFollowing()
{
  mCarFollowing = false;
}

bool DriverModule::checkEndOfTrajectoryFlag()
{
  return mTrajectoryEndReached;
}

size_t DriverModule::getCurrentTrajectoryIndex()
{
  return mCurrentTrajectoryIndex;
}

size_t DriverModule::getNumberOfRemainingTrajectoryPoints()
{
  return mTrajectory.size() - mLateralController.getIndexOfProjectedPose();
  // return mTrajetocry.size()
  // return mLateralController.getIndexOfProjectedPose()
}

size_t DriverModule::getNumberOfRemainingTrajectories()
{
  return mMultiTrajectory.trajectories.size() - mCurrentTrajectoryIndex;
}

///////////////////////////////////////////////////////////////
// Private functions
bool DriverModule::setTrajectory(const Trajectory2d& traj)
{
  if(traj.size() >= mLateralController.getMinTrajPoints())
  {
    mTrajectory = traj;
    // Calculate directions from one pose to the next:
    mTrajectory.calculateOrientations();
    // Calculate the radii of the circles at every trajectory pose:
    mTrajectory.calculateCurvature();

    mLateralController.setTrajectory(mTrajectory);

    mInitializedTrajectory = true;
/*
    LOGGING_INFO(controlLogger, "[DriverModule] Trajectory received: " << endl
        << "\t is Forward " << (mTrajectory.isForwardTrajectory())
        << endl);
*/
  } else {
    LOGGING_WARNING(controlLogger, "[DriverModule] WARNING: Trajectory has less than " << 
        mLateralController.getMinTrajPoints() << " points. Halting." << endl);
    mInitializedTrajectory = false;
    mTrajectoryEndReached = true;
  }
  return mInitializedTrajectory;
}
}	// namespace
}	// namespace
