// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// This program is free software licensed under the CDDL
// (COMMON DEVELOPMENT AND DISTRIBUTION LICENSE Version 1.0).
// You can find a copy of this license in LICENSE in the top
// directory of the source code.
//
// © Copyright 2016 FZI Forschungszentrum Informatik, Karlsruhe, Germany
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Micha Pfeiffer <ueczz@student.kit.edu>
 * \date    2015-12-04
 *
 */
//----------------------------------------------------------------------

#include "DriverModule.h"
#include <iostream>
#include <iomanip>
#include "controlLogging.h"
#include <oadrive_util/Config.h>
#include <oadrive_util/Broker.h>
using icl_core::logging::endl;
using icl_core::logging::flush;
using namespace oadrive::world;

namespace oadrive {
namespace control {


DriverModule::DriverModule()
  : mLateralController()
  , mCurrentTrajectoryIndex( 0 )
  , mCurrentPose( 0, 0, 0 )
  , mMaxSteering( (float)oadrive::util::Config::getDouble( "Driver", "MAX_STEERING", 29.9 ) )
  , mMinSteering( (float)oadrive::util::Config::getDouble( "Driver", "MIN_STEERING", 10 ) )
  , mCurrentSteering(0.0)
  , mCurrentSpeed(0)
  , mTargetSpeed(0)
  , mMaxSteeringSpeed((float)oadrive::util::Config::getDouble( "Driver", "MAX_STEERING_SPEED", 0.15 ))
  , mDriving(false)
  , mInitializedTrajectory(false)
  , mTrajectoryEndReached(false)
  , mLastImageStamp(boost::posix_time::microsec_clock::local_time())
  , mLowFrameRate(false)
{

  double SPEED_NORMAL = oadrive::util::Config::getDouble( "MissionControl", "SPEED_NORMAL", 0.6 );

   mMinSteeringSpeed = (float)oadrive::util::Config::getDouble( "Driver", "MIN_STEERING_SPEED", SPEED_NORMAL );
   mRatioMinMaxSteeringSpeedToMinMaxSteering = (mMaxSteeringSpeed - mMinSteeringSpeed) / (mMaxSteering - mMinSteering) ;
}


DriverModule::~DriverModule()
{
}

///////////////////////////////////////////////////////////////
// Public functions (mutex locked!)
// A public function in the DriverModule must NEVER call another
// public function, otherwise they will end up in a dead lock.
// Instead, call private functions.

bool DriverModule::setTrajectory( const MultiTrajectory& traj )
{
  mtx.lock();
  mTrajectoryEndReached = false;
  mInitializedTrajectory = false;
  LOGGING_INFO( controlLogger, "[DriverModule] New MultiTrajectory received: " << endl
		  << "\t(" << traj.trajectories.size() << " trajectories)" << endl );
  mMultiTrajectory = traj;
  if( traj.trajectories.size() > 0 )
  {

    mCurrentTrajectoryIndex = 0;
    bool result = setTrajectory( mMultiTrajectory.trajectories[mCurrentTrajectoryIndex] );
    mtx.unlock();
    return result;
  }
  mtx.unlock();
  return false;
}

void DriverModule::reset()
{
  mtx.lock();
  privateHalt();
  mTrajectoryEndReached = false;
  mInitializedTrajectory = false;
  mMultiTrajectory = MultiTrajectory();
  mTrajectory = Trajectory2d ();
  mLateralController.setTrajectory( mTrajectory );
  mCurrentTrajectoryIndex = 0;
  mtx.unlock();

}


void DriverModule::halt()
{
  mtx.lock();
  privateHalt();
  mtx.unlock();
}
void DriverModule::drive()
{
  mtx.lock();
  privateDrive();
  mtx.unlock();
}

//FEATURE check if we are to far from trajectory
void DriverModule::update( const ExtendedPose2d &carPose )
{
  if(mtx.try_lock() == false)
  {
    LOGGING_ERROR( controlLogger, "Locking failed @ DriverModule::update " << endl );
    return;
  }

  //check if framerate is down....
  boost::posix_time::time_duration frameDiff = boost::posix_time::microsec_clock::local_time() - mLastImageStamp;
  if(frameDiff.total_milliseconds() > 500) {
      //slow down if framerate is low
      mLowFrameRate = true;
      LOGGING_INFO( controlLogger, "Framerate is dropping... slow motion!" << endl );
  }

  mCurrentPose = carPose;
  if( mDriving && mInitializedTrajectory )
  {
    mCurrentSteering = mLateralController.calculateSteering( mCurrentPose.getPose() );

    // The car has a max steering angle of a little over mMaxSteering.
    // Make sure this angle is never reached, to avoid annoying warnings:
    if(mCurrentSteering >= mMaxSteering)
    {
      mCurrentSteering = mMaxSteering;
    }
    else if(mCurrentSteering <= -mMaxSteering)
    {
      mCurrentSteering = -mMaxSteering;
    }

    if(mTrajectory.size() > 2){

      // calculate speed according to trajectory
      float recommendedTrajectorySpeed;
      unsigned int curTrajIndex = mLateralController.getIndexOfProjectedPose();

      if(curTrajIndex < mTrajectory.size()-1){

        double speed1 = mTrajectory[curTrajIndex].getVelocity();
        double speed2 = mTrajectory[curTrajIndex+1].getVelocity();
        double ratio = mLateralController.getRatioToNextPoint();

        recommendedTrajectorySpeed = (1.0-ratio) * speed1 + ratio*speed2;
      }
      else{
        recommendedTrajectorySpeed = mTrajectory.back().getVelocity();
      }

      // calculate speed according to steering angle
      float recommendedSteeringSpeed = FLT_MAX;
      float absCurrentSteering = std::abs(mCurrentSteering);

      // if steering angle is big enough, calculate speed according to steering angle
      if(absCurrentSteering > mMinSteering /*&& !oadrive::util::Broker::isActive()*/) {
        recommendedSteeringSpeed = (mMaxSteering - absCurrentSteering) * mRatioMinMaxSteeringSpeedToMinMaxSteering + mMinSteeringSpeed;
      }

      // actual driving speed is the minimum of trajectory and steering speed
      mCurrentSpeed = std::min(recommendedTrajectorySpeed, recommendedSteeringSpeed);
      if(mLowFrameRate) {
        mCurrentSpeed = std::min(0.16f, mCurrentSpeed);
        std::cout << "WARNING: FRAME RATE IS VERY LOW! (Bad Camera Update rate?)" << std::endl;
      }

      // error check for NaN speed
      if(mCurrentSpeed != mCurrentSpeed){
        mCurrentSpeed = mTargetSpeed;
      }

      if(!mTrajectory.isForwardTrajectory())
      {
        mCurrentSpeed = -mCurrentSpeed;
      }

      LOGGING_INFO( controlLogger, "[DriverModule] New speed: " << mCurrentSpeed << endl );

    }

    if(mLateralController.hasReached())
    {
      mInitializedTrajectory = false;
      // If we're driving a multi trajectory and still have more trajectories to go:
      if( mMultiTrajectory.trajectories.size() > mCurrentTrajectoryIndex + 1 )
      {
        mCurrentTrajectoryIndex ++;
        setTrajectory( mMultiTrajectory.trajectories[mCurrentTrajectoryIndex] );
      } else {
        // Otherwise let the mission control know we're done.
        LOGGING_INFO( controlLogger, "[DriverModule] Reached end of trajectory." << endl);
        mTrajectoryEndReached = true;
      }
    }
  }
  else
  {
    LOGGING_INFO( controlLogger, "[DriverModule] NOT DRIVING !!!!" << endl);
    mCurrentSpeed = 0;
  }
  mtx.unlock();
}

float DriverModule::getSteeringAngle()
{
  float tmp;
  mtx.lock();
  tmp = mCurrentSteering;
  mtx.unlock();
  return tmp;
}

float DriverModule::getSpeed()
{
  float tmp;
  if(mtx.try_lock() == false)
  {
    LOGGING_ERROR( controlLogger, "Locking failed @ DriverModule::getSpeed " << endl );
    return 0;
  }
  LOGGING_INFO( controlLogger, "[DriverModule] New target speed: " << mCurrentSpeed << endl );
  tmp = mCurrentSpeed;

  mtx.unlock();
  return tmp;
}

void DriverModule::setTargetSpeed(float speed)
{
  mtx.lock();
  LOGGING_INFO( controlLogger, "[DriverModule] New target speed: "
      << speed << endl );
  mTargetSpeed = speed;
  mtx.unlock();
}

float DriverModule::getTargetSpeed()
{
  float speed;
  mtx.lock();
  speed = mTargetSpeed;
  mtx.unlock();
  return speed;
}

ExtendedPose2d DriverModule::getCarPose()
{
  ExtendedPose2d tmp;
  mtx.lock();
  tmp = mCurrentPose;

  boost::posix_time::time_duration frameDiff = boost::posix_time::microsec_clock::local_time() - mLastImageStamp;
  mLastImageStamp = boost::posix_time::microsec_clock::local_time();

  if(mLowFrameRate && frameDiff.total_milliseconds() < 300)
  {
      //Framerate is ok again
      mLowFrameRate = false;

      LOGGING_INFO( controlLogger, "Framerate is good again!" << endl );
  }
  mtx.unlock();
  return tmp;
}

bool DriverModule::checkEndOfTrajectoryFlag()
{
  bool val = false;
  mtx.lock();
  // Check if the end was reached:
  val = mTrajectoryEndReached;

  // Reset so that we don't get the event twice!
  mTrajectoryEndReached = false;

  mtx.unlock();
  return val;
}

size_t DriverModule::getCurrentTrajectoryIndex()
{
  size_t index;
  mtx.lock();
  index = mCurrentTrajectoryIndex;
  mtx.unlock();
  return index;
}

///////////////////////////////////////////////////////////////
// Private functions (not mutex locked)

bool DriverModule::setTrajectory( const Trajectory2d& traj )
{
  if( traj.size() >= mLateralController.getMinTrajPoints() )
  {
    mTrajectory = traj;
    // Calculate directions from one pose to the next:
    mTrajectory.calculateOrientations();
    // Calculate the radii of the circles at every trajectory pose:
    mTrajectory.calculateCurvature();

    mLateralController.setTrajectory( mTrajectory );

    mInitializedTrajectory = true;

    LOGGING_INFO( controlLogger, "[DriverModule] Trajectory received: " << endl
        << "\t is Forward " << (mTrajectory.isForwardTrajectory())
        << endl );

    // Update again to see if we've reached the end of this trajectory:
    mLateralController.calculateSteering( mCurrentPose.getPose() );

    // Check if this trajectory is drivable:
    if(mLateralController.hasReached())
    {
      mInitializedTrajectory = false;   // Stops driving
      // If we're driving a multi trajectory and still have more trajectories to go:
      if( mMultiTrajectory.trajectories.size() > mCurrentTrajectoryIndex + 1 )
      {
        mCurrentTrajectoryIndex ++;
        //does yet not make totally sense, since we only check the first trajectory at first and only if it has reached it's end, we check the next one (and we dont check whether we are at the end of the last trajectory in the multi trajectory)
        mInitializedTrajectory = setTrajectory(
            mMultiTrajectory.trajectories[mCurrentTrajectoryIndex] );
      } else {
        // Otherwise let the mission control know we're done.
        LOGGING_INFO( controlLogger, "[DriverModule] Reached end of trajectory." << endl);
        mTrajectoryEndReached = true;
        }
    }
  } else {
    LOGGING_WARNING( controlLogger, "[DriverModule] WARNING: Trajectory has less than " << 
        mLateralController.getMinTrajPoints() << " points. Halting." << endl );
    mInitializedTrajectory = false;
    mTrajectoryEndReached = true;
  }
  return mInitializedTrajectory;
}

void DriverModule::privateHalt()
{
  LOGGING_INFO( controlLogger, "[DriverModule] Halting." << endl );
  mDriving = false;
  mCurrentSpeed = 0;
}
void DriverModule::privateDrive()
{
  LOGGING_INFO( controlLogger, "[DriverModule] Driving." << endl );
  mDriving = true;
}

}	// namespace
}	// namespace
