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
 * \author  Robin Andlauer <robin.andlauer@student.kit.edu>
 * \date    2017-7-15
 * 
 * \author  Simon Roesler <simon.roesler@student.kit.edu>
 * \date    2018
 */
//----------------------------------------------------------------------
#include "controlLogging.h"
#include <oadrive_util/Config.h>
#include <numeric>
#include <oadrive_core/Trajectory2d.h>
#include "SpeedController.h"

using namespace oadrive::core;
using namespace oadrive::control;
using icl_core::logging::endl;
using icl_core::logging::flush;

SpeedController::SpeedController()
  // : mKp((float)oadrive::util::Config::getDouble( "Driver", "Kp", 3 ))
  // , mKi((float)oadrive::util::Config::getDouble( "Driver", "Ki", 3 ))
  : mKp((float)oadrive::util::Config::getDouble( "Driver", "Kp", 26 ))
  , mKi((float)oadrive::util::Config::getDouble( "Driver", "Ki", 30 ))
  , mMaxPI_Speed((float)oadrive::util::Config::getDouble( "Driver", "MaxPI_Speed", 54 ))
//  , mMaxSpeedDifferenceSum((mKi != 0) ? mMaxPI_Speed/(2*mKi) : 1) // sum*ki must not exceed mMaxPI_Speed/2 to prevent wind-up effects
  , mMaxSpeedDifferenceSum(mMaxPI_Speed) // sum*ki must not exceed mMaxPI_Speed/2 to prevent wind-up effects
  , mSpeedDifferenceSum(0)
  , mTargetDirection(1)
  , mNumberOfAveragedValues(5)
{

}

SpeedController::~SpeedController()
{

}

float SpeedController::update(const float targetSpeed, const float steeringAngle, const bool justRoll)
{
  // get sample time
  clock_t currentTime = clock();
  float dt = ((float)(currentTime - mLastUpdateTime)) / CLOCKS_PER_SEC;
  //security check
  if(std::abs(dt) > 1.0f)
    dt = 0;
  mLastUpdateTime = currentTime;

  float currentSpeed = calculateAverageSpeed();
  if (justRoll && currentSpeed > 0.05) {
    // We are rolling, so we wont give a speed signal
    return 0.0;
  }

  // if the driving direction changed, reset Isum of PI controller to avoid a possible insane acceleration
  if(mTargetDirection * targetSpeed < 0)
  {
    mTargetDirection *= -1;
    resetSpeedDifferenceSum();
  } else if (targetSpeed == 0.0f) {
      resetSpeedDifferenceSum();
      return 0.0;
  }
  // } else if (targetSpeed == 0.0f && mTargetDirection != 0) {
  //   // We switched to stopping
  //   mTargetDirection = 0;
  //   resetSpeedDifferenceSum();
  // } else if (mTargetDirection == 0 && targetSpeed != 0) {
  //   // We switched to driving
  //   mTargetDirection = targetSpeed > 0 ? 1 : -1;
  //   resetSpeedDifferenceSum();
  // }

  float speedDifference = targetSpeed - currentSpeed;
  mSpeedDifferenceSum += speedDifference*dt;

  //std::cout << "Steering: " << steeringAngle << " target: " << targetSpeed << ", diff: " << speedDifference << ", sum: " << mSpeedDifferenceSum << ", dt: " << dt << std::endl;


  //anti wind-up
  if(mSpeedDifferenceSum > mMaxSpeedDifferenceSum)
    mSpeedDifferenceSum = mMaxSpeedDifferenceSum;
  else if(mSpeedDifferenceSum < -mMaxSpeedDifferenceSum)
    mSpeedDifferenceSum = -mMaxSpeedDifferenceSum;

  // PI controlling
  float PI_speed = mKp * speedDifference + mKi * mSpeedDifferenceSum;

  // LOGGING_INFO( controlLogger, "[DriverModule] P:" << mKp * speedDifference << " " << speedDifference << endl );
  // LOGGING_INFO( controlLogger, "[DriverModule] I:" << mKi * mSpeedDifferenceSum << " " << mSpeedDifferenceSum <<endl );

  //security check:
  if (PI_speed > mMaxPI_Speed)
    PI_speed = mMaxPI_Speed;
  if (PI_speed < -mMaxPI_Speed)
    PI_speed = -mMaxPI_Speed;

  // Precontrol (TODO: Just some dummy parameters for now):
  //float precontrol = targetSpeed * 10;
  float precontrol = 0;
  if (targetSpeed > 0) {
    precontrol = 10 + std::max(((targetSpeed - 0.5) * 2), 0.0) + std::min(10.f, std::abs(steeringAngle) / 4);
  } else {
    precontrol = -10 + std::min((targetSpeed + 0.5) * 2, 0.0) - std::min(10.f, std::abs(steeringAngle) / 4);
  }

  float controlledSpeed = 0*precontrol + PI_speed;

  // check if the controller tries to actively deccelerate
  if (mTargetDirection * controlledSpeed < 0) {
    return 0.0;
  }

  return controlledSpeed;
}

void SpeedController::addSpeedSample(const float measuredSpeed)
{
  if(measuredSpeedList.size() == mNumberOfAveragedValues)
    measuredSpeedList.erase(measuredSpeedList.begin());
  measuredSpeedList.push_back(measuredSpeed);
}

float SpeedController::calculateAverageSpeed()
{
  return std::accumulate(measuredSpeedList.begin(),measuredSpeedList.end(),0.0f) / measuredSpeedList.size();
}

void SpeedController::resetSpeedDifferenceSum()
{
  mSpeedDifferenceSum = 0;
}
