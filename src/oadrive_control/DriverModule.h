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

#ifndef OADRIVE_CONTROL_DRIVERMODULE_H
#define OADRIVE_CONTROL_DRIVERMODULE_H

#include <oadrive_core/Trajectory2d.h>
#include <oadrive_control/LateralController.h>
#include <oadrive_control/SpeedController.h>
#include <oadrive_util/Timer.h>
#include <oadrive_world/MultiTrajectory.h>
#include <oadrive_obstacle/ObjectTracker.h>

#include <boost/thread/mutex.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

// set to enable active braking
#define ACTIVE_BRAKING false

using namespace oadrive::core;

namespace oadrive {
namespace control {

/*!
 * \brief The DriverModule handles communication between the lateral controller, the speed controller and the controller ros node
*/
class DriverModule
{
public:
  DriverModule();
  ~DriverModule();
  //!
  //! \brief setTrajectory Set new trajectory. Is reached will be reset.
  //! \param traj trajectory
  //! \return true if initialized correctly, false otherwise.
  //!
  bool setMultiTrajectory(const oadrive::world::MultiTrajectory &traj );
  //!
  //! \brief halt force stop of the car.
  //! \param if active brake is true, the controller will actively break (negative speed value) instead of setting the controller output to 0 only.
  //!  However, this decreases the accuracy of the estimation of the car position (Car pose estimation assumes the car to drive
  //!  backwards slightly which is not the case in reality)!!!
  //!
  void halt(const bool &active_brake);
  //! \brief enable driving
  void drive();
  ///!
  //! \brief update Calls lateral and speed controller to determine speed and steering output of the car!
  //! \param pose postion of the car
  //!
  void update( const ExtendedPose2d &pose );

  //!
  //! \brief getSteeringAngle get steering angle it is calculate in udate
  //! \return steering angle in degrees. (0° is straight forward)
  //!
  float getSteeringAngle() const;
  //!
  //! \brief getSpeed get driving speed
  //! \return speed
  //!
  float getSpeed() const;
  //!
  //! \brief setTargetSpeed set the maximum speed for the controler
  //! \param speed speed
  //!
  void setMaxSpeed(float speed);

  //!
  //! \brief enableCarFollowing give controller the car to follow. The controller will set the speed accordingly to keep a desired distance
  //! \brief WARNING: Never tested
  //! \param trackedCar to follow
  //!
  void enableCarFollowing(oadrive::obstacle::TrackedCar trackedCar);
  void disableCarFollowing();

  /*! Return maximum angle which will ever be returned by getSteering(). */
  float getMaxSteeringAngle() { return mMaxSteering; }

  /*! Checks if the end of the trajectory has been reached.*/
  bool checkEndOfTrajectoryFlag();

  /*! Find out which part of the MultiTrajectory the driver is currently on. */
  size_t getCurrentTrajectoryIndex();

  /*! Get number of left trajectory points */
  size_t getNumberOfRemainingTrajectoryPoints();
  size_t getNumberOfRemainingTrajectories();

  /*! Resets the Driver */
  void reset();

private:
  /*! set new trajectory */
  bool setTrajectory( const Trajectory2d &traj );
  float calculateTargetSpeed(const ExtendedPose2d &carPose);
  float calculateTrajectorySpeed();
  float calculateFollowCarSpeed(const ExtendedPose2d &egoCarPose);

  // maxSteering is required for both lateral and speed controller
  const float mMaxSteering;
  // current point id of the trajectory
  size_t mCurrentTrajectoryIndex;
  float mCurrentSteering;
  // desired speed that the speed controller aims to achieve
  float mDesiredSpeed;
  // current speed given by the speed controller to achiev desired speed
  float mTargetSpeed;
  // desired speed if we are driving without limitations like too sharp curves (see SpeedController::calculateSpeed)
  float mMaxSpeed;

  // used to determine wether isum has to be resetted (1 for forward driving, -1 for backward driving)
  bool mDriving;
  bool mBraking = false;
  bool mInitializedTrajectory;
  bool mTrajectoryEndReached;

  bool mCarFollowing = false;
  oadrive::obstacle::TrackedCar mTrackedCar;


  // desired distance to follow car
  const float mCarFollowingDistance;
  // P-controller
  const float mCarFollowingP;
  // cap speed controller for car following
  const float mCarFollowingMaxSpeed;

  LateralController mLateralController;

  oadrive::world::MultiTrajectory mMultiTrajectory;
  Trajectory2d mTrajectory;

  // Speed calc options
  // if steering is greater than mMinSteeringToCap, the speed is capped/reduced so the car slows down when driving curves
  const float mMinSteeringToCap;
  // recommended speed for mMaxSteering steering angle. Needed for speed calculation for curves
  const float mMaxSteeringSpeed;
  // recommended speed for mMinSteeringToCap steering angle
  const float mMinSteeringSpeed;
  // precalculated ratio between min and max steering speed and min and max steering
  // used in calculation of speed according to steering angle
  const float mRatioMinMaxSteeringSpeedToMinMaxSteering;

public:
  // use a proper alignment when calling the constructor.
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

}	//namespace
}	//namespace

#endif // include guard
