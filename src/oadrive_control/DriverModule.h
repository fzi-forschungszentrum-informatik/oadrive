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

#ifndef OADRIVE_CONTROL_DRIVERMODULE_H
#define OADRIVE_CONTROL_DRIVERMODULE_H

#include <oadrive_core/Trajectory2d.h>
#include <oadrive_control/LateralController.h>
#include <oadrive_util/Timer.h>
#include <oadrive_world/MultiTrajectory.h>

#include <boost/thread/mutex.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>


using namespace oadrive::core;

namespace oadrive {
namespace control {

/*! The DriverModule acts as a wrapper for a oadrive::core::Trajectory2D and oadrive::control::LateralController
*/
class DriverModule
{
public:

  // TODO: Remove or replace timer (not thread safe?)
  DriverModule();
  ~DriverModule();
  //!
  //! \brief setTrajectory Set new trajectory. Is reached will be reset.
  //! \param traj trajectory
  //! \return true if initialized correctly, false otherwise.
  //!
  bool setTrajectory(const oadrive::world::MultiTrajectory &traj );
  //!
  //! \brief halt force stop of the car. Speed will be set to 0
  //!
  void halt();
  //!
  //! \brief drive Start drive mode. Will be set to halt if the end of trajectory is reached
  //!
  void drive();
  ///!
  //! \brief update update Steering angle and check if position has been reached
  //! \param pose postion of the car
  //!
  void update( const ExtendedPose2d &pose );
  //!
  //! \brief getSteeringAngle get steering angle it is calculate in udate
  //! \return steering angle in degrees. (0° is straight forward)
  //!
  float getSteeringAngle();
  //!
  //! \brief getSpeed get driving speed
  //! \return speed
  //!
  float getSpeed();
  ///!
  //! \brief setTargetSpeed set the maximum speed for the controler
  //! \param speed speed
  //!
  void setTargetSpeed(float speed);
  float getTargetSpeed();

  ExtendedPose2d getCarPose();

  /*! Return maximum angle which will ever be returned by getSteering().
   * \note NOT mutex locked! */
  float getMaxSteeringAngle() { return mMaxSteering; }

  /*! Checks if the end of the trajectory has been reached.
   * \note After retrieving the flag, this resets the flag back to 'false',
   *    so that an event is only called once!*/
  bool checkEndOfTrajectoryFlag();

  /*! Find out which part of the MultiTrajectory the driver is currently on. */
  size_t getCurrentTrajectoryIndex();

  /*! Resets the Driver */
  void reset();

private:

  void privateDrive();
  void privateHalt();

  bool setTrajectory( const Trajectory2d &traj );

  LateralController mLateralController;

  oadrive::world::MultiTrajectory mMultiTrajectory;
  Trajectory2d mTrajectory;
  size_t mCurrentTrajectoryIndex;

  ExtendedPose2d mCurrentPose;
  float mMaxSteering;
  float mMinSteering;
  float mCurrentSteering;
  float mCurrentSpeed;
  float mTargetSpeed;

  // recommended speed for mMaxSteering steering angle
  float mMaxSteeringSpeed;
  // recommended speed for mMinSteering steering angle
  float mMinSteeringSpeed;
  /* ratio between min and max steering speed and min and max steering
   * used in calculation of speed according to steering angle
   * this product is pre-calculated for performance reasons
   */
  float mRatioMinMaxSteeringSpeedToMinMaxSteering;

  bool mDriving;
  bool mInitializedTrajectory;

  bool mTrajectoryEndReached;

  boost::posix_time::ptime mLastImageStamp;
  bool mLowFrameRate;

  boost::mutex mtx;

public:
  // use a proper alignment when calling the constructor.
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

}	//namespace
}	//namespace

#endif // include guard
