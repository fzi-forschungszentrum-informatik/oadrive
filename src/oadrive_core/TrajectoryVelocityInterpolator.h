// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-
// -- BEGIN LICENSE BLOCK ----------------------------------------------
// This file is part of the Open Autonomous Driving Library.
//
// This program is free software licensed under the CDDL
// (COMMON DEVELOPMENT AND DISTRIBUTION LICENSE Version 1.0).
// You can find a copy of this license in LICENSE.txt in the top
// directory of the source code.
//
// © Copyright 2015 FZI Forschungszentrum Informatik, Karlsruhe, Germany

// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
*
* \author  Marc Essinger <essinger@fzi.de>
* \date    2014-12-17
*
* Static class methods that calculate velocity profiles for different
* acceleration strategies.
*/
//----------------------------------------------------------------------

#ifndef OADRIVE_CORE_TRAJECTORY_VELOCITY_INTERPOLATOR_H_INCLUDED
#define OADRIVE_CORE_TRAJECTORY_VELOCITY_INTERPOLATOR_H_INCLUDED

#include <utility>
#include <limits>

#include <oadrive_core/Trajectory2d.h>

namespace oadrive {
namespace core {

/*!
 * \brief The TrajectoryVelocityInterpolator class
 *
 * Calculates speed profiles on a given range of Trajectory points, following the laws
 * of constant acceleration motion.
 *
 * See for example: http://en.wikipedia.org/wiki/Acceleration
 *
 */
class TrajectoryVelocityInterpolator
{

public:
  /*!
   * constantJoltAcceleratedMotion
   * Iterates a given interval of [\a start, \a end) of trajectory points and sets their velocity attributes
   * conforming to a constant jolt acceleration that begins at \a v_start at \a v_start and ends with \a v_end
   * on a trajectory point before \a end.
   *
   * Parameters:
   * \param start  Trajectory point to start with the motion.
   * \param end    Trajectory point before which the motion is expected to finish.
   * \param v_start Start velocity [m/s]
   * \param v_end   End Velocity [m/s]
   * \param max_acceleration Maximum absolute acceleration to use[m/s^2]
   * \param max_jolt Maximum absolute jolt to use [m/s^3]
   * \param overwrite If set to true, the velocity values of all points after reaching v_end will be set to v_end.
   *                  If set to false, the setting of velocities will stop after reaching v_end or when encountering
   *                  a trajectory point with a lower velocity than the calculated motion velocity. This allows the composition
   *                  of start and stop motions.
   *
   * Returns an iterator which is placed upon the last trajectory point of the acceleration. If it equals \a end
   * the acceleration could not be finished before reaching the given interval end.
   */
  template <class trajectory_iterator_t>
  static trajectory_iterator_t constantJoltAcceleratedMotion(trajectory_iterator_t start,
                                                             trajectory_iterator_t end,
                                                             const double v_start,
                                                             const double v_end,
                                                             const double max_acceleration,
                                                             const double max_jolt,
                                                             const bool overwrite);


  /*!
   * constantVelocityMotion
   *
   * Sets a constant velocity profile across [start, end) where each trajectory point has the
   * fixed \param velocity [m/s]
   */
  static void constantVelocityMotion(Trajectory2d& trajectory,
                                     const double velocity);

  /*! Tries to set a velocity profile where \param v_desired is held until braking begins and the
   * trajectory ends with velocity 0 at the last trajectory point.
   *
   * \param trajectory the trajectory to set velocity values for.
   * \param v_desired The desired velocity before braking starts [m/s]
   * \param a_brake   Brake acceleration. [m/s^2]
   * \param max_jolt  Maximum absolute jolt by which brake acceleration is to be increased. [m/s^3]
   * \param overwrite If set to true, the velocity value of each trajectory point will be overwritten. Same rules apply as for
   *                  \a overwrite in constantJoltAcceleratedMotion.
   * \returns \c True if a valid velocity profile could be set.
   */
  static inline bool brakeProfile(Trajectory2d& trajectory,
                                  const double v_desired,
                                  const double a_brake,
                                  const double max_jolt,
                                  const bool   overwrite=false)
  {
     Trajectory2d::ContainerType::reverse_iterator end = constantJoltAcceleratedMotion(
      trajectory.rbegin(), trajectory.rend(), 0, v_desired, a_brake, max_jolt, overwrite);
     if (end != trajectory.rend())
     {
       trajectory.velocityAvailable() = true;
       return true;
     }
     return false;
  }

  /*! Tries to set a velocity profile where \a v_desired is reached after accelerating from v_start to v_desired. Acceleration begins
   * on the first trajectory point.
   *
   * \param trajectory     The trajectory to set velocity values for.
   * \param v_start        The velocity of the first trajectory point.[m/s]
   * \param v_desired      The velocity to de/accelerate to. [m/s]
   * \param a_motor        The maximum possible acceleration. [m/s^2]
   * \param max_jolt       The maximum jolt [m/s^3]
   * \param overwrite      If set to true, the velocity value of each trajectory point will be overwritten. Same rules apply as for
   *                       \a overwrite in constantJoltAcceleratedMotion.
   * \returns \c True if a valid velocity profile could be set.
   */
  static bool startupProfile(Trajectory2d& trajectory,
                             const double v_start,
                             const double v_desired,
                             const double a_motor,
                             const double max_jolt,
                             const bool   overwrite=false)
  {
    Trajectory2d::ContainerType::iterator end = constantJoltAcceleratedMotion(
      trajectory.begin(), trajectory.end(), v_start, v_desired, a_motor, max_jolt, overwrite);
    if (end != trajectory.end())
    {
      trajectory.velocityAvailable() = true;
      return true;
    }
    return false;
  }

  /*! Combine a startup profile at the first half of the trajectory
   *  and a brake profile at the second half.
   *  If the given constraints are suitable, there will be a
   *  phase with constant velocity in the 'middle' sector of the
   *  trajectory.
   *  If the trajectory is too short, the maximum possible velocity
   *  will be determined and then used instead of v_desired.
   * \param trajectory     The trajectory to set velocity values for.
   * \param v_start        The velocity of the first trajectory point.[m/s]
   * \param v_desired      The velocity to de/accelerate to. [m/s]
   * \param a_motor        The maximum possible acceleration. [m/s^2]
   * \param a_brake        Brake acceleration. [m/s^2]
   * \param max_jolt       The maximum jolt [m/s^3]
   * \returns \c True if a valid velocity profile could be set.
   */
  static bool startupAndBrakeProfile(Trajectory2d& trajectory,
                                     const double v_start,
                                     const double v_desired,
                                     const double a_motor,
                                     const double a_brake,
                                     const double max_jolt);

  //! Plot the velocity profile contained within the given \a trajectroy to \a filename
  static void plotTrajectoryVelocity(const Trajectory2d& trajectory, const std::string& filename);

};

} // end of ns
} // end of ns
#endif
