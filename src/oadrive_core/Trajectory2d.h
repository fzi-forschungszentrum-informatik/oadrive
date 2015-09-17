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
 * \author  Sebastian Klemm <klemm@fzi.de>
 * \date    2015-02-06
 *
 */
//----------------------------------------------------------------------
#ifndef OADRIVE_CORE_TRAJECTORY_2D_H_INCLUDED
#define OADRIVE_CORE_TRAJECTORY_2D_H_INCLUDED

#include <icl_core/VectorWrapper.h>
#include <oadrive_core/Types.h>
#include <oadrive_core/ExtendedPose2d.h>

namespace oadrive {
namespace core {

/*! A Trajectory out of ExtendedPose2d.
 *  The trajectory contains multiple global flags to indicate
 *  the availability of such kind of data.
 *
 *  The trajectory must always be driven from index 0 to index size()-1.
 *  A valid trajectory may only contain one kind of movement direction (either
 *  forward or backward). If generated from data that would violate
 *  this criteria, \see shortcut().
 */
class Trajectory2d
{
public:

  typedef ExtendedPose2dVector ContainerType;

  /*! The trajectory itself based on a std::vector, providing all vector features.
   *  The macro provides a standard constructor with member initialization.
   */
  ICL_CORE_WRAP_VECTOR_INIT(Trajectory2d, ContainerType, private, m_trajectory,
                            // member initialization
                            m_curvature_available(false),
                            m_velocity_available(false),
                            m_is_forward_trajectory(true)
                            );

public:
  ~Trajectory2d();


  /* -------------------------------------------------------
   *   Access to members and poses within the trajectory
   * ------------------------------------------------------- */

  // access to curvature
  bool& curvatureAvailable();
  const bool& curvatureAvailable() const;

  // access to velocity
  bool& velocityAvailable();
  const bool& velocityAvailable() const;

  // shortcut to poses at given index
  Pose2d& operator () (std::size_t index);
  const Pose2d& operator () (std::size_t index) const;

  // access driving direction
  bool& isForwardTrajectory();
  const bool& isForwardTrajectory() const;

  /* -------------------------------------------------------
   *   Calculations on the trajectory
   * ------------------------------------------------------- */

  /*! Calculates the 2D curvature for an ordered set of three points.
   *  The curvature is defined as (1/radius), where the radius is
   *  calculated from the circle intersecting the three given points.
   *  A positive curvature means the curve turns left (positive yaw
   *  rate), a negative curvature means the curve turns right
   *  (negative yaw rate).
   *  \param vec_a First point of the trajectory.
   *  \param vec_b Second point of the trajectory.
   *  \param vec_c Third point of the trajectory.
   *  \param backward If \c true, assume the vehicle drives the
   *         trajectory in reverse.  This effectively flips the
   *         curvature sign.
   */
  double calculateCurvature(const Position2d& p1,
                            const Position2d& p2,
                            const Position2d& p3,
                            bool backward = false) const;

  /*! Calculates the curvature for the trajectory at each contained
   *  pose. This is equal to the reciprocal of a circle's radius
   *  through each set of 3 poses.
   *  Sets #m_curvature_available.
   */
  void calculateCurvature();

  /*! Calculate the trajectory's length when walking
   *  along each pose from start to end
   */
  double length() const;

  /*! Calculate the length when walking
   *  along each pose from start up to (including) \a index
   */
  double lengthUpTo(std::size_t index) const;

  /*! Calculate the length when walking
   *  along each pose from (including) \a index until the trajectory's end
   */
  double lengthFrom(std::size_t index) const;

  /*! Calculate the length when walking along each pose
   *  from \a start_index to \a end_index.
   */
  double lengthBetween(std::size_t start_index, std::size_t end_index) const;

  /*! Calculate the orientations of the poses contained in the
   *  trajectory. This is simply done by directing the poses
   *  towards their successor.
   *
   *  \note All previously contained orientations are overwritten.
   *
   *  \note For vehicles with kinematic restrictions such as
   *  cars this may produce invalid trajectories:
   *  this is only an approximation and underlies severe
   *  approximation errors if the trajectory's poses are not dense.
   */
  void calculateOrientations();

  /* -------------------------------------------------------
   *   Helper functions
   * ------------------------------------------------------- */

  /*! Create some test data.
   *  \param sampling_rate The distance in which to sample the test function.
   *  \param set_velocity A velocity will be added if set to \c true.
   *  \param movement_direction_forward The trajectory will be created for
   *     \li \c true  --> forward,
   *     \li \c false --> backward
   *  movement.
   */
  void fillWithTestData(double sampling_rate, bool set_velocity, bool movement_direction_forward);

  //! Generates a data and a plot file. Appends different file suffixes.
  void toGnuplot(const std::string& filename_without_suffix) const;

  //! Append \a other at the end of this trajectory.
  void append(const Trajectory2d& other);

  /*! Try to shortcut the trajectory, assuming a vehicle with car-like kinematics.
   *  Will remove poses from the trajectory that would violate the
   *  movement direction.
   */
  void shortcut();

private:

  //! A flag to indicate that curvature is available
  bool m_curvature_available;

  //! A flag to indicate that velocity is available
  bool m_velocity_available;

  /*! A flag to indicate forward movement, which is mainly used to
   *  spare checking ExtendedPose2d velocity sign.
   */
  bool m_is_forward_trajectory;

};

typedef std::vector<Trajectory2d> Trajectory2dSequence;

} // end of ns
} // end of ns

#endif
