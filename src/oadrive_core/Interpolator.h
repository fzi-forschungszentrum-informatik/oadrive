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
 * \date    2014-03-07
 *
 */
//----------------------------------------------------------------------
#ifndef OADRIVE_CORE_INTERPOLATOR_H_INCLUDED
#define OADRIVE_CORE_INTERPOLATOR_H_INCLUDED

#include <oadrive_core/Types.h>
#include <oadrive_core/Trajectory2d.h>

namespace oadrive {
namespace core {

//! Functions for interpolation on scalars and vectors / positions.
class Interpolator
{
public:

  //! Constructor
  Interpolator();

  //! Destructor
  ~Interpolator();

  /* -------------------------------------------
   *   Interpolation on values
   * ------------------------------------------- */

  /*! Perform linear interpolation between two double values. Interpolation is done according to \a ratio,
   *  e.g. a ratio of 0.5 will interpolate into the middle between \a value1 and \a value2
   *  \param value1 First value
   *  \param value2 Second value
   *  \param ratio Ratio in [0.0, 1.0] to interpolate with
   *  \return The interpolated value
   */
  static double interpolateLinear(double value1, double value2, double ratio);

  /*! Perform cubic interpolation over the given values.
   *  Interpolation is done between \a value1 and \a value2.
   *  Other values are used as support information.
   */
  static double interpolateCubic(double value0, double value1, double value2, double value3, double ratio);

  /*! Perform spline interpolation using Catmull-Rom polynom coefficients
   *  over the given values. The spline is forced to go through the
   *  given values.
   *  Interpolation is done between \a value1 and \a value2.
   *  Other values are used as support information.
   */
  static double interpolateSpline(double value0, double value1, double value2, double value3, double ratio);


  /* -------------------------------------------
   *   Interpolation on Positions
   * ------------------------------------------- */

  //! Component-wise linear interpolation using \see interpolateLinear()
  static Position2d interpolateLinear(const Position2d& p1,
                                      const Position2d& p2, double ratio);


  //! Component-wise cubic interpolation using \see interpolateCubic()
  static Position2d interpolateCubic(const Position2d& p0, const Position2d& p1,
                                     const Position2d& p2, const Position2d& p3, double ratio);


  //! Component-wise spline interpolation using \see interpolateSpline()
  static Position2d interpolateSpline(const Position2d& p0, const Position2d& p1,
                                      const Position2d& p2, const Position2d& p3, double ratio);

  /* ---------------------------------------------------------------
   *   Interpolation on orientations (quaternions)
   * --------------------------------------------------------------- */

  /*! Perform linear interpolation on quaternions using slerp.
   *  In fact this is just a wrapper around the Eigen implementation of slerp
   *  to provide a consistend interface.
   */
  static Quaternion interpolateLinear(const Quaternion& p, const Quaternion& q, double ratio);

  /*! Perform spherical cubic interpolation between the quaternions \a p and \a q.
   *  This is achieved by an iteration of three slerps, similar to de Casteljau algorithm,
   *  using kind of a bilinear interpolation.
   *  \note API is slightly different from other interpolation methods because of the underlying algorithm
   */
  static Quaternion interpolateCubic(const Quaternion& p, const Quaternion& a,
                                     const Quaternion& b, const Quaternion& q, double ratio);

  /*! Perform cubic spline interpolation over the given quaternions.
   *  This is achieved by using spherical cubic interpolation \see squad()
   *  and calculation of some further support quaternions.
   */
  static Quaternion interpolateSpline(const Quaternion& support_before, const Quaternion& p,
                                      const Quaternion& q, const Quaternion& support_after, double ratio);


  /* -------------------------------------------
   *   Interpolation on Poses
   * ------------------------------------------- */

  //! Component-wise linear interpolation using \see interpolateLinear()
  static Pose2d interpolateLinear(const Pose2d& p1,
                                  const Pose2d& p2, double ratio);

  /*! Component-wise linear interpolation using \see interpolateLinear()
   *  on the pose part and the velocity, if not \c nan.
   */
  static ExtendedPose2d interpolateLinear(const ExtendedPose2d& p1,
                                          const ExtendedPose2d& p2, double ratio);
  /* -------------------------------------------
   *   Interpolation on Trajectory
   * ------------------------------------------- */

  /*! Linearly interpolate into every segment of the
   *  given \a trajectory using the desired \a ratio within (0,1)
   */
  static void interpolateLinear(Trajectory2d& trajectory, double ratio);

  /*! Smooth a trajectory using B-Spline.
   *  A maximum number of \a steps of smoothing with B-Splines is performed.
   *  If no smoothing progress is detected regarding the value \a min_change,
   *  fewer steps may be performed.
   *  \note Only trajectories with at least 3 entries can be smoothed.
   */
  static void smoothBSpline(Trajectory2d& trajectory,
                            std::size_t max_steps = 5,
                            double min_change = std::numeric_limits<double>::epsilon());

private:

  /* ---------------------------------------------------------------
   *   Helper functions
   * --------------------------------------------------------------- */

  //! Calculate a support quaternion in between \a p and \a q influenced by \a o
  static Quaternion calculateSupportQuaternion(const Quaternion& o,   // o = p_{n-1}
                                               const Quaternion& p,   // p = p_{n}
                                               const Quaternion& q);  // q = p_{n+1}

};


} // end of ns
} // end of ns

#endif
