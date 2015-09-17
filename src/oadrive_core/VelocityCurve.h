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
 * \author  Jan Oberlaender <oberlaender@fzi.de>
 * \date    2014-12-21
 *
 */
//----------------------------------------------------------------------
#ifndef OADRIVE_CORE_VELOCITY_CURVE_H_INCLUDED
#define OADRIVE_CORE_VELOCITY_CURVE_H_INCLUDED

namespace oadrive {
namespace core {

/*! Stores parameters, and provides various convenience functions, for
 *  a velocity curve that accelerates/decelerates smoothly from an
 *  initial to a target velocity.  The curve is constrained by maximum
 *  acceleration and jolt values, to guarantee a certain smoothness
 *  (e.g. to ensure passenger comfort).  The resulting curve takes the
 *  following form:
 *  - Apply the maximum jolt (negative, if the target velocity is
 *    smaller than the initial velocity) for a time period #m_t1, but
 *    at most until the maximum acceleration is reached.
 *  - Apply a zero jolt for a time period #m_t2 (may be zero), i.e.,
 *    apply constant acceleration.
 *  - Apply the negative maximum jolt (positive, if the target
 *    velocity is smaller than the initial velocity) for a time period
 *    #m_t1, so that the acceleration is zero again, and the desired
 *    target velocity is reached exactly, when the time period ends.
 */
class VelocityCurve
{
public:
  /*! Calculate curve parameters to accelerate/decelerate from
   *  velocity \a v1 to target velocity \a v2, where the absolute
   *  acceleration must not exceed \a a_max and the absolute jolt must
   *  not exceed \a j_max.
   *  \param v1 Initial velocity [m/s].
   *  \param v2 Target velocity [m/s].
   *  \param a_max Maximum absolute acceleration [m/s^2].
   *  \param j_max Maximum absolute jolt [m/s^3].
   */
  VelocityCurve(double v1, double v2, double a_max, double j_max);

  //! The jolt over time.
  double j(double t) const;

  //! The acceleration over time.
  double a(double t) const;

  //! The velocity over time.
  double v(double t) const;

  //! The location [m] over time (assuming s(0)=0).
  double s(double t) const;

  /*! The time [s] at which the given velocity \a v is reached.
   *  Inverse of v().  Only valid if \a v lies between #m_v1 and
   *  #m_v2 (other velocities are never reached).
   */
  double tv(double v) const;

  /*! The time [s] at which the given location \a s is reached.
   *  Inverse of s().  Only valid if #m_v1 and #m_v2 have the same
   *  sign.  If either #m_v1 or #m_v2 is zero, the valid domain is
   *  limited as the trajectory does not move beyond the location at
   *  which the vehicle is stopped.
   */
  double ts(double s) const;

  /*! Starting at #m_v1, the maximum speed (or minimum, if #m_v2 <
   *  #m_v1) that can be reached \e smoothly at location \a s, meaning
   *  that acceleration at \a s is zero and the acceleration and jolt
   *  constraints are upheld.
   */
  double vDiffMax(double s) const;

  /*! Time period [s] for which the maximum jolt is applied (twice,
   *  once positive and once negative).
   */
  inline double joltPeriod() const { return m_t1; }

  /*! Time period [s] for which a zero jolt is applied (i.e. a
   *  constant acceleration takes place).
   */
  inline double constantAccelerationPeriod() const { return m_t2; }

  //! Time to target velocity in seconds.
  inline double timeToTargetVelocity() const { return 2*m_t1+m_t2; }

  //! Distance relative to start when target velocity is reached.
  inline double distanceAtTargetVelocity() const { return m_sc; }

  /*! Maximum acceleration effectively reached.  This may be lower
   *  than #m_a_max if the jolt limit is comparatively strict, so that
   *  the acceleration has to be reduced again before the maximum
   *  allowed acceleration is ever reached.
   */
  inline double effectiveMaxAcceleration() const { return m_a_max_eff; }

private:
  //! Initial velocity [m/s].
  const double m_v1;
  //! Target velocity [m/s].
  const double m_v2;
  //! Maximum absolute acceleration [m/s^2].
  const double m_a_max;
  //! Maximum absolute jolt [m/s^2].
  const double m_j_max;
  /*! Time period [s] for which the maximum jolt is applied (twice,
   *  once positive and once negative).
   */
  double m_t1;
  /*! Time period [s] for which a zero jolt is applied (i.e. a
   *  constant acceleration takes place).
   */
  double m_t2;
  /*! The sign of the velocity change (positive if #m_v2 > #m_v1,
   *  negative otherwise).
   */
  double m_sign;
  //! Velocity after initial jolt phase.
  double m_va;
  //! Velocity before final jolt phase.
  double m_vb;
  //! Effective maximum absolute acceleration.
  double m_a_max_eff;
  //! Position after initial jolt phase.
  double m_sa;
  //! Position before final jolt phase.
  double m_sb;
  //! Position when target velocity is reached.
  double m_sc;
};

} // end of ns
} // end of ns

#endif
