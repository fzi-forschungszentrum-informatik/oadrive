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
 * \date    2014-07-03
 *
 * Defines overloads of some standard math functions for use with
 * Eigen quaternions.  Note that Eigen treats quaternions almost
 * exclusively as a special type of transform, which is why most
 * standard arithmetic operations are not offered by Eigen.
 */
//----------------------------------------------------------------------
#ifndef OADRIVE_CORE_QUATERNION_FUNCTIONS_H_INCLUDED
#define OADRIVE_CORE_QUATERNION_FUNCTIONS_H_INCLUDED

#include <Eigen/Geometry>
#include <oadrive_core/MathFunctions.h>

namespace std {

//! Overloads the exponential function for quaternions.
template <typename TScalar, int t_options>
inline Eigen::Quaternion<TScalar, t_options> exp(const Eigen::Quaternion<TScalar, t_options>& q)
{
  const TScalar n = q.vec().norm();
  const TScalar ew = exp(q.w());
  const Eigen::Matrix<TScalar, 3, 1> im = ew * q.vec() * ::oadrive::core::sinc(n);
  return Eigen::Quaternion<TScalar, t_options>(ew * cos(n), im.x(), im.y(), im.z());
}

//! Overloads the logarithm function for quaternions.
template <typename TScalar, int t_options>
inline Eigen::Quaternion<TScalar, t_options> log(const Eigen::Quaternion<TScalar, t_options>& q)
{
  const TScalar n = q.vec().norm();
  const TScalar lw = log(std::abs(q.w()));
  if (n == 0)
  {
    return Eigen::Quaternion<TScalar, t_options>(lw, TScalar(0), TScalar(0), TScalar(0));
  }
  else
  {
    const Eigen::Matrix<TScalar, 3, 1> im = q.vec() * std::atan2(n, q.w()) / n;
    return Eigen::Quaternion<TScalar, t_options>(lw, im.x(), im.y(), im.z());
  }
}

}

//! Multiplication of a scalar and a quaternion.
template <typename TScalar, int t_options>
inline Eigen::Quaternion<TScalar, t_options> operator * (const TScalar s, const Eigen::Quaternion<TScalar, t_options>& q)
{
  return Eigen::Quaternion<TScalar, t_options>(s*q.w(), s*q.x(), s*q.y(), s*q.z());
}

//! Multiplication of a quaternion and a scalar.
template <typename TScalar, int t_options>
inline Eigen::Quaternion<TScalar, t_options> operator * (const Eigen::Quaternion<TScalar, t_options>& q, const TScalar s)
{
  return Eigen::Quaternion<TScalar, t_options>(s*q.w(), s*q.x(), s*q.y(), s*q.z());
}

//! Sum of two quaternions.
template <typename TScalar, int t_options>
inline Eigen::Quaternion<TScalar, t_options> operator + (const Eigen::Quaternion<TScalar, t_options>& q,
                                                         const Eigen::Quaternion<TScalar, t_options>& r)
{
  return Eigen::Quaternion<TScalar, t_options>(q.w()+r.w(), q.x()+r.x(), q.y()+r.y(), q.z()+r.z());
}

//! Difference of two quaternions.
template <typename TScalar, int t_options>
inline Eigen::Quaternion<TScalar, t_options> operator - (const Eigen::Quaternion<TScalar, t_options>& q,
                                                         const Eigen::Quaternion<TScalar, t_options>& r)
{
  return Eigen::Quaternion<TScalar, t_options>(q.w()-r.w(), q.x()-r.x(), q.y()-r.y(), q.z()-r.z());
}

#endif
