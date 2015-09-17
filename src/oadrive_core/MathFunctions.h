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
 * \author  Bernd Gassmann <gassmann@fzi.de>
 * \date    2000-04-14
 * \author  Jan Oberlaender <oberlaender@fzi.de>
 * \date    2013-09-01
 *
 * General mathematical functions missing from the standard libraries.
 */
//----------------------------------------------------------------------
#ifndef OADRIVE_CORE_MATH_FUNCTIONS_H_INCLUDED
#define OADRIVE_CORE_MATH_FUNCTIONS_H_INCLUDED

#include <cstddef>
#include <cmath>

namespace oadrive {
namespace core {

/*! Floating-point modulo.  Evaluates \a a fmod(\a b) and returns the
 *  result (0 <= result < b).
 */
inline double mod(const double a, const double b)
{
  double c = std::fmod(a, b);
  return c >= 0 ? c : c + b;
}

/*! Floating-point modulo.  Evaluates \a a fmod(\a b) and returns the
 *  result (0 <= result < b).
 */
inline float mod(const float a, const float b)
{
  float c = std::fmod(a, b);
  return c >= 0 ? c : c + b;
}

/*! Floating-point modulo.  Evaluates \a a fmod(\a b) and returns the
 *  result (0 <= result < b).
 */
inline double mod(const double a, const int b)
{
  return mod(a, double(b));
}

/*! Integer modulo.  Evaluates \a a mod(\a b) and returns the result
 *  (0 <= result < b).
 */
inline int mod(const int a, const int b)
{
  int c = a % b;
  return c >= 0 ? c : c + b;
}

/*! Integer modulo.  Evaluates \a a mod(\a b) and returns the result
 *  (0 <= result < b).
 */
inline long mod(const long a, const long b)
{
  int c = a % b;
  return c >= 0 ? c : c + b;
}

/*! Returns the sign (1 or -1) of \a a. The sign of zero here is
 *  defined as 1!
 */
inline int sgn(const double a)
{
  return a >= 0 ? 1 : -1;
}

/*! Returns the sign (1 or -1) of \a a. The sign of zero here is defined as 1!
 */
inline int sgn(const long double a)
{
  return a >= 0 ? 1 : -1;
}

/*! Returns the sign (1, -1 or 0) of \a a. The sign of zero here is defined as 0!
 */
inline int sgn0(const double a)
{
  return a > 0 ? 1 : (a < 0 ? -1 : 0);
}

/*! Returns the sign (1, -1 or 0) of \a a. The sign of zero here is defined as 0!
 */
inline int sgn0(const long double a)
{
  return a > 0 ? 1 : (a < 0 ? -1 : 0);
}

/*! Returns the sign (1 or -1) of \a a. The sign of zero here is defined as 1!
 */
inline int sgn(const long a)
{
  return a >= 0 ? 1 : -1;
}

/*! Returns the sign (1, -1 or 0) of \a a. The sign of zero here is defined as 0!
 */
inline int sgn0(const long a)
{
  return a > 0 ? 1 : (a < 0 ? -1 : 0);
}

//! Returns the \a angle (in rad) normalized to [0, 2*M_PI).
inline double normalizeAngleUnsigned(const double angle)
{
  return mod(angle, 2*M_PI);
}

//! Returns the \a angle (in rad) normalized to [0, 2*M_PI).
inline float normalizeAngleUnsigned(const float angle)
{
  return mod(angle, float(2*M_PI));
}

//! Returns the \a angle (in rad) normalized to [-M_PI, M_PI).
inline double normalizeAngleSigned(const double angle)
{
  return mod(angle + M_PI, 2*M_PI) - M_PI;
}

//! Returns the \a angle (in rad) normalized to [-M_PI, M_PI).
inline float normalizeAngleSigned(const float angle)
{
  return mod(angle + float(M_PI), float(2*M_PI)) - float(M_PI);
}

//! Returns the \a angle (in rad) normalized to [ref-M_PI, ref+M_PI).
inline double normalizeAngleRef(const double angle, const double ref)
{
  return mod(angle - ref + M_PI, 2*M_PI) + ref - M_PI;
}

//! Returns the \a angle (in rad) normalized to [ref-M_PI, ref+M_PI).
inline float normalizeAngleRef(const float angle, const float ref)
{
  return mod(angle - ref + float(M_PI), float(2*M_PI)) + ref - float(M_PI);
}

//! Returns the \a angle (in degrees) normalized to [-180, 180).
inline double normalizeAngleInDegreeSigned(const double angle)
{
  return mod(angle + 180.0, 360.0) - 180.0;
}

//! Returns the \a angle (in degrees) normalized to [-180, 180).
inline float normalizeAngleInDegreeSigned(const float angle)
{
  return mod(angle + 180.0f, 360.0f) - 180.0f;
}

//! Returns \a a to the \a b'th power for integer values.
inline int pow(const int a, int b)
{
  int ret;
  for (ret = 1; b > 0; --b)
  {
    ret *= a;
  }
  return ret;
}

//! Rounds \a value with a specified precision.
template <typename TScalar>
inline TScalar round(const TScalar value, const int precision = 0)
{
  TScalar fact = pow(1e1, TScalar(precision));
  return std::floor(((value*fact) + 0.5)) / fact;
}

//! Returns the ceiling of \a value with a specified precision.
template <typename TScalar>
inline TScalar ceil(const TScalar value, const int precision = 0)
{
  TScalar fact = pow(1e1, TScalar(precision));
  return std::ceil(value*fact) / fact;
}

//! Returns the floor of \a value with a specified precision.
template <typename TScalar>
inline TScalar floor(const TScalar value, const int precision = 0)
{
  TScalar fact = pow(1e1, TScalar(precision));
  return std::floor(value*fact) / fact;
}

/*! Evaluate the Gaussian function at point \a x with mean \a mu and
 *  standard deviation \a sigma.
 *  \note This is \em not a normal probability distribution function
 *        (it lacks normalization to a total area of 1)!
 */
template <typename TScalar>
inline TScalar gaussian(TScalar x, TScalar mu, TScalar sigma)
{
  return std::exp(-(x-mu)*(x-mu)/(TScalar(2.0)*sigma*sigma));
}

/*! Evaluate the general Gaussian function at point \a x with peak
 *  height \a h, mean \a mu, standard deviation \a sigma and offset o.
 *  \note This is \em not a normal probability distribution function
 *        (it lacks normalization to a total area of 1)!  You can,
 *        however, obtain a normal PDF by setting \a h = 1/(\a sigma *
 *        sqrt(2*M_PI)), and \a o = 0.
 */
template <typename TScalar>
inline TScalar gaussian(TScalar x, TScalar mu, TScalar sigma, TScalar h, TScalar o)
{
  return h*std::exp(-(x-mu)*(x-mu)/(TScalar(2.0)*sigma*sigma)) + o;
}

/*! Calculate the squared value of x
 *  without actually using power functions.
 */
template <typename TScalar>
inline TScalar square(TScalar x)
{
  return x*x;
}

/*! Calculates sin(x)/x, with the special case that for x=0, the
 *  limiting value 1 is returned.
 */
inline float sinc(const float x)
{
  return (x != 0.0) ? std::sin(x)/x : 1.0;
}

/*! Calculates sin(x)/x, with the special case that for x=0, the
 *  limiting value 1 is returned.
 */
inline double sinc(const double x)
{
  return (x != 0.0) ? std::sin(x)/x : 1.0;
}

/*! Normalized version of sinc(x).  Calculates sin(pi*x)/(pi*x), with
 *  the special case that for x=0, the limiting value 1 is returned.
 */
inline float nsinc(const float x)
{
  return (x != 0.0) ? std::sin(M_PI*x)/(M_PI*x) : 1.0;
}

/*! Normalized version of sinc(x).  Calculates sin(pi*x)/(pi*x), with
 *  the special case that for x=0, the limiting value 1 is returned.
 */
inline double nsinc(const double x)
{
  return (x != 0.0) ? std::sin(M_PI*x)/(M_PI*x) : 1.0;
}

} // end of ns
} // end of ns

#endif
