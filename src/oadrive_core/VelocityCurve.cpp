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
#include "VelocityCurve.h"

#include <cmath>
#include <limits>
#include <vector>

#include <unsupported/Eigen/Polynomials>

namespace oadrive {
namespace core {

VelocityCurve::VelocityCurve(double v1, double v2, double a_max, double j_max)
  : m_v1(v1),
    m_v2(v2),
    m_a_max(a_max),
    m_j_max(j_max),
    m_sign((v2 >= v1) ? 1.0 : -1.0),
    m_a_max_eff(a_max)
{
  // Remember the velocity difference.
  const double abs_delta_v = std::abs(v2-v1);
  // If the delta is almost zero, we do nothing.
  if (abs_delta_v < std::numeric_limits<double>::epsilon() * std::abs(v2+v1) * 2
      || abs_delta_v < std::numeric_limits<double>::min())
  {
    m_t1 = 0;
    m_t2 = 0;
    m_va = m_vb = m_v1;
    m_a_max_eff = 0;
    m_sa = m_sb = m_sc = 0;
    return;
  }

  // First, we ignore a_max and simply apply the maximum jolt.
  //        { t*j_max         0 <= t < t_j
  // a(t) = { (t_j-t)*j_max   t_j <= t < 2*t_j
  //        { 0               otherwise
  //
  //      = { v1                                          t < 0
  // v(t) = { v1 + 0.5*j_max*t^2                          0 <= t < t_j
  //      = { v1 + 0.5*j_max*(t_j^2-t^2) + t*t_j*j_max    t_j <= t < 2*t_j
  //      = { v1 + 0.5*j_max*(-3*t_j^2) + 2*j_max*t_j^2   2*t_j <= t
  //
  // The final term for v(t) must be identical to v2, so assuming
  // without loss of generality that v2 > v1, we get
  //     v1 + 0.5*j_max*(-3*t_j^2) + 2*j_max*t_j^2 = v2
  // <=>      0.5*j_max*(-3*t_j^2) + 2*j_max*t_j^2 = v2 - v1 = delta_v
  // <=>                j_max*(-1.5t_j^2 + 2t_j^2) = delta_v
  // <=>                           0.5*j_max*t_j^2 = delta_v
  // <=>                                     t_j^2 = 2*delta_v/j_max
  // <=>                                       t_j = sqrt(2*delta_v/j_max) .
  double t_j = std::sqrt(abs_delta_v / j_max);

  // Now we consider the maximum allowed acceleration.  We cannot
  // apply the maximum jolt for too long, since a(t) = t*j_max must
  // not exceed a_max.
  //     t_j * j_max <= a_max
  // <=>         t_j <= a_max/j_max
  double t_j_max = a_max / j_max;

  // If we do not exceed a_max, everything is fine:
  if (t_j <= t_j_max)
  {
    m_t1 = t_j;
    m_t2 = 0;
    m_va = m_vb = m_v1 + 0.5*m_sign*j_max*m_t1*m_t1;
    m_a_max_eff = m_t1*j_max;
  }
  // Otherwise we must wait for a while before applying the reverse
  // jolt.
  else
  {
    m_t1 = t_j_max;
    // Now we need to figure out for how long we must apply the
    // constant maximum acceleration.  After applying the maximum
    // jolt, we have reached a velocity of
    //   v_a = v1 + 0.5*j_max*m_t1^2 .
    // If we are to reach the target velocity after applying the
    // negative maximum jolt, we must reach the following velocity
    // before applying it:
    //      v2 = v_b + m_t1*a_max - 0.5*j_max*m_t1^2
    // <=> v_b = v2 - m_t1*a_max + 0.5*j_max*m_t1^2
    // Consequently, we must stay at a_max for as long as it takes to
    // get from v_a to v_b.
    double v_a = 0.5*j_max*m_t1*m_t1;
    double v_b = abs_delta_v - m_t1*(a_max-0.5*j_max*m_t1);
    double secondary_delta_v = v_b-v_a;
    //     v_a + t*a_max = v_b
    // <=>       t*a_max = v_b-v_a
    // <=>             t = (v_b-v_a)/a_max
    m_t2 = secondary_delta_v / a_max;
    m_va = m_v1 + m_sign*v_a;
    m_vb = m_v1 + m_sign*v_b;
  }

  // Calculate relative positions at the beginning and end of the jolt
  // phases.
  m_sa = m_v1*m_t1 + m_sign*m_j_max*m_t1*m_t1*m_t1/6.0;
  m_sb = m_sa + m_t2*m_va + 0.5*m_sign*m_a_max_eff*m_t2*m_t2;
  m_sc = m_sb + m_t1*m_vb + 0.5*m_sign*m_a_max_eff*m_t1*m_t1 - m_sign*m_j_max*m_t1*m_t1*m_t1/6.0;
}

double VelocityCurve::j(double t) const
{
  if      (t < 0)           { return 0;               }
  else if (t < m_t1)        { return m_sign*m_j_max;  }
  else if (t < m_t1+m_t2)   { return 0;               }
  else if (t < 2*m_t1+m_t2) { return -m_sign*m_j_max; }
  else                      { return 0;               }
}

double VelocityCurve::a(double t) const
{
  if      (t < 0)           { return 0;                              }
  else if (t < m_t1)        { return t*m_sign*m_j_max;               }
  else if (t < m_t1+m_t2)   { return m_sign*m_a_max_eff;             }
  else if (t < 2*m_t1+m_t2) { return (2*m_t1+m_t2-t)*m_sign*m_j_max; }
  else                      { return 0;                              }
}

double VelocityCurve::v(double t) const
{
  const double td1 = t-m_t1;
  const double td2 = t-m_t1-m_t2;
  if      (t < 0)           { return m_v1;                                              }
  else if (t < m_t1)        { return m_v1 + 0.5*m_sign*m_j_max*t*t;                     }
  else if (t < m_t1+m_t2)   { return m_va + m_sign*m_a_max_eff*td1;                     }
  else if (t < 2*m_t1+m_t2) { return m_vb + m_sign*td2*(m_a_max_eff - 0.5*m_j_max*td2); }
  else                      { return m_v2;                                              }
}

double VelocityCurve::s(double t) const
{
  const double td1 = t-m_t1;
  const double td2 = t-m_t1-m_t2;
  if      (t < 0)           { return m_v1*t;                                                             }
  else if (t < m_t1)        { return m_v1*t + m_sign*m_j_max*t*t*t/6.0;                                  }
  else if (t < m_t1+m_t2)   { return m_sa + td1*(m_va + 0.5*m_sign*m_a_max_eff*td1);                     }
  else if (t < 2*m_t1+m_t2) { return m_sb + td2*(m_vb + 0.5*td2*m_sign*(m_a_max_eff - m_j_max*td2/3.0)); }
  else                      { return m_sc + (t-2*m_t1-m_t2)*m_v2;                                        }
}

double VelocityCurve::tv(double v) const
{
  if ((m_v1 <= m_v2 && (v < m_v1 || m_v2 < v)) ||
      (m_v2 <= m_v1 && (v < m_v2 || m_v1 < v)))
  {
    return std::numeric_limits<double>::quiet_NaN();
  }
  if (v == m_v1)
  {
    return 0;
  }
  if (v == m_v2)
  {
    return 2*m_t1+m_t2;
  }
  if (m_sign*v < m_sign*m_va)
  {
    return std::sqrt((v-m_v1)/(0.5*m_sign*m_j_max));
  }
  else if (m_sign*v < m_sign*m_vb)
  {
    return m_t1 + (v-m_va)/(m_sign*m_a_max_eff);
  }
  else if (m_sign*v <= m_sign*m_v2)
  {
    double p = -2*m_a_max_eff/m_j_max;
    double q = -2*(m_vb-v)/(m_sign*m_j_max);
    return -p/2 - std::sqrt(p*p/4-q) + m_t1 + m_t2;
  }
  else
  {
    return std::numeric_limits<double>::quiet_NaN();
  }
}

double VelocityCurve::ts(double s) const
{
  if (m_v1 * m_v2 < 0)
  {
    return std::numeric_limits<double>::quiet_NaN();
  }
  double vsign = (m_v1+m_v2) > 0 ? 1 : -1;

  // s(0) = 0 by definition.
  if (s == 0)
  {
    return 0;
  }

  // Before first jolt.
  if (vsign*s < 0)
  {
    if (m_v1 != 0)
    {
      return s/m_v1;
    }
    else
    {
      return std::numeric_limits<double>::quiet_NaN();
    }
  }

  // First jolt.
  if (vsign*s <= vsign*m_sa)
  {
    //  s = m_v1*t + m_j_max/6.0*t^3
    //  (m_sign*m_j_max/6.0)*t^3 + m_v1*t - s = 0
    Eigen::Vector4d polynomial;
    polynomial <<
      -s,
      m_v1,
      0,
      m_sign * m_j_max / 6;
    Eigen::PolynomialSolver<double, 3> solver;
    solver.compute(polynomial);
    bool has_a_real_root = false;
    double root = solver.absSmallestRealRoot(has_a_real_root);
    if (has_a_real_root)
    {
      return root;
    }
    else
    {
      return std::numeric_limits<double>::quiet_NaN();
    }
  }

  // Constant acceleration phase.
  if (vsign*s <= vsign*m_sb)
  {
    //  s = m_sa + td1*(m_va + 0.5*m_sign*m_a_max_eff*td1)
    //    = m_sa + td1*m_va + 0.5*m_sign_m_a_max_eff*td1^2
    //  0.5*m_sign_m_a_max_eff*td1^2 + td1*m_va + m_sa - s = 0
    // Let a = 0.5*m_sign*m_a_max_eff, then
    //  td1^2 + td1*m_va/a + (m_sa - s)/a = 0
    double a = 0.5*m_sign*m_a_max_eff;
    Eigen::Vector3d polynomial;
    polynomial <<
      (m_sa-s)/a,
      m_va/a,
      1;
    Eigen::PolynomialSolver<double, 2> solver;
    solver.compute(polynomial);
    bool has_a_real_root = false;
    double root = solver.absSmallestRealRoot(has_a_real_root);
    if (has_a_real_root)
    {
      return root + m_t1;
    }
    else
    {
      return std::numeric_limits<double>::quiet_NaN();
    }
  }

  // Second jolt.
  if (vsign*s <= vsign*m_sc)
  {
    //  s = m_sb + td2*(m_vb + 0.5*td2*m_sign*(m_a_max_eff - m_j_max*td2/3.0))
    //    = m_sb + td2*m_vb + 0.5*td2^2*m_sign*m_a_max_eff - 0.5*td2^3*m_sign*m_j_max/3.0
    //    = m_sb + m_vb * td2 + 0.5*m_sign*m_a_max_eff * td2^2 - m_sign*m_j_max/6.0 * td2^3
    // Let a = -m_sign*m_j_max/6.0
    //     b = 0.5*m_sign*m_a_max_eff
    //     c = m_vb
    //     d = m_sb-s
    // Then
    //  a*td2^3 + b*td2^2 + c*td + d = 0
    Eigen::Vector4d polynomial;
    polynomial <<
      m_sb-s,
      m_vb,
      0.5*m_sign*m_a_max_eff,
      -m_sign*m_j_max/6.0;
    Eigen::PolynomialSolver<double, 3> solver;
    solver.compute(polynomial);
    bool has_a_real_root = false;
    double root = solver.absSmallestRealRoot(has_a_real_root);
    if (has_a_real_root)
    {
      return root + m_t1 + m_t2;
    }
    else
    {
      return std::numeric_limits<double>::quiet_NaN();
    }
  }

  // After second jolt.
  return 2*m_t1 + m_t2 + (s-m_sc)/m_v2;
}

double VelocityCurve::vDiffMax(double s) const
{
  // This method only works if we're not changing directions.
  if (m_v1 * m_v2 < 0)
  {
    return std::numeric_limits<double>::quiet_NaN();
  }
  if (m_v1 <= 0 && m_v2 <= 0)
  {
    // Force a negative location.
    s = -std::abs(s);
  }
  else
  {
    // Force a positive location.
    s = std::abs(s);
  }

  // Max acceleration is reached after the following time:
  // a_max = t_max_acc*j_max => t_max_acc = a_max/j_max
  const double t_max_acc = std::abs(m_a_max / m_j_max);

  // Velocity after the initial jolt phase:
  const double v_max_acc = m_v1 + m_sign*m_j_max/2.0*t_max_acc*t_max_acc;

  // Position after initial jolt phase.
  const double sa = m_v1*t_max_acc + m_sign*m_j_max*t_max_acc*t_max_acc*t_max_acc/6.0;

  // Find out for how long we can apply a constant acceleration.
  Eigen::Vector3d polynomial;
  polynomial <<
    // Const terms
    sa + t_max_acc*v_max_acc + 0.5*m_sign*m_a_max*m_t1*m_t1 - m_sign*m_j_max*m_t1*m_t1*m_t1/6.0 - s,
    // Linear terms
    v_max_acc + t_max_acc*m_sign*m_a_max,
    // Quadratic terms
    0.5*m_sign*m_a_max;
  Eigen::PolynomialSolver<double, 2> solver;
  solver.compute(polynomial);
  bool has_a_real_root = false;
  // Take the solution for the smaller positive time (the one reached
  // first, causally).
  double t_const_a_max = solver.smallestRealRoot(has_a_real_root);
  if (has_a_real_root && t_const_a_max < 0)
  {
    t_const_a_max = solver.greatestRealRoot(has_a_real_root);
  }
  if (has_a_real_root)
  {
    if (t_const_a_max < 0)
    {
      // Not enough time to reach the full acceleration, need to solve
      // it differently.

      // // Position after initial jolt phase.
      // const double sa = m_v1*TT + m_sign*m_j_max*TT*TT*TT/6.0;
      // // Velocity after initial jolt phase.
      // const double va = m_v1 + m_sign*m_j_max/2.0*TT*TT;
      // // Acceleration after initial jolt phase.
      // const double aa = m_sign*m_j_max*TT;
      // // Position after final jolt phase.
      // const double sb = sa + TT*va + aa/2.0*TT*TT - m_sign*m_j_max*TT*TT*TT/6.0;

      // // sb - s = 0
      // // s = m_v1*TT + m_sign*m_j_max*TT*TT*TT/6.0 + TT*(m_v1 + m_sign*m_j_max/2.0*TT*TT)
      // //   + m_sign*m_j_max/2.0*TT*TT*TT - m_sign*m_j_max*TT*TT*TT/6.0;
      // s = m_sign*m_j_max/2.0*TT*TT*TT
      //   + m_sign*m_j_max/2.0*TT*TT*TT;

      // s = 2*v1*t + j_max*t^3

      Eigen::Vector4d polynomial;
      polynomial <<
        -s,
        2.0*m_v1,
        0.0,
        m_sign*m_j_max;
      Eigen::PolynomialSolver<double, 3> solver;
      solver.compute(polynomial);
      std::vector<double> real_roots;
      solver.realRoots(real_roots);
      if (real_roots.empty())
      {
        // Car changes direction before location is reached.
        return 0;
      }
      else
      {
        std::vector<double>::const_iterator min_pos_it = real_roots.end();
        for (std::vector<double>::const_iterator it = real_roots.begin(); it != real_roots.end(); ++it)
        {
          if (*it >= 0 && (min_pos_it == real_roots.end() || *it < *min_pos_it))
          {
            min_pos_it = it;
          }
        }
        if (min_pos_it == real_roots.end())
        {
          // Impossible without a Delorean and 1.21 Gigawatts.
          return 0;
        }
        // Apply max jolt for this long.
        double tt = *min_pos_it;
        // Velocity before final jolt phase.
        double vb = m_v1 + 0.5*m_sign*m_j_max*tt*tt;
        // Acceleration after initial jolt phase.
        double a_max_eff = tt*m_j_max;
        // Velocity after final jolt phase.
        double vc = vb + m_sign*a_max_eff*tt - 0.5*m_sign*m_j_max*tt*tt;
        return vc;
      }
    }
    else
    {
      // Velocity before final jolt phase.
      double vb = v_max_acc + m_sign*t_const_a_max*m_a_max;
      // Velocity after final jolt phase.
      double vc = vb + m_sign*t_max_acc*m_a_max - m_sign*m_j_max/2.0*t_max_acc*t_max_acc;
      return vc;
    }
  }
  else
  {
    // Car changes direction before location is reached.
    return 0;
  }
}

} // end of ns
} // end of ns
