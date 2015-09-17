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
 * \date    2015-02-07
 *
 */
//----------------------------------------------------------------------
#include "Interpolator.h"
#include "QuaternionFunctions.h"

namespace oadrive {
namespace core {

Interpolator::Interpolator()
{
}

Interpolator::~Interpolator()
{
}

double Interpolator::interpolateLinear(double value1, double value2, double ratio)
{
  return (value1 * (1.0 - ratio) + value2 * ratio);
}

double Interpolator::interpolateCubic(double value0, double value1, double value2, double value3, double ratio)
{
  // coefficients:
  const double a0 = value3 - value2 - value0 + value1;
  const double a1 = value0 - value1 - a0;
  const double a2 = value2 - value0;
  const double a3 = value1;
  const double ratio_square = ratio * ratio;

  return (a0 * ratio * ratio_square + a1 * ratio_square + a2 * ratio + a3);
}

double Interpolator::interpolateSpline(double value0, double value1, double value2, double value3, double ratio)
{
  // coefficients:
  const double a0 = -0.5*value0 + 1.5*value1 - 1.5*value2 + 0.5*value3;
  const double a1 = value0 - 1.5*value1 + 2*value2 - 0.5*value3;
  const double a2 = -0.5*value0 + 0.5*value2;
  const double ratio_square = ratio * ratio;

  return (a0 * ratio * ratio_square + a1 * ratio_square + a2 * ratio + value1);
}

Position2d Interpolator::interpolateLinear(const Position2d& p1,
                                           const Position2d& p2, double ratio)
{
  Position2d interpolated;
  interpolated.x() = interpolateLinear(p1.x(), p2.x(), ratio);
  interpolated.y() = interpolateLinear(p1.y(), p2.y(), ratio);
  return interpolated;
}

Position2d Interpolator::interpolateCubic(const Position2d& p0, const Position2d& p1,
                                          const Position2d& p2, const Position2d& p3, double ratio)
{
  Position2d interpolated;
  interpolated.x() = interpolateCubic(p0.x(), p1.x(), p2.x(), p3.x(), ratio);
  interpolated.y() = interpolateCubic(p0.y(), p1.y(), p2.y(), p3.y(), ratio);
  return interpolated;
}

Position2d Interpolator::interpolateSpline(const Position2d& p0, const Position2d& p1,
                                           const Position2d& p2, const Position2d& p3, double ratio)
{
  Position2d interpolated;
  interpolated.x() = interpolateSpline(p0.x(), p1.x(), p2.x(), p3.x(), ratio);
  interpolated.y() = interpolateSpline(p0.y(), p1.y(), p2.y(), p3.y(), ratio);
  return interpolated;
}

Quaternion Interpolator::interpolateLinear(const Quaternion& p, const Quaternion& q, double ratio)
{
  return p.slerp(ratio, q);
}

Quaternion Interpolator::interpolateCubic(const Quaternion& p, const Quaternion& a,
                                          const Quaternion& b, const Quaternion& q, double ratio)
{
  const Eigen::Quaterniond one   = p.slerp(ratio, q);
  const Eigen::Quaterniond other = a.slerp(ratio, b);

  return one.slerp(2.*ratio*(1.-ratio), other);
}

Quaternion Interpolator::interpolateSpline(const Quaternion& support_before,
                                           const Quaternion& p, const Quaternion& q,
                                           const Quaternion& support_after, double ratio)
{
  const Eigen::Quaterniond support_a = calculateSupportQuaternion(support_before, p, q);
  const Eigen::Quaterniond support_b = calculateSupportQuaternion(p, q, support_after);
  return interpolateCubic(p, support_a, support_b, q, ratio);
}

Pose2d Interpolator::interpolateLinear(const Pose2d& p1, const Pose2d& p2, double ratio)
{
  Pose2d interpolated;

  // translation part
  interpolated.translation() = interpolateLinear(p1.translation(), p2.translation(), ratio);

  // rotation part
  double yaw = 0.;
  const double yaw_from = PoseTraits<Pose2d>::yaw(p1);
  const double yaw_to   = PoseTraits<Pose2d>::yaw(p2);
  double diff = yaw_to - yaw_from;

  if (std::abs(diff) <= M_PI)
  {
    yaw = yaw_from + diff * ratio;
  }
  else
  {
    if (diff > 0.0)
    {
      diff = 2.0 * M_PI - diff;
    }
    else
    {
      diff = -2.0 * M_PI - diff;
      yaw = yaw_from - diff * ratio;

      if (yaw > M_PI)
      {
        yaw -= 2.0 * M_PI;
      }
      else
      {
        if (yaw < -M_PI)
        {
          yaw += 2.0 * M_PI;
        }
      }
    }
  }
  PoseTraits<Pose2d>::fromOrientationRPY(interpolated, yaw);
  return interpolated;
}

ExtendedPose2d Interpolator::interpolateLinear(const ExtendedPose2d& p1, const ExtendedPose2d& p2, double ratio)
{
  ExtendedPose2d interpolated;
  interpolated.pose() = interpolateLinear(p1.pose(), p2.pose(), ratio);
  if (!isnan(p1.getVelocity()) && !isnan(p2.getVelocity()))
  {
    interpolated.setVelocity(interpolateLinear(p1.getVelocity(), p2.getVelocity(), ratio));
  }
  if (!isnan(p1.getCurvature()) && !isnan(p2.getCurvature()))
  {
    interpolated.setCurvature(interpolateLinear(p1.getCurvature(), p2.getCurvature(), ratio));
  }

  return interpolated;
}

void Interpolator::interpolateLinear(Trajectory2d& trajectory, double ratio)
{
  if ((ratio <= 0) || (ratio >=1) || (trajectory.size() < 2))
  {
    return;
  }

  Trajectory2d interpolated;
  // allocate enough memory to prevent avoidable copy operations
  interpolated.reserve(trajectory.size() + trajectory.size()-1);
  ExtendedPose2d temp;

  // Always add the first point in the trajectory.
  interpolated.push_back(trajectory.front());
  for (std::size_t i=0; i+1<trajectory.size(); ++i)
  {
    temp = interpolateLinear(trajectory[i], trajectory[i+1], ratio);
    interpolated.push_back(temp);
    interpolated.push_back(trajectory[i+1]);
  }

  if (trajectory.curvatureAvailable())
  {
    interpolated.calculateCurvature();
  }

  // save forward flag
  interpolated.isForwardTrajectory() = trajectory.isForwardTrajectory();

  // store result
  trajectory = interpolated;

}

void Interpolator::smoothBSpline(Trajectory2d& trajectory, std::size_t max_steps, double min_change)
{
  if (trajectory.size() < 3)
  {
    return;
  }

  // to save some calculations during interpolation we first
  // invalidate curvature and calculate it once at the end
  const bool recalc_curvature = trajectory.curvatureAvailable();
  if (recalc_curvature)
  {
    trajectory.curvatureAvailable() = false;
  }

  ExtendedPose2d temp1, temp2;
  for (std::size_t step=0; step < max_steps; ++step)
  {
    interpolateLinear(trajectory, 0.5);

    std::size_t i = 2;
    std::size_t u = 0;
    std::size_t n1 = trajectory.size() - 1;

    while (i < n1)
    {
      temp1 = interpolateLinear(trajectory[i - 1], trajectory[i], 0.5);
      temp2 = interpolateLinear(trajectory[i], trajectory[i + 1], 0.5);
      temp1 = interpolateLinear(temp1, temp2, 0.5);
      if (trajectory[i].distance(temp1)> min_change)
      {
        trajectory[i] = temp1;
        ++u;
      }
      i += 2;
    }

    if (u == 0)
    {
      break;
    }
  }
  if (recalc_curvature)
  {
    trajectory.calculateCurvature();
  }
}

Quaternion Interpolator::calculateSupportQuaternion(const Quaternion& o, const Quaternion& p, const Quaternion& q)
{
  return p * std::exp(-0.25 * (std::log(p.inverse() * q) + std::log(p.inverse() * o)));
}

} // end of ns
} // end of ns
