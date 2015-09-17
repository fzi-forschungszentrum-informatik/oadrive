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
*/
//----------------------------------------------------------------------

#include "TrajectoryVelocityInterpolator.h"

#include <oadrive_core/VelocityCurve.h>

#include <utility>
#include <iostream>
#include <cmath>
#include <fstream>

namespace oadrive {
namespace core {

void TrajectoryVelocityInterpolator::constantVelocityMotion(Trajectory2d& trajectory,
                                                            const double v)
{
  for (Trajectory2d::iterator cur=trajectory.begin(); cur != trajectory.end(); ++cur)
  {
    cur->setVelocity(v);
  }
  trajectory.velocityAvailable() = true;
}

bool TrajectoryVelocityInterpolator::startupAndBrakeProfile(Trajectory2d& trajectory,
                                                            const double v_start,
                                                            const double v_desired,
                                                            const double a_motor,
                                                            const double a_brake,
                                                            const double max_jolt)
{
  VelocityCurve reverse_brake_curve(0., v_desired, std::abs(a_brake), max_jolt);
  // the reachable velocity if we would drive the reverse brake curve for half of the trajectory
  const double v_brake_reachable = reverse_brake_curve.vDiffMax(trajectory.length() / 2.);
  const double v_reachable = std::min(v_brake_reachable, v_desired);

  const bool start_ok = startupProfile(trajectory, v_start, v_reachable, a_motor, max_jolt, true);
  const bool brake_ok = brakeProfile(trajectory, v_reachable, a_brake, max_jolt, false);

  return start_ok && brake_ok;
}

void TrajectoryVelocityInterpolator::plotTrajectoryVelocity(const Trajectory2d& trajectory, const std::string& filename)
{
  printf("\nWriting trajectory velocities to logfile %s\n", filename.c_str());

  std::ofstream logfile(filename.c_str());
  logfile << "# Data: index , velocity" << std::endl;
  for (size_t i=0; i<trajectory.size(); ++i)
  {
    logfile << i << " " << trajectory[i].getVelocity() << std::endl;
  }

  logfile << std::endl;
  logfile.close();
}

template <class trajectory_iterator_t>
trajectory_iterator_t TrajectoryVelocityInterpolator::constantJoltAcceleratedMotion(trajectory_iterator_t start,
                                                                                    trajectory_iterator_t end,
                                                                                    const double v_start,
                                                                                    const double v_end,
                                                                                    const double max_acceleration,
                                                                                    const double max_jolt,
                                                                                    const bool overwrite)
{
  VelocityCurve curve(v_start, v_end, max_acceleration, max_jolt);
  double s = 0.;
  double t = curve.ts(s);
  double v = curve.v(t);
  bool   target_velocity_reached = false;

  start->setVelocity(v);

  trajectory_iterator_t cur, prev, ret;
  ret = end;

  for (cur = start+1, prev = start; cur != end; ++cur, ++prev)
  {
    if (target_velocity_reached && overwrite)
    {
      cur->setVelocity(v_end);
      continue;
    }

    s = s + (cur->getPosition() - prev->getPosition()).norm();
    t = curve.ts(s);
    v = curve.v(t);

    if (!overwrite && cur->getVelocity() < v)
    {
      // we encountered a point with a lower set target velocity that must not be overwritten. Acceleration ends thus prematurely on prev.
      ret = prev;
      break;
    }

    cur->setVelocity(v);

    if (t >= curve.timeToTargetVelocity())
    {
      // target velocity has been reached.
      target_velocity_reached = true;
      ret = cur;
      if (!overwrite) break;
    }
  }
  return ret;
}

template
Trajectory2d::ContainerType::iterator
TrajectoryVelocityInterpolator::constantJoltAcceleratedMotion(Trajectory2d::ContainerType::iterator start,
                                                              Trajectory2d::ContainerType::iterator end,
                                                              const double v_start,
                                                              const double v_end,
                                                              const double max_acceleration,
                                                              const double jolt,
                                                              const bool overwrite);
template
Trajectory2d::ContainerType::reverse_iterator
TrajectoryVelocityInterpolator::constantJoltAcceleratedMotion(Trajectory2d::ContainerType::reverse_iterator start,
                                                              Trajectory2d::ContainerType::reverse_iterator end,
                                                              const double v_start,
                                                              const double v_end,
                                                              const double max_acceleration,
                                                              const double jolt,
                                                              const bool overwrite);

} // end of ns
} // end of ns
