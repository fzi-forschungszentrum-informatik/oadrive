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
#include <icl_core_logging/Logging.h>
#include <oadrive_core/Trajectory2d.h>
#include <oadrive_core/TrajectoryVelocityInterpolator.h>

using namespace oadrive::core;

int main(int argc, char** argv)
{
  icl_core::logging::LifeCycle llc(argc, argv);

  /* -------------------------------------------------------------
   *   velocity curve interpolation test program
   * ------------------------------------------------------------- */

  Trajectory2d trajectory;
  trajectory.fillWithTestData(1, false, true);

  double startup_velocity = 1;
  double max_velocity = 3.1;
  double max_acc = 0.5;
  double max_dec = 0.4;
  double max_jolt = 0.05;

  TrajectoryVelocityInterpolator::startupAndBrakeProfile(trajectory,
                                                         startup_velocity,
                                                         max_velocity,
                                                         max_acc,
                                                         max_dec,
                                                         max_jolt);

  trajectory.toGnuplot("/tmp/velocity_profile");

  return 0;
}
