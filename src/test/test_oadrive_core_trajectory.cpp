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
#include <oadrive_core/Interpolator.h>

using namespace oadrive::core;

int main(int argc, char** argv)
{
  icl_core::logging::LifeCycle llc(argc, argv);

  /* -------------------------------------------------------------
   *   Trajectory test program
   * ------------------------------------------------------------- */

  // Test plotting and generating of test data
  {
    Trajectory2d trajectory;

     // with velocity
    trajectory.fillWithTestData(1., true, true);
    trajectory.toGnuplot("/tmp/trajectory_vel");

    // without velocity
    trajectory.clear();
    trajectory.fillWithTestData(1., false, true);
    trajectory.toGnuplot("/tmp/trajectory_no_vel");
  }

  // Test trajectory interpolation
  {
    Trajectory2d trajectory;
    ExtendedPose2d p;

    p.setPosition(0., 0.);
    p.setYaw(0.);
    trajectory.push_back(p);

    p.setPosition(5., 1.);
    p.setYaw(0.4);
    trajectory.push_back(p);

    p.setPosition(7., 3.);
    p.setYaw(0.);
    trajectory.push_back(p);

    p.setPosition(11., -1.);
    p.setYaw(-0.3);
    trajectory.push_back(p);

    trajectory.toGnuplot("/tmp/raw_test1"); // raw data
    Interpolator::interpolateLinear(trajectory, 0.5);
    trajectory.toGnuplot("/tmp/res_test1"); // resulting data
  }

  // Test trajectory smoothing
  {
    Trajectory2d trajectory;
    ExtendedPose2d p;

    p.setPosition(0., 0.);
    p.setYaw(0.);
    trajectory.push_back(p);

    p.setPosition(5., 1.);
    p.setYaw(0.4);
    trajectory.push_back(p);

    p.setPosition(7., 3.);
    p.setYaw(0.);
    trajectory.push_back(p);

    p.setPosition(11., -1.);
    p.setYaw(-0.3);
    trajectory.push_back(p);

    trajectory.toGnuplot("/tmp/raw_test2"); // raw data
    Interpolator::smoothBSpline(trajectory);
    trajectory.toGnuplot("/tmp/res_test2"); // resulting data
  }


  // Test trajectory appending
  {
    Trajectory2d t1, t2;
    ExtendedPose2d p;

    // fill t1
    p.setPosition(0., 0.);
    t1.push_back(p);

    p.setPosition(1., 0.);
    t1.push_back(p);

    p.setPosition(2., 0.);
    t1.push_back(p);

    p.setPosition(3., 0.);
    t1.push_back(p);

    // fill t2
    p.setPosition(6., 0.);
    t2.push_back(p);

    p.setPosition(7., 0.);
    t2.push_back(p);

    p.setPosition(8., 0.);
    t2.push_back(p);

    p.setPosition(9., 0.);
    t2.push_back(p);


    t1.toGnuplot("/tmp/raw_test3_1"); // raw data
    t2.toGnuplot("/tmp/raw_test3_2"); // raw data
    t1.append(t2);
    t1.toGnuplot("/tmp/res_test3"); // resulting data
  }


  // Test trajectory shortcutting
  {
    Trajectory2d t_forward, t_backward;
    ExtendedPose2d p;

    // forward test
    t_forward.isForwardTrajectory() = true;

    p.setPosition(0., 0.);
    t_forward.push_back(p);

    p.setPosition(1., 0.);
    t_forward.push_back(p);

    p.setPosition(2., 0.);
    t_forward.push_back(p);

    p.setPosition(3., 0.);
    t_forward.push_back(p);

    // -- here the trajectory should be shortcut --
    p.setPosition(2., 0.);
    t_forward.push_back(p);

    p.setPosition(2.5, 0.);
    t_forward.push_back(p);

    p.setPosition(3.5, 0.);
    t_forward.push_back(p);

    p.setPosition(4.5, 0.);
    t_forward.push_back(p);


    t_forward.toGnuplot("/tmp/raw_test4_1"); // raw data
    t_forward.shortcut();
    t_forward.toGnuplot("/tmp/res_test4_1"); // resulting data

    // backward test
    t_backward.isForwardTrajectory() = false;

    p.setPosition(4.5, 0.);
    t_backward.push_back(p);

    p.setPosition(3.5, 0.);
    t_backward.push_back(p);

    p.setPosition(2.5, 0.);
    t_backward.push_back(p);

    p.setPosition(2., 0.);
    t_backward.push_back(p);

    // -- here the trajectory should be shortcut --
    p.setPosition(3., 0.);
    t_backward.push_back(p);

    p.setPosition(2., 0.);
    t_backward.push_back(p);

    p.setPosition(1., 0.);
    t_backward.push_back(p);

    p.setPosition(0., 0.);
    t_backward.push_back(p);


    t_backward.toGnuplot("/tmp/raw_test4_2"); // raw data
    t_backward.shortcut();
    t_backward.toGnuplot("/tmp/res_test4_2"); // resulting data
  }
  return 0;
}
