// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// This program is free software licensed under the CDDL
// (COMMON DEVELOPMENT AND DISTRIBUTION LICENSE Version 1.0).
// You can find a copy of this license in LICENSE in the top
// directory of the source code.
//
// © Copyright 2018 FZI Forschungszentrum Informatik, Karlsruhe, Germany
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

  // Test trajectory serialisation:
  {
	Trajectory2d t1, t2;
	ExtendedPose2d p1, p2;

    p1.setPose( double(rand()) / double(RAND_MAX),
			double(rand()) / double(RAND_MAX),
			double(rand()) / double(RAND_MAX) );
    t1.push_back(p1);
    p1.setPose( double(rand()) / double(RAND_MAX),
			double(rand()) / double(RAND_MAX),
			double(rand()) / double(RAND_MAX) );
    t1.push_back(p1);
    p1.setPose( double(rand()) / double(RAND_MAX),
			double(rand()) / double(RAND_MAX),
			double(rand()) / double(RAND_MAX) );
    t1.push_back(p1);

	std::stringstream sstr;
	sstr << p1;
	sstr >> p2;

	std::cout << "Pose generated: " << p1;
	std::cout << "Pose seriealized and deserialized: " << p2;

	sstr.str("");
	sstr << t1;
	//std::cout << "STREAM: " << sstr.str() << std::endl;
	sstr >> t2;

	std::cout << "Trajectory generated:\n" << t1;
	std::cout << "Trajectory seriealized and deserialized:\n" << t2;
  }

  //test Trajectory post at distance from
  {
    Trajectory2d traj;
    ExtendedPose2d p;
    p.setPosition(-1,0);
    traj.push_back(p);
    p.setPosition(0,0);
    traj.push_back(p);
    p.setPosition(1,0);
    traj.push_back(p);
    p.setPosition(2,0);
    traj.push_back(p);
    p.setPosition(3,0);
    traj.push_back(p);
    p.setPosition(4,0);
    traj.push_back(p);
    ExtendedPose2d result;
    traj.poseAtDistanceFrom(0,0.5,result);
    bool test1 = result.getX() == -0.5;
    traj.poseAtDistanceFrom(0,2,result);
    bool test2 = result.getX() == 1;
    traj.poseAtDistanceFrom(0,2.5,result);
    bool test3 = result.getX() == 1.5;
    traj.poseAtDistanceFrom(1,2.5,result);
    bool test4 = result.getX() == 2.5;
    traj.poseAtDistanceFrom(2,-1.1,result);
    bool test5 = (result.getX() - (-0.1))<0.00001;
    traj.poseAtDistanceFrom(5,-0.9,result);
    bool test6 = result.getX() == 3.1;
    if(test1&&test2&&test3&&test4&&test5&&test6)
    {
      std::cout<<"Test Trajectory pose at distance from passed"<<std::endl;
    }
    else
    {
      std::cout<<"Test Trajectory pose at distance from NOT passed"<<std::endl;
      std::cout<<"The results are: "<<"Test1: "<<test1<<" Test2: "<<test2<<" Test3: "<<test3<<" Test4: "<<test4<<"Test5: "<<test5<<"Test6: "<<test6<<std::endl;
    }


  }
  //test resampling sorry not automated...
  {
    Trajectory2d traj;
    ExtendedPose2d p;
    p.setPosition(-1,0);
    traj.push_back(p);
    p.setPosition(0,0);
    traj.push_back(p);
    p.setPosition(1,0);
    traj.push_back(p);
    p.setPosition(2,0);
    traj.push_back(p);
    p.setPosition(3,0);
    traj.push_back(p);
    p.setPosition(4,0);
    traj.push_back(p);
    p.setPosition(4.1,0);
    traj.push_back(p);
    p.setPosition(4.2,0);
    traj.push_back(p);
    p.setPosition(4.3,0);
    traj.push_back(p);
    p.setPosition(4.4,0);
    traj.push_back(p);
    p.setPosition(4.5,0);
    traj.push_back(p);
    traj.resample(0.3);
    std::cout<<"Resampled Traj"<<std::endl<<traj;
    std::cout<<"Distance should be 0.3m beteween the points except the last point"<<std::endl;
  }


  return 0;
}
