// -- BEGIN LICENSE BLOCK ----------------------------------------------
// This program is free software licensed under the CDDL
// (COMMON DEVELOPMENT AND DISTRIBUTION LICENSE Version 1.0).
// You can find a copy of this license in LICENSE in the top
// directory of the source code.
//
// © Copyright 2018 FZI Forschungszentrum Informatik, Karlsruhe, Germany
// -- END LICENSE BLOCK ------------------------------------------------

//
// Created by fabian on 02.08.17.
//
#include <iostream>
#include <oadrive_control/DriverModule.h>
#include <oadrive_control/LateralController.h>
#include <oadrive_world/TrajectoryFactory.h>
#include <oadrive_util/Config.h>
#include <time.h>
#include <oadrive_world/Environment.h>

using namespace oadrive::core;
using namespace oadrive::control;
using namespace oadrive::world;
using namespace oadrive::util;

#define NUM_PATCHES 5

int main(int argc, char **argv)
{
  oadrive::util::Config::setConfigPath(
  "/home/fabian/oadrive/Aufzeichnungen/AADC2017/road_detection_test/config/",
          "ABRA");


  PatchPtrList street;
  Environment::init();
  Environment::getInstance()->updateCarPose(ExtendedPose2d(-0.3f, 0.f, 0.f));


  for (int i = 0; i < NUM_PATCHES; i++)
  {
    ExtendedPose2d *pose = new ExtendedPose2d(0.25f + float(i) * 0.5f, 0.f, 0.f);
    Patch *patch = new Patch(STRAIGHT, *pose);
    std::cout << "x: " << patch->getX() << " y: " << patch->getY() << " yaw: " << patch->getY()
              << std::endl;
    street.push_back(boost::shared_ptr<Patch>(patch));
  }

  int i = 0;
  for (PatchPtr p : street)
  {
    p->setPatchID(i++);
  }

  ExtendedPose2d *pose = new ExtendedPose2d(0.25f + float(1.05) * 0.5f, 0.f, 0.f);
  Patch *patch = new Patch(STRAIGHT, *pose);
  std::cout << "x: " << patch->getX() << " y: " << patch->getY() << " yaw: " << patch->getY()
            << std::endl;
  auto p = boost::shared_ptr<Patch>(patch);
  p->setPatchID(0);
  street.push_back(p);

  int test =   Environment::getInstance()->getStreet()->size();
  int test2 = street.size();

  for (PatchPtrList::iterator it = street.begin(); it != street.end(); it++)
  {
    Environment::getInstance()->addPatch(*it);
  }

  std::cout << "------------ \n size: " <<   Environment::getInstance()->getStreet()->size() << std::endl;


  for (PatchPtrList::const_iterator it = (*  Environment::getInstance()->getStreet()).begin();
       it != (*Environment::getInstance()->getStreet()).end(); it++)
  {
    std::cout << "x: " << (*it)->getX() << " y: " << (*it)->getY() << " yaw: " << (*it)->getY()
              << std::endl;
  }


  TrajectoryFactory *fctry = new TrajectoryFactory();

  double start = clock();
  Environment::getInstance()->generateNextTrajectory();
  double execTime = (clock() - start) / CLOCKS_PER_SEC;
  std::cout << "Time: " << execTime << std::endl;

  MultiTrajectory traj = Environment::getInstance()->getMultiTrajectory();

  std::cout << "-----------------" << std::endl;

  for (int i = 0; i < traj.trajectories[0].size(); i++)
  {
    std::cout << "x: " << traj.trajectories[0].at(i).getX() << " y: "
              << traj.trajectories[0].at(i).getY() << " yaw: "
              << traj.trajectories[0].at(i).getYaw() << std::endl;
  }


  return 0;
}