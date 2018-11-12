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
 * \author  David Zimmerer <dzimmerer@gmail.com>
 * \author  Micha Pfeiffer <ueczz@student.kit.edu>
 * \date    2016-01-16
 * 
 * \author  Simon Roesler <simon.roesler@student.kit.edu>
 * \date    2018
 * 
 * \author  Mark Hueneberg <hueneber@fzi.de>
 * \date    2018
 *
 */
//----------------------------------------------------------------------

#include "TrajectoryFactory.h"
#include <oadrive_core/Interpolator.h>
#include <oadrive_world/worldLogging.h>
#include <boost/filesystem.hpp>
#include <iostream>
#include <sstream>

#include "Environment.h"
//#include "TrajectoryDatabase.h"

using namespace oadrive::core;
using namespace boost::filesystem;
using icl_core::logging::endl;
using icl_core::logging::flush;

namespace oadrive
{
namespace world
{

TrajectoryFactory::TrajectoryFactory()
        : mTrajectoryMode(PATCH_TRAJECTORY), mDebugMode(false), mTrajectoryCounter(0), mDebugFolder("/home/aadc/Desktop/dbg_traj/")
{}


/******************************************************************************/
/***************************** public methods *********************************/
/******************************************************************************/

void TrajectoryFactory::clearOldTraj()
{
  mOldTraj.clear();
}

void TrajectoryFactory::setFixedTrajectory(enumTrajectory trajName, double scaleFactor)
{
  bool appendToCar = true;

  switch (trajName)
  {
    case TRAJ_PULLOUT_PARALLEL:
      mTrajectoryName = "pullout_parallel";
      break;
    case TRAJ_PARKING_PARALLEL:
      // appendToCar = false;	// append to patch, not car!
      mTrajectoryName = "park_parallel";
      break;
    case TRAJ_PARKING_CROSS:
      mTrajectoryName = "park_cross";
      break;
    case TRAJ_PULLOUT_CROSS_LEFT:
      mTrajectoryName = "pullout_cross_left";
      break;
    case TRAJ_PULLOUT_CROSS_RIGHT:
      mTrajectoryName = "pullout_cross_right";
      break;
    case TRAJ_BACKUP_OBSTACLE:
      mTrajectoryName = "backup_obstacle";
      break;
    case TRAJ_FORWARD_SMALL:
      mTrajectoryName = "forward_small";
      break;
    case TRAJ_U_TURN:
      mTrajectoryName = "turn_slow";
      break;
    case TRAJ_MIDDLE:
      mTrajectoryName = "middle";
      break;
    default:
      mTrajectoryName = "";
      break;
  }

  LOGGING_INFO(worldLogger, "Trajectory name: " << mTrajectoryName << endl);

  if (TrajectoryDatabase::hasTrajectory(mTrajectoryName))
  {
    MultiTrajectory multiTraj = TrajectoryDatabase::getTrajectory(mTrajectoryName);
    LOGGING_INFO(worldLogger, "\tTrajectory size: " << multiTraj.trajectories.size() << endl);
    if (scaleFactor > 0)
    {
      LOGGING_INFO(worldLogger, "\tHey Ho scale traj with : " << scaleFactor << endl);
      for (std::vector<Trajectory2d>::iterator it =
              multiTraj.trajectories.begin();
           it != multiTraj.trajectories.end(); it++)
      {
        for (unsigned int i = 0; i < (*it).size(); ++i)
        {
          (*it)[i].setY((*it)[i].getY() * scaleFactor);
          (*it)[i].setX((*it)[i].getX() * scaleFactor);
        }
      }
    }

    if (appendToCar)
    {
      multiTraj = rotateAndMoveTrajectory(multiTraj, Environment::getInstance()->getCarPose());
    }
    LOGGING_INFO(worldLogger, "\tTrajectory size after rotateAndMove: "
            << multiTraj.trajectories.size() << endl);

    setFixedTrajectory(multiTraj);
    mCurrentMultiTraj = multiTraj;
    LOGGING_INFO(worldLogger, "\tTrajectory size after copy: "
            << mCurrentMultiTraj.trajectories.size()
            << endl);
  }
  else
  {
    std::stringstream sstr;
    sstr << "Cannot find trajectory in database for manuever " << trajName
         << ".";
    throw (sstr.str().c_str());
  }
}

void TrajectoryFactory::setGenerateFromPatches()
{
  // Remember that we are currently in PATCH_TRAJECTORY mode:
  mTrajectoryMode = PATCH_TRAJECTORY;
}

Trajectory2d TrajectoryFactory::generateFromPatches(const Environment &env, bool useOld)
{
  LOGGING_INFO(worldLogger, "[TF] Generating patch trajectory." << endl);

  if (mDebugMode)
  {
    mTrajectoryCounter++;
  }

  Trajectory2d newTraj;

  PatchPtr lastPastPatch;
  createTrajStartPoints(env, &newTraj, lastPastPatch);
  createTrajFromPatches(&newTraj, lastPastPatch);
  if (mDebugMode)
  {
    writeToFile(newTraj, "raw");
  }

  // validateTrajPoints(&newTraj);
  if (mDebugMode)
  {
    writeToFile(newTraj, "rawWorkaround");
  }

  // there can be ZigZag in the trajectory. for example if we are going back at a crossing
  removeZigZag(newTraj);
  // // LOGGING_INFO(worldLogger, "Remove ZigZag" << endl);

  incorporateOldTraj(env, &newTraj, useOld);

  smoothTrajectory(&newTraj);

  LOGGING_INFO(worldLogger, "Traj. is set. Traj. size is: " << newTraj.size() << endl);

  return newTraj;
}


Trajectory2d TrajectoryFactory::generateTestTrajectory(std::string trajType)
{
  Trajectory2d traj;
  if (trajType == "rectangle")
  {
    float sizeX = 3.0;  // circle of 1.5 meters radius.
    float sizeY = 0.7;
    float x = 0;
    float y = 0;
    float stepSize = 0.1;
    for (; x < sizeX - stepSize; x += stepSize)
    {
      ExtendedPose2d pose(x, -y, 0);
      traj.push_back(pose);
    }
    for (; y < sizeY - stepSize; y += stepSize)
    {
      ExtendedPose2d pose(x, -y, 0);
      traj.push_back(pose);
    }
    for (; x > 0; x -= stepSize)
    {
      ExtendedPose2d pose(x, -y, 0);
      traj.push_back(pose);
    }
    for (; y > 0 + stepSize; y -= stepSize)
    {
      ExtendedPose2d pose(x, -y, 0);
      traj.push_back(pose);
    }
  }
  else if (trajType == "uTurn")
  {
    float sizeX = 3.0;  // circle of 1.5 meters radius.
    float sizeY = 0.7;
    float x = 0;
    float y = 0;
    float stepSize = 0.1;
    for (; x < sizeX - stepSize; x += stepSize)
    {
      ExtendedPose2d pose(x, -y, 0);
      traj.push_back(pose);
    }
    for (; y < sizeY - stepSize; y += stepSize)
    {
      ExtendedPose2d pose(x, -y, 0);
      traj.push_back(pose);
    }
    for (; x > 0; x -= stepSize)
    {
      ExtendedPose2d pose(x, -y, 0);
      traj.push_back(pose);
    }
  }
  else if (trajType == "uTurnLeft")
  {
    float sizeX = 3.0;  // circle of 1.5 meters radius.
    float sizeY = 0.7;
    float x = 0;
    float y = 0;
    float stepSize = 0.1;
    for (; x < sizeX - stepSize; x += stepSize)
    {
      ExtendedPose2d pose(x, y, 0);
      traj.push_back(pose);
    }
    for (; y < sizeY - stepSize; y += stepSize)
    {
      ExtendedPose2d pose(x, y, 0);
      traj.push_back(pose);
    }
    for (; x > 0; x -= stepSize)
    {
      ExtendedPose2d pose(x, y, 0);
      traj.push_back(pose);
    }
  }
  else if (trajType == "halfCircle")
  {
    float radius = 2.0;  // circle of 1.5 meters radius.
    float stepSize = M_PI * 1.0 / 36;
    for (float angle = -M_PI / 2; angle < M_PI / 2 - stepSize;
         angle += stepSize)
    {
      ExtendedPose2d pose(radius * sin(angle), -radius + radius * cos(angle),
                          0);
      traj.push_back(pose);
    }

  }
  else if (trajType == "oval")
  {
    float radius = 1.5;
    float longSection = 1.5;
    float shortSection = 0.9;
    float stepSize = M_PI * 1.0 / 36;
    // long section:
    for (float x = 0; x < longSection; x += 0.2)
    {
      ExtendedPose2d pose(x, 0, 0);
      traj.push_back(pose);
    }
    for (float angle = -M_PI / 2; angle < 0 - stepSize; angle += stepSize)
    {
      ExtendedPose2d pose(longSection + radius * cos(angle), radius + radius * sin(angle), 0);
      traj.push_back(pose);
    }
    for (float y = 0; y < shortSection; y += 0.2)
    {
      ExtendedPose2d pose(longSection + radius, radius + y, 0);
      traj.push_back(pose);
    }
    for (float angle = 0; angle < M_PI / 2 - stepSize; angle += stepSize)
    {
      ExtendedPose2d pose(longSection + radius * cos(angle),
                          radius + shortSection + radius * sin(angle), 0);
      traj.push_back(pose);
    }
    for (float x = longSection; x > 0; x -= 0.2)
    {
      ExtendedPose2d pose(x, shortSection + 2 * radius, 0);
      traj.push_back(pose);
    }
    for (float angle = M_PI / 2; angle < M_PI - stepSize; angle += stepSize)
    {
      ExtendedPose2d pose(radius * cos(angle), radius + shortSection + radius * sin(angle), 0);
      traj.push_back(pose);
    }
    for (float y = shortSection; y > 0; y -= 0.2)
    {
      ExtendedPose2d pose(-radius, radius + y, 0);
      traj.push_back(pose);
    }
    for (float angle = M_PI; angle < 3 * M_PI / 2 - stepSize; angle += stepSize)
    {
      ExtendedPose2d pose(radius * cos(angle), radius + radius * sin(angle), 0);
      traj.push_back(pose);
    }
  }
  else if (trajType == "forwardSmall")
  {
    float distance = 0.5f;
    float numberOfPoints = 5.0f;
    for (float x = 0; x <= distance; x += distance / numberOfPoints)
    {
      ExtendedPose2d pose(x, 0, 0);
      traj.push_back(pose);
    }
  }
  else if (trajType == "2x2curve")
  {
    float distance = 1.0f;
    float numberOfPoints = 5.0f;
    float x;
    for (x = 0; x <= distance; x += distance / numberOfPoints)
    {
      ExtendedPose2d pose(x, 0, 0);
      traj.push_back(pose);
    }

    float radius = 0.3 + sqrt(2); // outer 2x2 curve radius?
    float stepSize = M_PI * 1.0 / 18;
    for (float angle = 0; angle < M_PI / 2 - stepSize; angle += stepSize)
    {
      ExtendedPose2d pose(x + radius * sin(angle), radius - radius * cos(angle), 0);
      traj.push_back(pose);
    }

  }
  else if (trajType == "sharpRight")
  {
    float radius = 1.0;    // circle of 1.5 meters radius.
    float stepSize = M_PI * 1.0 / 18;
    for (float angle = 0; angle < M_PI / 2 - stepSize; angle += stepSize)
    {
      ExtendedPose2d pose(radius * sin(angle), -radius + radius * cos(angle), 0);
      traj.push_back(pose);
    }
  }
  else if (trajType == "right")
  {
    float radius = 2.0;    // circle of 1.5 meters radius.
    float stepSize = M_PI * 1.0 / 36;
    for (float angle = 0; angle < M_PI / 2 - stepSize; angle += stepSize)
    {
      ExtendedPose2d pose(radius * sin(angle), -radius + radius * cos(angle), 0);
      traj.push_back(pose);
    }
  }
  else if (trajType == "sharpLeft")
  {
    float radius = 1;    // circle of 1.5 meters radius.
    float stepSize = M_PI * 1.0 / 36;
    for (float angle = 0; angle < M_PI / 2 - stepSize; angle += stepSize)
    {
      ExtendedPose2d pose(radius * sin(angle), radius - radius * cos(angle), 0);
      traj.push_back(pose);
    }
  }
  else if (trajType == "left")
  {
    float radius = 2.0;    // circle of 1.5 meters radius.
    float stepSize = M_PI * 1.0 / 36;
    for (float angle = 0; angle < M_PI / 2 - stepSize; angle += stepSize)
    {
      ExtendedPose2d pose(radius * sin(angle), radius - radius * cos(angle), 0);
      traj.push_back(pose);
    }
  }
  else if (trajType == "backwardsSmall")
  {
    float distance = -0.15f;
    float numberOfPoints = 5.0f;
    for (float x = 0; x >= distance; x += distance / numberOfPoints)
    {
      ExtendedPose2d pose(x, 0, 0);
      traj.push_back(pose);
    }
    traj.isForwardTrajectory() = false;
  }
  else
  {               // circle by default
    float radius = 2.0;  // circle of 1.5 meters radius.
    float stepSize = M_PI * 2.0 / 36;
    for (float angle = -stepSize; angle < M_PI * 2.0 - stepSize;
         angle += stepSize)
    {
      ExtendedPose2d pose(radius * sin(angle), -radius + radius * cos(angle),
                          0);
      traj.push_back(pose);
    }
  }
  return traj;
}

core::Trajectory2d TrajectoryFactory::getJunctionTrajectory(TurnDirection turnDirection, LaneType lane)
{
  Trajectory2d trajectory;
  // First drive length of the car before executing motion designated for junction

  if (turnDirection == STRAIGHT_TURN)
  {
    float stepSize = 0.2; //step with 20cm
    for (float x = 0.0; x < PATCH_LENGTHS[CROSS_SECTION]; x += stepSize)
    {
      trajectory.push_back(ExtendedPose2d(x, 0, 0));
    }
  }
  else if (turnDirection == LEFT_TURN)
  {
    // double radius = 0.75;
    // float stepSize = M_PI * 1.0 / 36;
    // for (double angle = 0; angle < M_PI / 2 - stepSize; angle += stepSize)
    // {
    //   ExtendedPose2d pose(-0.25 + radius * sin(angle), radius - radius * cos(angle), angle);
    //   trajectory.push_back(pose);
    // }
    float xOffset = 0;
    if (lane == LANE_LEFT) {
      xOffset = -0.5;
    } else if (lane == SWITCH_TARGET_LANE) {
      xOffset = -0.6;
    } else if (lane == LANE_CENTER) {
      xOffset = -0.25;
    }

    double radius = 0.5;
    float stepSize = M_PI * 1.0 / 36;
    for (double angle = 0; angle < M_PI / 2 - stepSize; angle += stepSize)
    {
      ExtendedPose2d pose(xOffset + 0.15 + radius * sin(angle), radius - radius * cos(angle), angle);
      trajectory.push_back(pose);
    }

    // Leave the intersection straight
    for (double y = 0; y <= 0.5; y += 0.1) {
      trajectory.push_back(ExtendedPose2d(xOffset + 0.15 + radius, radius + y, M_PI / 2));
    }
  }
  else if (turnDirection == RIGHT_TURN)
  {
    float xOffset = 0;
    if (lane == LANE_LEFT) {
      xOffset = 0.5;
    } else if (lane == SWITCH_TARGET_LANE) {
      xOffset = 0.6;
    } else if (lane == LANE_CENTER) {
      xOffset = 0.25;
    }

    // trajectory.push_back(ExtendedPose2d(CAR_LENGTH, 0, 0));
    double radius = 0.5;
    float stepSize = M_PI * 1.0 / 36;
    for (double angle = 0; angle < M_PI / 2 - stepSize; angle += stepSize)
    {
      // ExtendedPose2d pose((-radius / 2) + radius * sin(angle), (-radius / 2) - radius + radius * cos(angle), -angle);
      ExtendedPose2d pose(xOffset + -0.25 + radius * sin(angle), -radius + radius * cos(angle), -angle);
      trajectory.push_back(pose);
    }
  }
  else
  {
    //LOGGING_WARN(worldLogger, "Unable to get trajectory for junction: Unknown TurnDirection provided: " << turnDirection << endl);
  }

  return trajectory;
}

Trajectory2d TrajectoryFactory::generatePulloutLeft()
{
  Trajectory2d pulloutLeft = TrajectoryDatabase::getSingleTrajectory("pullout_cross_left");

  ExtendedPose2d carPose = Environment::getInstance()->getCarPose();
  float yaw = carPose.getYaw();
  for (size_t i = 0; i < pulloutLeft.size(); i++)
  {
    double x = pulloutLeft.at(i).getX();
    double y = pulloutLeft.at(i).getY();

    // Rotate point by angle of car pose:
    ExtendedPose2d rotated(x * cos(yaw) - y * sin(yaw), x * sin(yaw) + y * cos(yaw), 0);
    // Add car position:
    pulloutLeft.at(i).setX(rotated.getX() + carPose.getX());
    pulloutLeft.at(i).setY(rotated.getY() + carPose.getY());
    pulloutLeft.at(i).setYaw(fmod(pulloutLeft.at(i).getYaw() + carPose.getYaw(), 2 * M_PI));
  }

  return pulloutLeft;
}

Trajectory2d TrajectoryFactory::generatePulloutRight()
{
  Trajectory2d pulloutRight = TrajectoryDatabase::getSingleTrajectory("pullout_cross_right");

  ExtendedPose2d carPose = Environment::getInstance()->getCarPose();
  float yaw = carPose.getYaw();
  for (size_t i = 0; i < pulloutRight.size(); i++)
  {
    double x = pulloutRight.at(i).getX();
    double y = pulloutRight.at(i).getY();

    // Rotate point by angle of car pose:
    ExtendedPose2d rotated(x * cos(yaw) - y * sin(yaw), x * sin(yaw) + y * cos(yaw), 0);
    // Add car position:

    pulloutRight.at(i).setX(rotated.getX() + carPose.getX());
    pulloutRight.at(i).setY(rotated.getY() + carPose.getY());
    pulloutRight.at(i).setYaw(fmod(pulloutRight.at(i).getYaw() + carPose.getYaw(), 2 * M_PI));
  }

  return pulloutRight;
}

MultiTrajectory TrajectoryFactory::generateRamp()
{
  MultiTrajectory multiTraj;
  ExtendedPose2d carPose = Environment::getInstance()->getCarPose();

  // Generate straight 3.0m

  float normCarX = 1.0 * cos(carPose.getYaw());
  float normCarY = 1.0 * sin(carPose.getYaw());

  const float upperLength = 1.15;
  const float straightLength = 3.2;
  const float radius = 1.85;


  float normOrtoCarX = 1.0 * cos(carPose.getYaw() + M_PI_2);
  float normOrtoCarY = 1.0 * sin(carPose.getYaw() + M_PI_2);
  
  ExtendedPose2d nextWayPoint = carPose + (straightLength + RAMP_START_OFFSET) * Position2d(normCarX, normCarY);
  Trajectory2d straight1;
  straight1.push_back(carPose);
  straight1.push_back(nextWayPoint);
  straight1.resample(0.1);
  multiTraj.trajectories.push_back(straight1);



  // Trajectory2d halfCircle = TrajectoryDatabase::getSingleTrajectory("halfcircle2m");

  Trajectory2d halfCircle = createCircle(radius);
  mirrorAtY(halfCircle);
  moveTrajectoryToStart(halfCircle, nextWayPoint);
  multiTraj.trajectories.push_back(halfCircle);

  Trajectory2d straight3;
  nextWayPoint = nextWayPoint + radius * Position2d(normOrtoCarX, normOrtoCarY) + radius * Position2d(normCarX, normCarY);
  nextWayPoint.setYaw(nextWayPoint.getYaw() + M_PI / 2);
  straight3.push_back(nextWayPoint);
  nextWayPoint = nextWayPoint + upperLength * Position2d(normOrtoCarX, normOrtoCarY);
  straight3.push_back(nextWayPoint);
  straight3.resample(0.1);
  multiTraj.trajectories.push_back(straight3);

  // moveTrajectoryToStart(halfCircle, nextWayPoint);


  Trajectory2d halfCircle2 = createCircle(radius);
  mirrorAtY(halfCircle2);
  moveTrajectoryToStart(halfCircle2, nextWayPoint);
  multiTraj.trajectories.push_back(halfCircle2);

  // Trajectory2d straight4;
  // nextWayPoint = nextWayPoint - radius * Position2d(normOrtoCarX, normOrtoCarY) - radius * Position2d(normCarX, normCarY);
  // nextWayPoint.setYaw(nextWayPoint.getYaw() + M_PI / 2);
  // straight4.push_back(nextWayPoint);
  // nextWayPoint = nextWayPoint + 1.0 * Position2d(normOrtoCarX, normOrtoCarY);
  // straight4.push_back(nextWayPoint);
  // multiTraj.trajectories.push_back(straight4);
  

  Trajectory2d straight4;
  
  nextWayPoint = nextWayPoint + radius * Position2d(normOrtoCarX, normOrtoCarY) - radius * Position2d(normCarX, normCarY);
  nextWayPoint.setYaw(nextWayPoint.getYaw() + M_PI / 2);
  straight4.push_back(nextWayPoint);
  
  nextWayPoint = nextWayPoint - straightLength * Position2d(normCarX, normCarY);
  straight4.push_back(nextWayPoint);
  straight4.resample(0.1);
  multiTraj.trajectories.push_back(straight4);

  // Trajectory2d straight2;
  // ExtendedPose2d nextWayPoint2 = nextWayPoint + 5.0 * Position2d(normOrtoCarX, normOrtoCarY);
  // nextWayPoint2.setYaw(nextWayPoint2.getYaw() + M_PI);
  // straight2.push_back(nextWayPoint2);
  // ExtendedPose2d nextWayPoint3 = nextWayPoint2 - straightLength * Position2d(normCarX, normCarY);
  // straight2.push_back(nextWayPoint3);
  // straight2.resample(0.1);


  // multiTraj.trajectories.push_back(straight2);


  return multiTraj;
}

Trajectory2d TrajectoryFactory::createCircle(double radius, double startAngle) {
  Trajectory2d traj;

  float stepSize = M_PI * 1.0 / 36;
  for (float angle = startAngle; angle < startAngle + M_PI / 2 - stepSize;
        angle += stepSize)
  {
    ExtendedPose2d pose(radius * sin(angle), -radius + radius * cos(angle),
                        -angle);
    traj.push_back(pose);
  }

  return traj;
}

void TrajectoryFactory::mirrorAtY(Trajectory2d &traj) {
  for (size_t i = 0; i < traj.size(); i++)
  {
    traj.at(i).setY(-traj.at(i).getY());
    traj.at(i).setYaw(-traj.at(i).getYaw());
  }
}

void TrajectoryFactory::moveTrajectoryToStart(Trajectory2d &traj, ExtendedPose2d start) {
  const float yaw = start.getYaw();
  for (size_t i = 0; i < traj.size(); i++)
  {
    double x = traj.at(i).getX();
    double y = traj.at(i).getY();

    // Rotate point by angle of car pose:
    ExtendedPose2d rotated(x * cos(yaw) - y * sin(yaw), x * sin(yaw) + y * cos(yaw), 0);

    // Add car position:
    traj.at(i).setX(rotated.getX() + start.getX());
    traj.at(i).setY(rotated.getY() + start.getY());
    traj.at(i).setYaw(fmod(traj.at(i).getYaw() + start.getYaw(), 2 * M_PI));
  }
}

MultiTrajectory TrajectoryFactory::generateCrossPark(core::Pose2d& takeoff)
{
  MultiTrajectory crossPark = TrajectoryDatabase::getTrajectory("park_cross");

  ExtendedPose2d takeoffPose = ExtendedPose2d(takeoff);
  ExtendedPose2d carPose = Environment::getInstance()->getCarPose();
  
  float yaw = takeoffPose.getYaw();

  for (auto &traj : crossPark.trajectories) {
    
    for (size_t i = 0; i < traj.size(); i++)
    {
      double x = traj.at(i).getX();
      double y = traj.at(i).getY();

      // Rotate point by angle of car pose:
      ExtendedPose2d rotated(x * cos(yaw) - y * sin(yaw), x * sin(yaw) + y * cos(yaw), 0);

      // Add car position:
      traj.at(i).setX(rotated.getX() + takeoffPose.getX());
      traj.at(i).setY(rotated.getY() + takeoffPose.getY());
      traj.at(i).setYaw(fmod(traj.at(i).getYaw() + takeoffPose.getYaw(), 2 * M_PI));
    }
  }

  // generate path to takeoff
  Trajectory2d takeoffTraj;
  takeoffTraj.push_back(carPose);
  takeoffTraj.push_back(takeoffPose);
  takeoffTraj.resample(0.1);


  // Append first park trajectory (This is intended to help the lateral controller)
  // takeoffTraj.append(crossPark.trajectories[0]);
  takeoffTraj.calculateCurvature();
  
  // crossPark.trajectories[0] = takeoffTraj;

  crossPark.trajectories.insert(crossPark.trajectories.begin(), takeoffTraj);

  return crossPark;
}

core::Trajectory2d TrajectoryFactory::generateBackup() {

  Trajectory2d backup = TrajectoryDatabase::getSingleTrajectory("backup_obstacle");

  ExtendedPose2d carPose = Environment::getInstance()->getCarPose();
  float yaw = carPose.getYaw();
  for (size_t i = 0; i < backup.size(); i++)
  {
    double x = backup.at(i).getX();
    double y = backup.at(i).getY();

    // Rotate point by angle of car pose:
    ExtendedPose2d rotated(x * cos(yaw) - y * sin(yaw), x * sin(yaw) + y * cos(yaw), 0);
    // Add car position:

    backup.at(i).setX(rotated.getX() + carPose.getX());
    backup.at(i).setY(rotated.getY() + carPose.getY());
    backup.at(i).setYaw(fmod(backup.at(i).getYaw() + carPose.getYaw(), 2 * M_PI));
  }

  return backup;
}

/******************************************************************************/
/***************************** private methods *********************************/
/******************************************************************************/

void TrajectoryFactory::setFixedTrajectory(MultiTrajectory traj)
{
  mTrajectoryMode = FIXED_TRAJECTORY;
  Environment::getInstance()->setTrajectory(traj);
}

bool TrajectoryFactory::isInFrontOnTraj(ExtendedPose2d &projection, std::size_t nearest_pose_index,
                                        Trajectory2d traj, std::size_t traj_index, double offset)
{
  Trajectory2d newTraj;
  newTraj.append(traj);

  if (traj_index > nearest_pose_index)
  {
    traj_index++;
  }
  else
  {
    return false;
  }

  newTraj.insert(newTraj.begin() + nearest_pose_index + 1, projection);

  double distance = newTraj.lengthBetween(nearest_pose_index + 1, traj_index);

  if (distance > offset)
  {
    return true;
  }
  else
  {
    return false;
  }
}

void TrajectoryFactory::createTrajStartPoints(const Environment &env, Trajectory2d *newTrajectory,
                                              PatchPtr outLastPastPatch)
{
  const PatchPtrList *street = env.getStreet();

  if (street->size() > 0)
  {
    mPatchIterator = *(street->begin());

    mlastCarPose = env.getCarPose();
    mlastPatchPose = env.getCarPose();

    const PatchPtrList *pastCarPatches = env.getCarPatchHistory();
    ExtendedPose2dVector pastCarPoses = env.getPastCarPoses();

    int lastUsedPastPose = pastCarPoses.size() - 1;

    // If we've already been on a patch, start there:
    if (pastCarPatches->size() >= 3)
    {

      PatchPtrList::const_iterator it = pastCarPatches->begin();
      std::advance(it, pastCarPatches->size() - 3);
      mPatchIterator = *it;

      int poseIndexBeforeLastPatch =
              Environment::getInstance()->getCarPoseIndexBeforePatch(mPatchIterator->getId());
      mlastCarPose = pastCarPoses[poseIndexBeforeLastPatch];

      mlastPatchPose = (mPatchIterator)->getPose();
      PatchPtr next = mPatchIterator->getSuccessor();

      if (next)
      {
        mPatchIterator = next;
      }
      lastUsedPastPose = -1;
    }

    else
    {
      mlastCarPose = pastCarPoses.back();
      mlastPatchPose = mPatchIterator->getPose();

      mPatchIterator = *(street->begin());
      PatchPtr next = mPatchIterator->getSuccessor();

      if (next)
      {
        mPatchIterator = next;
      }
      else
      {
        mlastPatchPose = pastCarPoses.back();
      }
    }

    // append previous driven trajectory

    if (lastUsedPastPose == 0)
    {
      lastUsedPastPose = pastCarPoses.size() - 1;
    }

    unsigned int firstUsedPastPose = 0;
    if (lastUsedPastPose > 5)
    {
      firstUsedPastPose = lastUsedPastPose - 5;
    }


    if (lastUsedPastPose != -1)
    {
      for (ExtendedPose2dVector::iterator it =
              pastCarPoses.begin() + firstUsedPastPose;
           it != pastCarPoses.begin() + lastUsedPastPose + 1; it++)
      {
        newTrajectory->push_back(*it);
      }
    }

    if (pastCarPatches->size() > 0)
    {
      outLastPastPatch = pastCarPatches->back();
    }
  }
}

void TrajectoryFactory::createTrajFromPatches(Trajectory2d *newTrajectory, PatchPtr lastPastPatch)
{
  int counter = 0;
  bool thisPatchShouldBeFixed = false;

  while (mPatchIterator && counter < GO_FRONT_PATCHES)
  {
    LOGGING_INFO(worldLogger, "[TF] Loop Iterator: " << mPatchIterator->getPatchID() << " "
                                                     << mPatchIterator->getPose() << endl);

    // Find out which child of the current patch (==iterator) will be the next
    // patch, depending on the last patches' position. It also takes into
    // account the driving direction which was set for the current patch. If
    // the last patch was, for example, to the NORTH of the current patch,
    // then getNextChild will return the child towards the SOUTH (but only if
    // the drivingDirection set for the iterator is "DD_STRAIGHT"!)
    PatchPtr next = mPatchIterator->getSuccessor();

    // if (next)
    // {
    //   if (mPatchIterator->isSwitch())
    //   {
    //     LaneType new_lane = mPatchIterator->getLane() == LANE_LEFT ? LANE_RIGHT : LANE_LEFT;
    //     next->setLane(new_lane);
    //   }
    //   else
    //   {
    //     next->setLane(mPatchIterator->getLane());
    //   }
    // }

    // get the trajectory reference points of the patch and append it to
    // current trajectory:
    Trajectory2d *patchTraj = mPatchIterator->getTrajectoryFromMC(mlastCarPose,
                                                                  mPatchIterator->getLane());

    if (patchTraj->size() == 0)
    { break; }

    if (mPatchIterator->getPatchType() == CROSS_SECTION)
    {
      addTrajPointsForCrossSection(newTrajectory, *patchTraj);
    }
    else
    {
      newTrajectory->append(*patchTraj);
    }

    mlastPatchPose = mPatchIterator->getPose();
    mlastCarPose = (*newTrajectory)[newTrajectory->size() - 1];

    if ((mPatchIterator->getPatchType() == STRAIGHT || mPatchIterator->getPatchType() == CROSS_SECTION) && (mPatchIterator->getLane() == LANE_LEFT || mPatchIterator->getLane() == SWITCH_TARGET_LANE))
    {
      // Per default the car pose yaw for left lane trajectories is reversed.
      // Make sure yaw points "forward" by reversing again.
      mlastCarPose.setYaw(mlastCarPose.getYaw() + M_PI);
    }

    if (thisPatchShouldBeFixed)
    {
      mPatchIterator->setFixed();
    }

    if (mPatchIterator == lastPastPatch)
    {
      thisPatchShouldBeFixed = true;
    }
    else
    {
      thisPatchShouldBeFixed = false;
    }

    mPatchIterator = next;
    counter++;
  }
}

void TrajectoryFactory::addTrajPointsForCrossSection(Trajectory2d *trajectory,
                                                     const Trajectory2d &crossSectionTraj)
{
  // Add more points to the front and end. This is to make sure the zigzag
  // removal doesn't remove important parts of this trajectory.
  Position2d pos;
  Position2d dir;
  ExtendedPose2d first = crossSectionTraj.at(0);
  dir = crossSectionTraj.at(0).getPosition() - crossSectionTraj.at(1).getPosition();
  dir.normalize();
  for (int i = 0; i < 10; i++)
  {
    // Add a new point before the first:
    pos = first.getPosition() + dir * i * 0.01;
    trajectory->push_back(ExtendedPose2d(pos(0), pos(1), first.getYaw()));
  }

  trajectory->append(crossSectionTraj);

  ExtendedPose2d last = crossSectionTraj.back();
  dir = crossSectionTraj.at(crossSectionTraj.size() - 1).getPosition() -
        crossSectionTraj.at(crossSectionTraj.size() - 2).getPosition();
  dir.normalize();
  for (int i = 0; i < 10; i++)
  {
    // Add a new point before the first:
    pos = last.getPosition() + dir * i * 0.01;
    trajectory->push_back(ExtendedPose2d(pos(0), pos(1), last.getYaw()));
  }
}

void TrajectoryFactory::validateTrajPoints(Trajectory2d *trajectory)
{
  // Ugly workaround because we get poses which lie way off the map.
  // Ignore all poses which are very far away:
  Trajectory2d trajCopy = *trajectory;
  trajectory->clear();
  for (size_t i = 0; i < trajCopy.size(); i++)
  {
    if (fabs(trajCopy[i].getX()) < 1e3 && fabs(trajCopy[i].getY()) < 1e3)
    {
      trajectory->push_back(trajCopy[i]);
    }
    else
    {
      LOGGING_WARNING(worldLogger, "Removing trajectory point (too far out)! "
              << mTrajectoryCounter << endl);
    }
  }
}

void TrajectoryFactory::incorporateOldTraj(const Environment &env, Trajectory2d *trajectory,
                                           bool useOldTraj)
{
  Trajectory2d beforeSmooth;
  beforeSmooth = *trajectory;
  // Resampling Intervall. (Traj is stored in this intervall.) we have have 2 *
  // resamplingIntervall constant under the car
  const double resamplingIntervall = 0.05;

  if (useOldTraj && mOldTraj.size() > 0)
  {
    LOGGING_INFO(worldLogger, "we have a old Traj" << endl);
    std::size_t carOnTrajIndex = 0;
    ExtendedPose2d projection;
    double distance;
    // we want to reuse the last points before the car
    env.calculateProjection(mOldTraj, env.getCarPose(), projection, distance,
                            carOnTrajIndex);

    int lastTrajIndex = std::min(carOnTrajIndex + 2, mOldTraj.size() - 1);
    Trajectory2d newTrajBack;
    const float maxDistOldTraj = 1;
    // copy old trajectory
    for (int i = 0; i <= lastTrajIndex; i++)
    {
      if (env.getCar()->calcDistTo(mOldTraj[i]) < maxDistOldTraj)
      {
        newTrajBack.push_back(mOldTraj[i]);
      }
    }

    ExtendedPose2d backEnd = env.getCarPose();
    // determine end of old traj
    if (newTrajBack.size() > 0)
    {
      backEnd = newTrajBack.back();
    }
    // give some space for a smooth transiton
    LOGGING_INFO(worldLogger, "add old points" << endl);
    double offset = 0.10;  // if you will use this offset you will get
    // surprising results at crossings with 90 degrees
    LOGGING_INFO(worldLogger, "Beginn Resampling. Length of Traj is: "
            << trajectory->length() << "Traj Point count: "
            << trajectory->size() << endl);

    beforeSmooth.resample(resamplingIntervall);
    LOGGING_INFO(worldLogger, "Traj is now resampled" << endl);
    bool takeOnePoint = false;
    // add the points of the new trajectory. (of course only the one which are
    // in front of the old points which we have taken)

    // calc projection for the inFrontOfTraj method
    std::size_t projIndex = 0;
    env.calculateProjection(beforeSmooth, backEnd, projection, distance,
                            projIndex);

    for (std::size_t i = 0; i < beforeSmooth.size(); i++)
    {
      if (isInFrontOnTraj(projection, projIndex, beforeSmooth, i, offset))
      {
        newTrajBack.push_back(beforeSmooth[i]);
        takeOnePoint = true;
      }
      else if (takeOnePoint)
      {
        std::cerr << "remove one point of the traj. but take the point before"
                  << std::endl;
      }
    }

    trajectory->clear();
    trajectory->append(newTrajBack);
    // resample the trajectory to avoid bad results in the next interation
    trajectory->resample(resamplingIntervall);
    // save traj for next iteration
    mOldTraj = *trajectory;

  }
  else if (useOldTraj)
  {
    // we have no old traj so we must take the new one and save the old one
    LOGGING_INFO(worldLogger, "we have no old Traj" << endl);
    trajectory->resample(resamplingIntervall);
    mOldTraj = *trajectory;
  }
}

void TrajectoryFactory::smoothTrajectory(Trajectory2d *trajectory)
{
  LOGGING_INFO(worldLogger, "begin smoothing Traj. Size is: " << trajectory->size() << endl);

  // now we smooth the complete Trajectory. The results under the car are
  // hopefully the same. (This is the reason why we use some old points)
  if (trajectory->size() == 2)
  {
    // interpolate Linear, so we can calculate the curvature
    // no usefull bSpline is possible
    oadrive::core::Interpolator::interpolateLinear(*trajectory, 0.5);
  }
  else
  {
    // resample to give the b-Splines more space
    trajectory->resample(0.30);
    // TODO: Just one round for testing purposes
    oadrive::core::Interpolator::smoothBSpline(*trajectory, 3);
  }
  // do this things only if we have enough points. otherwise the code will crash
  if (trajectory->size() > 1)
  {
    // now there are a lot of Points due the b Spline. Resample to get smooth
    // curvatures (What would shannon say to this resamplings?)
    trajectory->calculateOrientations();
    trajectory->resample(0.15);
    trajectory->simplify(0.012);  // simplify to avoid big curvatures
    trajectory->calculateCurvature();
  }
}

int TrajectoryFactory::removeZigZag(oadrive::core::Trajectory2d &traj)
{
  int removed = 0;
  if (traj.size() < 3)
  { return removed; }
  traj.calculateOrientations();

  ExtendedPose2d fstPose = traj[0];
  for (Trajectory2d::iterator cur = traj.begin() + 2; cur != traj.end() - 1;)
  {
    ExtendedPose2d fstPose = *(cur - 1);
    ExtendedPose2d sndPose = *cur;
    ExtendedPose2d trdPose = *(cur + 1);
    double x1 = sndPose.getX() - fstPose.getX();
    double x2 = sndPose.getX() - trdPose.getX();

    double y1 = sndPose.getY() - fstPose.getY();
    double y2 = sndPose.getY() - trdPose.getY();

    double denom1 = std::max(sqrt(x1 * x1 + y1 * y1), 0.000001);
    double denom2 = std::max(sqrt(x2 * x2 + y2 * y2), 0.000001);

    x1 = x1 / denom1;
    y1 = y1 / denom1;

    x2 = x2 / denom2;
    y2 = y2 / denom2;

    double angle = acos(x1 * x2 + y1 * y2);

    if (angle < ZICK_ZACK_ANGLE)
    {
      // raus loeschen
      // traj.erase(traj.begin()+sndInd);
      cur = traj.erase(cur);
      cur = traj.erase(cur);
      if (traj.size() <= 3)
      {
        break;
      }
      if (cur != traj.begin() + 1)
      {
        cur--;
      }

      removed++;
      removed++;
    }
    else
    {
      cur++;
    }
  }
  return removed;
}

MultiTrajectory TrajectoryFactory::rotateAndMoveTrajectory(MultiTrajectory &multiTraj,
                                                           const ExtendedPose2d &pose)
{
  MultiTrajectory rotatedMultiTrajectory;
  for (unsigned int t = 0; t < multiTraj.trajectories.size(); t++)
  {
    Trajectory2d rotated, traj;
    traj = multiTraj.trajectories[t];
    for (size_t i = 0; i < traj.size(); i++)
    {
      // Rotate the trajecotry point acoording to "pose"'s yaw:
      double x = traj[i].getX() * cos(pose.getYaw()) -
                 traj[i].getY() * sin(pose.getYaw());
      double y = traj[i].getX() * sin(pose.getYaw()) +
                 traj[i].getY() * cos(pose.getYaw());
      // Append position:
      ExtendedPose2d newPose(pose.getX() + x, pose.getY() + y,
                             pose.getYaw() + traj[i].getYaw());
      rotated.push_back(newPose);
    }
    rotated.isForwardTrajectory() = traj.isForwardTrajectory();
    rotatedMultiTrajectory.trajectories.push_back(rotated);
  }
  return rotatedMultiTrajectory;
}

void TrajectoryFactory::writeToFile(Trajectory2d &traj, std::string name)
{
 std::stringstream path;
 path << mDebugFolder;
 path << name;
 path << mTrajectoryCounter;
 path << ".txt";
 std::ofstream file(path.str().c_str());
 file << traj;
}

core::Trajectory2d TrajectoryFactory::getPreParkingTrajectory()
{
  // Trajectory to the beginning of the actual parking trajectory.
  return Trajectory2d();
}

core::Trajectory2d TrajectoryFactory::getParkingTrajectory()
{
  TrajectoryDatabase::getSingleTrajectory("park_cross");

  return Trajectory2d();
}

}  // namespace world
}  // namespace oadrive
