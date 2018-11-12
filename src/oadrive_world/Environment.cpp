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
 * \author  Peter Zimmer <peter-zimmer@gmx.net>
 * \author  Micha Pfeiffer <ueczz@student.kit.edu>
 * \date    2015-11-24
 * 
 * \author  Simon Roesler <simon.roesler@student.kit.edu>
 * \date    2018
 * 
 * \author  Mark Hueneberg <hueneber@fzi.de>
 * \date    2018
 *
 * Center-Piece for Oadrive, representtion of the current world.
 *
 */
//----------------------------------------------------------------------

#include "Environment.h"

#include <oadrive_core/Interpolator.h>
#include <oadrive_world/worldLogging.h>
#include <oadrive_world/aadc_roadSign_enums.h>
#include <oadrive_util/Config.h>

#include <boost/pointer_cast.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

using namespace boost::posix_time;
using namespace oadrive::util;
using namespace oadrive::core;

using icl_core::logging::endl;
using icl_core::logging::flush;

namespace oadrive
{
namespace world
{

// Initialize as NULL-Pointer:
boost::shared_ptr<Environment> Environment::mInstance;

EnvironmentPtr Environment::init(CoordinateConverter* coordConverter, Timer* timer)
{
  if (mInstance)
  {
    LOGGING_ERROR(worldLogger, "" << "Cannot initialize another Environment!" << endl
                                  << "\tEnvironment is a Singleton." << endl
                                  << "\tCall init() only once, then use getInstance()." << endl);
  }
  else
  {
    mInstance = EnvironmentPtr(new Environment(coordConverter, timer));
    if (getInstance())
    {
      LOGGING_INFO(worldLogger, "Constructed new Environment." << endl);
    }
    else
    {
      LOGGING_INFO(worldLogger, "Could not construct new Environment." << endl);
    }
  }

  return mInstance;
}

EnvironmentPtr Environment::reset()
{
  if (mInstance)
  {
    Timer* timer = mInstance->getTimer();
    CoordinateConverter* coordConverter = mInstance->getCoordConverter();
    // Resetting the java-way: (:P)
    mInstance = EnvironmentPtr(new Environment(coordConverter, timer));
    LOGGING_INFO(worldLogger, "Environment was reset." << endl);
  }
  else
  {
    LOGGING_ERROR(worldLogger, "" << "Cannot reset Environment!" << endl
                                  << "\tNo instance found. Call init() before." << endl);
  }
  return mInstance;
}

EnvironmentPtr Environment::getInstance()
{
  if (!mInstance)
  {
    LOGGING_ERROR(worldLogger, "" << "Cannot get Environment Instance!" << endl
                                  << "\tCall init() once, then use getInstance()." << endl);
  }
  return mInstance;
}

// Constructor. Is private!
Environment::Environment(CoordinateConverter* coordConverter, oadrive::util::Timer* timer) :
        mCar(new EnvObject(ExtendedPose2d(0, 0, 0), CAR_WIDTH, CAR_LENGTH)),
        mLastTakeoffPose(core::Pose2d()),
        mCoordConverter(coordConverter),
        mMultiTrajectory(),
        mTimer(timer),
        mStandstillTime(0),
        mStanding(0),
        mCurrentlyDrivingTrajectory(false)
{
  mStorePosesBeforePatch = true;

  std::string configFolder = Config::getConfigPath();
  
  TrajectoryDatabase::load(configFolder + "/trajectories");

  loggingManager.initialize();
  loggingManager.setLogLevel(icl_core::logging::LogLevel::eLL_WARNING);
  worldLogger::create();

  if (mTimer != NULL)
  {
    LOGGING_INFO(worldLogger, "Registering Environment as timer event listener." << endl);
    mTimer->setTimer(1000, TIMER_TYPE_REMOVE_OLD_OBJECTS);
  }
}

Environment::~Environment()
{
}

/******************************************************************************/
/**************************** public methods *********************************/
/******************************************************************************/

void Environment::updateCarPose(const ExtendedPose2d &carPose)
{
  mLastCall = mNow;
  mNow = std::chrono::high_resolution_clock::now();

  getCar()->setPose(carPose);


  if (mPastCarPoses.size() == 0)
  {
    mPastCarPoses.push_back(carPose);
  }

  updateHistory();
}

void Environment::addPatch(PatchPtr patch)
{
  mRoad.addRoadPatch(patch);
}

void Environment::deleteOldPatches() 
{
  mRoad.deleteOldPatches();
}

const PatchPtrList* Environment::getStreet() const { return mRoad.getRoad(); }

const PatchPtrList* Environment::getCarPatchHistory() const { return mRoad.getCarPatchHistory(); }

MultiTrajectory Environment::getMultiTrajectory()
{
  return mMultiTrajectory;
}

void Environment::setTrajectory(const MultiTrajectory &traj)
{
  mMultiTrajectory = traj;

  // TODO(kolja) Later on, maybe look at the SegFault that happens on this call.
}

ExtendedPose2dVector Environment::getPastCarPoses() const
{
  return mPastCarPoses;
}

unsigned int Environment::getCarPoseIndexBeforePatch(unsigned int patchID)
{
  return mCarPoseIndexBeforePatch[patchID];
}

ExtendedPose2d Environment::getCarPose() const
{
  return getCar()->getPose();
}

bool
Environment::calculateProjection(const Trajectory2d &trajectory, const ExtendedPose2d &curPosition,
                                 ExtendedPose2d &projection,
                                 double &distance, std::size_t &nearest_pose_index) const
{
  if (trajectory.size() == 0)
  {
    return false;
  }
  float dist1, dist2, ratio, curr_distance_squared;
  float t;

  float shortest_distance_squared = std::numeric_limits<float>::infinity();

  bool shortest_distance_found_before_first_point = true;
  nearest_pose_index = 0;

  Position2d position = curPosition.getPosition();
  const double &xP = position.x();
  const double &yP = position.y();
  for (u_int32_t i = 0; i < trajectory.size() - 1; ++i)
  {
    const double &x1 = trajectory[i].getX();
    const double &y1 = trajectory[i].getY();
    const double &x2 = trajectory[i + 1].getX();
    const double &y2 = trajectory[i + 1].getY();

    double Xnew, Ynew;

    const double &APx = xP - x1;
    const double &APy = yP - y1;
    const double &ABx = x2 - x1;
    const double &ABy = y2 - y1;
    const double &magAB2 = ABx * ABx + ABy * ABy;
    const double &ABdotAP = ABx * APx + ABy * APy;
    t = ABdotAP / magAB2;

    if (t < 0)
    {
      Xnew = x1;
      Ynew = y1;
    }
    else if (t > 1)
    {
      Xnew = x2;
      Ynew = y2;
    }
    else
    {
      Xnew = x1 + ABx * t;
      Ynew = y1 + ABy * t;
    }


    double a = (xP - Xnew) * (xP - Xnew);
    double b = (yP - Ynew) * (yP - Ynew);
    curr_distance_squared = (a + b);

    if (curr_distance_squared < shortest_distance_squared)
    {
      shortest_distance_squared = curr_distance_squared;
      shortest_distance_found_before_first_point = (i == 0 && t < 0);

      nearest_pose_index = i;

      dist1 = sqrt((Xnew - x1) * (Xnew - x1) + (Ynew - y1) * (Ynew - y1));
      dist2 = sqrt((Xnew - x2) * (Xnew - x2) + (Ynew - y2) * (Ynew - y2));
      ratio = dist1 / (dist1 + dist2);

    }
  }
  //m_ratio = ratio;

  // A-B:
  const Position2d vector_ab =
          trajectory[nearest_pose_index].getPose().rotation() * Position2d(1.0, 0.0);
  // A-Vehicle
  const Position2d vector_a_vehicle = position - trajectory[nearest_pose_index].getPosition();

  // pose of the car is before the beginning of the trajectory -> use distance to projection on AB-Vector
  if (shortest_distance_found_before_first_point)
  {
    // copy speed and curvature from first point
    projection = trajectory.front();

    // Projection
    const double t = vector_a_vehicle.dot(vector_ab) / vector_ab.squaredNorm();
    projection.setPosition(trajectory[0].getPosition() + vector_ab * t);

    // Calculate distance
    distance = (projection.getPosition() - position).norm();

  }
  else
  {
    //calculate projected pose
    projection = oadrive::core::Interpolator::interpolateLinear(trajectory[nearest_pose_index],
                                                                trajectory[nearest_pose_index + 1],
                                                                ratio);

    // distance calculation
    distance = sqrt(shortest_distance_squared);

  }

  //Sign of distance
  Eigen::Matrix<double, 2, 2> m;
  m.col(0) << vector_ab;
  m.col(1) << vector_a_vehicle;

  // If the determinant is negative, the point lies on the right hand to the line
  if (m.determinant() > 0.0)
  {
    distance = -distance;
  }

  return shortest_distance_found_before_first_point;
}

void Environment::addTrafficSign(TrafficSign trafficSign)
{
  TrafficSignPtr trafficSignPtr = boost::make_shared<TrafficSign>(trafficSign.getType(), trafficSign.getPose());

  mRoad.addTrafficSign(trafficSignPtr);
}

/******************************************************************************/
/**************************** private methods *********************************/
/******************************************************************************/

void Environment::updateHistory()
{
  //get current Patches the car is on
  PatchPtrList newCarPatches;

  std::vector<int> patchIDsCarIsOn = mRoad.updateHistory(getCarPose());

  for (int id : patchIDsCarIsOn)
  {
    int size = mPastCarPoses.size() - 1;
    if (mStorePosesBeforePatch)
    {
      mCarPoseIndexBeforePatch[id] = std::max(0, size);
    }
  }

  //add new car pos to history if car moved more than the CAR_DIFF_POS_THRES threshold
  ExtendedPose2d lastPos = mPastCarPoses.back();
  float diff = (lastPos.getPosition() - getCarPose().getPosition()).norm();
  if (diff > CAR_DIFF_POS_THRES)
  {  
    mStanding = 0;
   
  } 
  else
  {
    if (mStanding < 0)
    {
      mStanding = 0;
    }     
   
    mPastCarPoses.pop_back();

    mStanding += std::chrono::duration_cast<std::chrono::milliseconds>(mNow - mLastCall).count();
  }
  mPastCarPoses.push_back(getCar()->getPose());
}

void Environment::processEvent(Event event)
{
  switch (event)
  {
    case Event::FINISH_INTERSECTION:
      // mRoad.deletePatchesUntilLastIntersection();
      LOGGING_WARNING(worldLogger, "FINISH_INTERSECTION" << endl);      
      break;
    case Event::APPROACH_INTERSECTION:
      LOGGING_WARNING(worldLogger, "APPROACH_INTERSECTION" << endl);
      mStandstill = true;
      break;
    case Event::TRAJECTORY_FINISHED:
      LOGGING_WARNING(worldLogger, "TRAJECTORY_FINISHED" << endl);
      mCurrentlyDrivingTrajectory = false;
      break;
    case Event::RESET_ROAD:
      LOGGING_WARNING(worldLogger, "RESET_ROAD" << endl);
      mRoad.reset();
      break;
    default:
      break;
  }
}

bool Environment::generateNextTrajectory(StateMachine::State currentState)
{
  bool newTrajectory = false;
  Trajectory2d nextTrajectory;
  MultiTrajectory nextMultiTraj;
  bool isMulti = false;

  if (mCurrentlyDrivingTrajectory)
    return false;

  switch (currentState)
  {
    case StateMachine::State::DRIVING_RAMP:
      // nextMultiTraj = mTrajectoryFactory.generateRamp();
      // isMulti = true;
      // newTrajectory = true;
      // mCurrentlyDrivingTrajectory = true;
      // In this case the controller is driving on its own.. Good luck!
    break;
    case StateMachine::State::PULLING_OUT_LEFT:
      nextTrajectory = mTrajectoryFactory.generatePulloutLeft();
      newTrajectory = true;
      mCurrentlyDrivingTrajectory = true;
      break;
    case StateMachine::State::PULLING_OUT_RIGHT:
      nextTrajectory = mTrajectoryFactory.generatePulloutRight();
      newTrajectory = true;
      mCurrentlyDrivingTrajectory = true;
      break;
    case StateMachine::State::PARKING_LEFT:
      if (!mNewTakeoffPose)
        return false;
      nextMultiTraj = mTrajectoryFactory.generateCrossPark(mLastTakeoffPose);
      isMulti = true;
      newTrajectory = true;
      mCurrentlyDrivingTrajectory = true;
      mNewTakeoffPose = false;
      break;
    case StateMachine::State::PARKING_RIGHT:
      if (!mNewTakeoffPose)
        return false;
      nextMultiTraj = mTrajectoryFactory.generateCrossPark(mLastTakeoffPose);
      isMulti = true;
      newTrajectory = true;
      mCurrentlyDrivingTrajectory = true;
      mNewTakeoffPose = false;
      break;
    case StateMachine::State::PRE_BYPASSING:
      std::cout << "Now in Pre Bypass mode " << std::endl;
      nextTrajectory = mTrajectoryFactory.generateBackup();
      newTrajectory = true;
      mCurrentlyDrivingTrajectory = true;
      break;
    default:
      // We can only plan a traj. if we have a car position.. 
      if (mPastCarPoses.size() > 0) {
        nextTrajectory = mTrajectoryFactory.generateFromPatches(*this, true);
        newTrajectory = true;
      }
      break;
  }

  if (newTrajectory)
  {
    if (!isMulti) {
      nextMultiTraj.trajectories.push_back(nextTrajectory);
    }
    mMultiTrajectory = nextMultiTraj;
  }
  return newTrajectory;
}

DrivingCommand Environment::checkForObstaclesInPath()
{
  return DrivingCommand::DRIVE;
}

bool Environment::checkForStop()
{
  PatchPtr relevantIntersection = mRoad.getNextIntersection();

  // Check if we have to stop
  bool stop = false;

  // Case 1: We have a road AND a traffic sign
  if (relevantIntersection && relevantIntersection->hasTrafficSign())
  {
    TrafficSignPtr trafficSign = relevantIntersection->getTrafficSign();
    if (trafficSign->getType() == STOP_AND_GIVE_WAY)
    {
      std::cout << "Waiting at intersection since: " << mStanding << "ms" << std::endl;
      if (mStandstill && mStanding < STOP_TIME_INTERSECTION)
      {
        stop = true;
      }
      else
      {
        stop = relevantTrafficWithoutRightOfWay(relevantIntersection);

        std::cout << "Continue driving after " << mStanding << "ms " << "relevant Traffic: " << stop << std::endl;
        mStandstill = false;
      }
    }
    else if (trafficSign->getType() == GIVE_WAY)
    {
      stop = relevantTrafficWithoutRightOfWay(relevantIntersection);
      std::cout << "relevantTrafficWithoutRightOfWay " << stop << std::endl;
    }
    else if (trafficSign->getType() == HAVE_WAY)
    {
      stop = relevantTrafficWithRightOfWay(relevantIntersection);
    }
    else if (trafficSign->getType() == UNMARKED_INTERSECTION)
    {
      stop = relevantTrafficRightHasRightOfWay(relevantIntersection);
    }
  }
    // Case 2: We have no traffic sign so right has right-of-way.
  else if (relevantIntersection)
  {
    stop = relevantTrafficRightHasRightOfWay(relevantIntersection);
    std::cout << "relevant Traffic: " << stop << std::endl;
  } else {
    std::cout << "Intersection in checkForStop not found!" << std::endl;
  }

 return stop;
}

bool Environment::relevantTrafficWithoutRightOfWay(const PatchPtr intersection)
{
  bool relevantTraffic = false;
  if (intersection->getAction() == DD_STRAIGHT)
  {
    relevantTraffic = checkForTrafficLeft(intersection) || checkForTrafficRight(intersection);
  }
  else if (intersection->getAction() == DD_RIGHT)
  {
    relevantTraffic = checkForTrafficLeft(intersection);
  }
  else if (intersection->getAction() == DD_LEFT)
  {
    relevantTraffic = checkForTrafficLeft(intersection) || checkForTrafficRight(intersection) ||
                      checkForTrafficStraight(intersection);
  }
  return relevantTraffic;
}

bool Environment::relevantTrafficWithRightOfWay(const PatchPtr intersection)
{
  bool relevantTraffic = false;
  if (intersection->getAction() == DD_LEFT)
  {
    relevantTraffic = checkForTrafficStraight(intersection);
  }
  return relevantTraffic;
}
bool Environment::relevantTrafficRightHasRightOfWay(const PatchPtr intersection)
{
  bool relevantTraffic = false;
  if (intersection->getAction() == DD_STRAIGHT)
  {
    relevantTraffic = checkForTrafficRight(intersection);
  }
  else if (intersection->getAction() == DD_RIGHT)
  {
    relevantTraffic = false;
  }
  else if (intersection->getAction() == DD_LEFT)
  {
    relevantTraffic = checkForTrafficRight(intersection) || checkForTrafficStraight(intersection);
  }
  return relevantTraffic;
}

void Environment::resetTrafficCars()
{
  mTrafficCars.clear();
}

void Environment::addTrafficCar(obstacle::TrackedCar trafficCar)
{
  EnvObjectPtr trafficCarEnv = boost::make_shared<EnvObject>(trafficCar.object.pose);
  trafficCarEnv->setVelocity(trafficCar.speed);

  mTrafficCars.push_back(trafficCarEnv);
  // std::cout << mTrafficCars.size() << std::endl;
}

void Environment::addObstacle(EnvObjectPtr obstacle)
{
  mObstacles.push_back(obstacle);
}

void Environment::addPedestrian(EnvObjectPtr pedestrian)
{
  mPedestrians.push_back(pedestrian);
}

bool Environment::checkForTrafficLeft(const PatchPtr intersection)
{
  bool trafficFromLeft = false;
  for (auto trafficCarIt = mTrafficCars.begin(); trafficCarIt != mTrafficCars.end(); trafficCarIt++)
  {
    ExtendedPose2d poseRelIntersection =
            mCoordConverter->world2Car(intersection->getPose(), (*trafficCarIt)->getPose());
    ExtendedPose2d velocityRelIntersection =
            mCoordConverter->world2Car(intersection->getPose(), (*trafficCarIt)->getVelocity());

    // The ref point of the traffic car is the nearest point. This should be the rear right point
    // when the traffic car leaves the intersection it it is driving from left to right
    if (poseRelIntersection.getY() > -0.5f && poseRelIntersection.getY() < 2.5 && std::abs(poseRelIntersection.getX()) < 0.6)
    {
      // check driving direction
//      if (velocityRelIntersection.getY() < 0.f)
      {
        trafficFromLeft = true;
      }
    }
  }
  return trafficFromLeft;
}

bool Environment::checkForTrafficStraight(const PatchPtr intersection)
{
  bool trafficFromStraight = false;
  for (auto trafficCarIt = mTrafficCars.begin(); trafficCarIt != mTrafficCars.end(); trafficCarIt++)
  {
    ExtendedPose2d poseRelIntersection =
            mCoordConverter->world2Car(intersection->getPose(), (*trafficCarIt)->getPose());
    ExtendedPose2d velocityRelIntersection =
            mCoordConverter->world2Car(intersection->getPose(), (*trafficCarIt)->getVelocity());

    // The ref point of the traffic car is the nearest point. This should be the front left point
    // when the traffic car leaves the intersection if it is driving from left to right.
    if (poseRelIntersection.getX() > -0.5f && poseRelIntersection.getX() < 3.0f && std::abs(poseRelIntersection.getY()) < 0.6)
    {
      // check driving direction
//      if (velocityRelIntersection.getX() < 0.f)
      {
        trafficFromStraight = true;
      }
    }
  }
  return trafficFromStraight;
}

bool Environment::checkForTrafficRight(const PatchPtr intersection)
{
  bool trafficFromRight = false;
  for (auto trafficCarIt = mTrafficCars.begin(); trafficCarIt != mTrafficCars.end(); trafficCarIt++)
  {
    ExtendedPose2d poseRelIntersection =
            mCoordConverter->world2Car(intersection->getPose(), (*trafficCarIt)->getPose());
    ExtendedPose2d velocityRelIntersection =
            mCoordConverter->world2Car(intersection->getPose(), (*trafficCarIt)->getVelocity());

    // The ref point of the traffic car is the nearest point. This should be the rear left point
    // when the traffic car leaves the intersection if it is driving from right to left
    if (poseRelIntersection.getY() < 0.5f && poseRelIntersection.getY() > -1.75f && std::abs(poseRelIntersection.getX()) < 0.6)
    {
      // check driving direction
//      if (velocityRelIntersection.getY() > 0.f)
      {
        trafficFromRight = true;
      }
    }
  }
  return trafficFromRight;
}

const EnvObjectPtrList Environment::getTrafficCars()
{
  return mTrafficCars;
}

void Environment::setLane(LaneType lane) {
  // TODO: Remove functionality from trajectoryfactory mTrajectoryFactory.setLane(lane);
  mRoad.setLane(lane, 3); // also change one old patch 
}

void Environment::setParkingTakeoffPoint(core::Pose2d& takeoff) {
  mNewTakeoffPose = true;
  mLastTakeoffPose = takeoff;
}

}  // namespace
}  // namespace
