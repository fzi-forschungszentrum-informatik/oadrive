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
 * \author  Mark Hueneberg <hueneber@fzi.de>
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


#ifndef OADRIVE_WORLD_ENVIRONMENT_H_
#define OADRIVE_WORLD_ENVIRONMENT_H_

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <oadrive_util/CoordinateConverter.h>
#include <oadrive_util/Timer.h>
#include <oadrive_obstacle/ObjectTracker.h>
#include <oadrive_core/Trajectory2d.h>
#include <icl_core_logging/LoggingManager.h>

#include "EnvObject.h"
#include "Patch.h"
#include "MultiTrajectory.h"
#include "Road.h"
#include "TrajectoryFactory.h"
#include "StateMachine.h"

#include <list>
#include <vector>
#include <boost/shared_ptr.hpp>
#include <chrono>

class EnvironmentPainter;

#include "EnvironmentPainter.h"


#define CAR_DIFF_POS_THRES 0.001

#define CAR_WIDTH 0.30
#define CAR_LENGTH 0.58

#define STOP_TIME_INTERSECTION 2000 // in milliseconds

namespace oadrive
{
namespace world
{

enum Event
{
  APPROACH_INTERSECTION,
  FINISH_INTERSECTION,
  TRAJECTORY_FINISHED,
  RESET_ROAD
};
enum DrivingCommand
{
  STOP, DRIVE, NONE
};

class Environment;

typedef boost::shared_ptr<Environment> EnvironmentPtr;

class Environment
{
  friend class EnvironmentPainter;  // let Painter access all of my private elements

public:
  /*! Constructs the singleton. Must only be called once. */
  static EnvironmentPtr init(util::CoordinateConverter *coordConverter = NULL,
                             util::Timer *timer = NULL);

/*! Returns an instance to the singleton.
   * Make sure Environment::init was called before!*/
  static EnvironmentPtr getInstance();

  /*! Re-initializes the environment (by deleting and recreating it)
   * Should be called when the position is reset.*/
  static EnvironmentPtr reset();

  // Public constructor to allow switch to ROS messages
  Environment(util::CoordinateConverter *coordConverter, util::Timer *timer);

  ~Environment();

  ///////////////////////////////////////////////////////////////////
  // Event handling functions:
  void processEvent(Event event);

  bool checkForStop();

  ///////////////////////////////////////////////////////////////////
  // Right of way handling functions:
  bool relevantTrafficWithoutRightOfWay(const PatchPtr intersection);

  bool relevantTrafficWithRightOfWay(const PatchPtr intersection);

  bool relevantTrafficRightHasRightOfWay(const PatchPtr intersection);

  bool checkForTrafficLeft(const PatchPtr intersection);

  bool checkForTrafficStraight(const PatchPtr intersection);

  bool checkForTrafficRight(const PatchPtr intersection);

  ///////////////////////////////////////////////////////////////////
  // Traffic sign functions:
  void addTrafficSign(TrafficSign trafficSign);

  ///////////////////////////////////////////////////////////////////
  // Car pose functions:

  /*! Should be called once at the beginning of each new image frame.
   * Triggers event regions (if car enters or leaves them).
   * Updates the car position history.*/
  void updateCarPose(const core::ExtendedPose2d &carPose);

  EnvObjectPtr getCar() const
  { return mCar; }

  core::ExtendedPose2d getCarPose() const;

  core::ExtendedPose2dVector getPastCarPoses() const;

  unsigned int getCarPoseIndexBeforePatch(unsigned int patchID);

  ///////////////////////////////////////////////////////////////////
  // Street and patch functions:
  void addPatch(PatchPtr patch);

  void deleteOldPatches();

  const PatchPtrList *getStreet() const;

  const PatchPtrList *getCarPatchHistory() const;

  ///////////////////////////////////////////////////////////////////
  // Obstacle functions:
  void addObstacle(EnvObjectPtr obstacle);

  DrivingCommand checkForObstaclesInPath();

  ///////////////////////////////////////////////////////////////////
  // Object functions:
  void resetTrafficCars();
  void addTrafficCar(obstacle::TrackedCar trafficCar);

  const EnvObjectPtrList getTrafficCars();

  void addPedestrian(EnvObjectPtr pedestrian);

  ///////////////////////////////////////////////////////////////////
  // Trajectory functions:
  bool generateNextTrajectory(StateMachine::State state);

  MultiTrajectory getMultiTrajectory();

  void setTrajectory(const MultiTrajectory &traj);

  bool calculateProjection(const core::Trajectory2d &trajectory,
                           const core::ExtendedPose2d &curPosition,
                           core::ExtendedPose2d &projection,
                           double &distance,
                           size_t &nearest_pose_index) const;

  ///////////////////////////////////////////////////////////////////
  // Misc functions:
  bool isStorePosesBeforePatch() const
  {
    return mStorePosesBeforePatch;
  }

  void setStorePosesBeforePatch(bool storePosesBeforePatch)
  {
    Environment::mStorePosesBeforePatch = storePosesBeforePatch;
  }

  util::Timer *getTimer()
  { return mTimer; }

  util::CoordinateConverter *getCoordConverter()
  { return mCoordConverter; }

  void setLane(LaneType lane);

  void setParkingTakeoffPoint(core::Pose2d& takeoff);

private:

  void updateHistory();

  /******************************************************************************/
/**************************** class attributes ********************************/
/******************************************************************************/

  /*! The one and only instance of this singleton.*/
  static boost::shared_ptr<Environment> mInstance;

  EnvObjectPtr mCar;

  util::CoordinateConverter *mCoordConverter;

  Road mRoad;

  TrajectoryFactory mTrajectoryFactory;

  core::ExtendedPose2dVector mPastCarPoses;
  
  std::map<unsigned int, int> mCarPoseIndexBeforePatch;
  bool mStorePosesBeforePatch;

  // for parking
  core::Pose2d mLastTakeoffPose;
  bool mNewTakeoffPose = false;

  MultiTrajectory mMultiTrajectory;

  std::list<EnvObjectPtr> mTrafficCars;
  std::list<EnvObjectPtr> mPedestrians;
  std::list<EnvObjectPtr> mObstacles;

  //! \brief mTimer This timer is nessesary to remove objects which have a expiered time to live
  //! \note This timer has to be updated by the interface
  util::Timer *mTimer;
  std::chrono::high_resolution_clock::time_point mLastCall;
  std::chrono::high_resolution_clock::time_point mNow;
  int mStandstillTime = 0;
  int mStanding = 0;
  bool mStandstill = false;
  bool mCurrentlyDrivingTrajectory = false;

  EnvironmentPainter mEnvironmentPainter;

  icl_core::logging::LoggingManager &loggingManager = icl_core::logging::LoggingManager::instance();

public:
  // use a proper alignment when calling the constructor.
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

}  // namespace
} // namespace

#endif
