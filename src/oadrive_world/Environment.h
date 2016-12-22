// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// This program is free software licensed under the CDDL
// (COMMON DEVELOPMENT AND DISTRIBUTION LICENSE Version 1.0).
// You can find a copy of this license in LICENSE in the top
// directory of the source code.
//
// © Copyright 2016 FZI Forschungszentrum Informatik, Karlsruhe, Germany
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  David Zimmerer <dzimmerer@gmail.com>
 * \author  Peter Zimmer <peter-zimmer@gmx.net>
 * \author  Micha Pfeiffer <ueczz@student.kit.edu>
 * \date    2015-11-24
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
#include <oadrive_core/Trajectory2d.h>
#include <oadrive_control/DriverModule.h>
#include <oadrive_util/TimerEventListener.h>
#include <oadrive_obstacle/ProcessUS.h>

#include "EnvObject.h"
//#include "PatchStitcher.h"
#include "PatchStitcher.h"
//#include "PatchStitcherComplex.h"
#include "WorldEventListener.h"
#include "Patch.h"
#include "EventRegion.h"
#include "Obstacle.h"
#include "TrafficSign.h"
#include "MultiTrajectory.h"

#include <oadrive_util/Timer.h>

#include <list>
#include <map>
#include <vector>
#include <boost/date_time/local_time/local_time.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/shared_ptr.hpp>

class EnvironmentPainter;
#include "EnvironmentPainter.h"

#define CAR_DIFF_POS_THRES 0.01

#define GO_BACK_IN_TRAJ_HISTORY 5

#define CAR_WIDTH 0.30
#define CAR_LENGTH 0.58

#define OLD_PATCHES_LIFE_TIME 5
#define OLD_PATCHES_MIN_DIST 2.0
#define OLD_TRAFFICSIGN_LIFE_TIME 5
#define OLD_TRAFFICSIGN_MIN_DIST 3.5

#define OBSTACLE_SIGN_DISTANCE 0.30

#define OBSTACLE_FULL_BREAK_THRES 0.1

#define OBSTACLE_MERGING_DIST 0.05

using namespace oadrive::core;
using namespace oadrive::world;
using namespace oadrive::util;

namespace oadrive{
namespace world{

class PatchStitcher;

struct DebugPoint
{
  oadrive::core::ExtendedPose2d pose;
  cv::Scalar col;
  boost::posix_time::time_duration time;
  bool drawDirection;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

typedef std::list<DebugPoint, Eigen::aligned_allocator<DebugPoint> > DebugPointList;

class Environment;
typedef boost::shared_ptr<Environment> EnvironmentPtr;

class Environment: public TimerEventListener {
  friend class EnvironmentPainter;	// let Painter access all of my private elements

public:
  /*! Constructs the singleton. Must only be called once. */
  static EnvironmentPtr init( oadrive::util::CoordinateConverter* coordConverter = NULL,
               oadrive::control::DriverModule* driver = NULL,
               oadrive::util::Timer *timer = NULL);
  /*! Returns an instance to the singleton.
   * Make sure Environment::init was called before!*/
  static EnvironmentPtr getInstance();

  void setEventListener( WorldEventListener* listener );

  /*! Re-initializes the environment (by deleting and recreating it)
   * Should be called when the position is reset.*/
  static EnvironmentPtr reset();

  ~Environment();

  ///////////////////////////////////////////////////////////////////
  // Car pose functions:
 
  /*! Should be called once at the beginning of each new image frame.
   * Triggers event regions (if car enters or leaves them).
   * Updates the car position history.*/
  void updateCarPose(const ExtendedPose2d &carPose);
  EnvObjectPtr getCar() { return mCar; }
  ExtendedPose2d getCarPose();
  ExtendedPose2dVector getPastCarPoses();
  unsigned int getCarPoseIndexBeforePatch(unsigned int patchID);

  ///////////////////////////////////////////////////////////////////
  // Street and patch functions:

  /*! Add (temporary) patch found by StreetPatcher */
  void addPatch(PatchPtr patch);

  /*! Add a "finalized" street patch (i.e. one that has been chosen by the PatchStitcher).
         * \note Before calling this, make sure all the new children have been added to the patch! */
  void addStreetPatch(PatchPtr streetPatch);

  /*! Generate event regions for a newly added special patch.
         * \note Must be called for new CROSS_SECTION and PARKING patches.
         * Can also be called when PARKING patch changes orientation. */
  void generateEventRegionsForPatch( PatchPtr streetPatch );

  PatchPtrList* getStreet() { return &mStreet; }
  PatchPtrList* getParkingLots() { return &mParkingLots; }
  //PatchPtrList* getUnsortedPatches() { return &mUnsortedPatches; }
  PatchPtrList* getOpenPatches() { return &mOpenPatches; }
  PatchPtrList* getPastCarPatches() { return &mPastCarPatches; }
  PatchPtrList* getCurrentCarPatches() { return &mCurrentCarPatches; }
  ExtendedPose2dVectorByUIntMap* getCarPosesOnPatch() {
    return &mCarPosesOnPatch;
  }

  void removeOldStreetPatches(int seconds);
  void removeOldObjects();
  void clearAllPatches();
  void replaceStreetPatch(PatchPtr replacePatch, PatchPtr newPatch);

  void removeOldTrafficSigns();

  ///////////////////////////////////////////////////////////////////
  // Trajectory functions:

  void setTrajectory( const oadrive::core::Trajectory2d& traj );
  void setTrajectory( const MultiTrajectory& traj );
  void setRawTrajectory( const oadrive::core::Trajectory2d& traj )
  {
    mRawTrajectory = traj;
  }
  Trajectory2d getTrajectory();
  //void cannotGeneratePatchTrajectory();
  void setLane(enumLane lane);
  enumLane getLane();

  bool updateTrajSpeed(Trajectory2d &traj);
  void updateMultiTrajSpeed();

  ///////////////////////////////////////////////////////////////////
  // Event region functions:

  void addEventRegion(EventRegionPtr eventReg);
  void removeEventRegion(EventRegionPtr eventReg);
  void checkEventRegions( const ExtendedPose2d &newPose, const ExtendedPose2d &previousPose );
  bool isOnParkSignEventRegion();

  ///////////////////////////////////////////////////////////////////
  // Traffic sign functions:

  /*! Adds a new traffic sign to map.
   * If another traffic sign of the same type already exists in the region, they will be merged.
   * \note Merging currently means that the position of the sign which was detected last will be
   * 		used, because we assume that the last position is always the one where the car is
   * 		closest to the sign, meaning the placement of the sign will hopefully be the most
   * 		accurate. */
  void addTrafficSign(TrafficSignPtr sign);
  /*! Overload for convenience. */
  void addTrafficSign(const ExtendedPose2d &pose, int type);
  TrafficSignPtr getNearestTrafficSign(const ExtendedPose2d &pose, int signType );
  TrafficSignPtrList getTrafficSigns();

  /*! returns a Traffic Sign for a Patch if there is a fitting one
   * Otherwise returns a null pointer */
  TrafficSignPtr getTrafficSignAtPatch(PatchPtr other);

  ///////////////////////////////////////////////////////////////////
  // Obstacle functions:
  
  void addObstacle(ObstaclePtr &obstacle);

  /*! Returns an obstacle which is relevant i.e. in the Trajectory*/
  void getObstacleInTrajectory(ObstaclePtrList& inTrajObst, double distOffset = 0.03);

  /* Clears all obstacles detected by the sensors of the given type.
   * \note Also deletes the event regions! */
  void clearObstacles( SensorType type );

  /*! isRelevantObstacle check if Obstacle near the simple trajectory
   * \param obstacle obstacle to check
   * \return true if it is in the way */
  bool isRelevantObstacle(ObstaclePtr obstacle, Trajectory2d &traj, double distOffset = 0.03);

  //! removes all obstacles which time to live has expiered. Removes also the appendant event regions (Not tested with more than 1 event region).
  //! /note this function is called by his self every second. (If timer instance is not NULL;)
  //void removeOldObstacles();
  bool isObstacleFree(EnvObjectPtr other, bool checkNonStaticObstacles = false );
  bool isObstacleFree(EnvObjectPtr other, drivingDirection dd, bool checkNonStaticObstacles = false );

  bool isInFrontOfCar(EnvObjectPtr other, double offset = 0);
  bool isMultiRelevantObstacle(ObstaclePtr obstacle, double distOffset = 0.03);
  ObstaclePtr getNearstRelevantObstacle(ExtendedPose2d pos, ObstaclePtrList relObst,
      Trajectory2d &traj);
  bool calculateProjection( const Trajectory2d &trajectory, const ExtendedPose2d &curPosition,
      ExtendedPose2d &projection, double &distance, size_t &nearest_pose_index);
  void insertObstaclePoints(Trajectory2d &traj);
  void updateTrajectorySpeed(Trajectory2d &traj);
  void interpolateTrajSpeed(Trajectory2d &traj, unsigned int ind1, unsigned int ind2,
      bool allZero);
  bool isEndOfTrajReached() const {
    return mEndOfTrajReached;
  }
  void setEndOfTrajReached(bool endOfTrajReached) {
    Environment::mEndOfTrajReached = endOfTrajReached;
  }
  void resetStreetLane();

  ///////////////////////////////////////////////////////////////////
  // ACC functions:
  
  bool isAccOn() const;
  void setAccOn(bool mAccOn);
  bool isOverwriteAccMinSpeed() const;
  void setOverwriteAccMinSpeed(bool mOverwriteAccMinSpeed);
  double getNewAccMinSpeed() const;
  void setNewAccMinSpeed(double mNewAccMinSpeed);

  ///////////////////////////////////////////////////////////////////
  // Timer functions:
  
  /*! Implement TimerEventListener: */
  void eventTimerFired( timerType type, unsigned long timerID );

  ///////////////////////////////////////////////////////////////////
  // Sensor settings:
  
  void initUSSensorLimits();
  oadrive::obstacle::usSensor* getCurrentUSSensorLimits() {
    return &mUSSensorLimits[mCurrentUSSensorLimits];
  }
  void setCurrentUSSensorLimits( oadrive::obstacle::enumUSSensorLimits state );

  ///////////////////////////////////////////////////////////////////
  // Debug functions:

  /*! Debug function. Uses EnvironmentPainter class to draw a map of this Environment.*/
  cv::Mat getEnvAsImage( double x, double y, double radius, float pixelsPerMeter = 20);

  /*! Add a point to the map output for a certain amount of time
   * \note secondsUntilDeletion can be a floating-point value for sub-second precision.
   * \note The points will be deleted when the secondsUntilDeletion is
   * 		reached, but only if getEnvAsImage is called.
   * \param drawDirection True = draw Direction false = draw Point
   */
  void addDebugPoint( const oadrive::core::ExtendedPose2d &pose,
      cv::Scalar col, double secondsUntilDeletion, bool drawDirection );
  void addDebugPoint( const Position2d &pos, cv::Scalar col,
      double secondsUntilDeletion );
  /*! collectDebugPoints fetch Debug Points from various functions */
  void collectDebugPoints();

  /*! Remember certain obstacles?
   * Setting this to 'true' will let the environment remember
   * certain obstacles between frames.
   * This is used to remember all obstacles which are interesting
   * for parking (i.e. because they block a parking spot). */
  void setRememberObstacles( bool remember );

  /*! Calculates whether the obstacle is towards the car's right side */
  bool couldBeOnParkingLot( ObstaclePtr obstacle );

  ///////////////////////////////////////////////////////////////////
  // Misc functions:

  //! map the given angle into the range [0 .. 2*Pi)
  static double angleMod2Pi(double angle);

  //! map the given angle into the range [0 .. Pi)
  static double angleModPi(double angle);

  //! return value mod modulo
  static double modulo(double value, double modulo);

  std::string toJson(double radius);

  bool existsCarPoseIndexBeforePatch(unsigned int patchID);
  bool isCarHasBackedUp() const {
    return mCarHasBackedUp;
  }
  void setCarHasBackedUp(bool CarHasBackedUp) {
    Environment::mCarHasBackedUp = CarHasBackedUp;
  }
  bool isStorePosesBeforePatch() const {
    return mStorePosesBeforePatch;
  }
  void setStorePosesBeforePatch(bool storePosesBeforePatch) {
    Environment::mStorePosesBeforePatch = storePosesBeforePatch;
  }

  oadrive::util::Timer* getTimer() { return mTimer; }
  oadrive::control::DriverModule* getDriver() { return mDriver; }
  oadrive::util::CoordinateConverter* getCoordConverter() { return mCoordConverter; }

private:
  /*! Private so that it cannot be called */
  Environment( oadrive::util::CoordinateConverter* coordConverter = NULL,
               oadrive::control::DriverModule* driver = NULL,
               oadrive::util::Timer *timer = NULL);
  //Environment(Environment const&);        // copy constructor is private
  //Environment& operator=(Environment const&);  // assignment operator is private

  /*! The one and only instance of this singleton.*/
  static boost::shared_ptr<Environment> mInstance;

  EnvObjectPtr mCar;
  oadrive::util::CoordinateConverter* mCoordConverter;

  //PatchPtrList mUnsortedPatches;
  PatchPtrList mStreet;
  PatchPtrList mParkingLots;
  PatchPtrList mOpenPatches;

  EventRegionPtrList mEventRegions;

  ObstaclePtrList mObstacles;
  bool mRememberObstacles;

  TrafficSignPtrList mTrafficSigns;

  oadrive::control::DriverModule* mDriver;

  EnvObjectPtr findNearestInList( const ExtendedPose2d &pose, EnvObjectPtrList* envObjects);
  //EnvObjectPtr findNearestInList(double x, double y, EnvObjectPtrList* envObjects, int numbr, EnvObjectPtr nearest);

  oadrive::world::PatchStitcher mPatchStitcher;

  unsigned int mMaxNumberOfUnsortedPatches;

  WorldEventListener* mEventListener;
  //void drawEnvObjects(cv::Mat img, double x, double y, double radius,float pixelsPerMeter);

  bool mAccOn;
  bool mOverwriteAccMinSpeed;
  double mNewAccMinSpeed;

  enumLane mLane;

  void updateHistory();
  PatchPtrList mCurrentCarPatches;
  PatchPtrList mPastCarPatches;
  ExtendedPose2dVector mPastCarPoses;
  ExtendedPose2dVectorByUIntMap mCarPosesOnPatch;
  std::map <unsigned int , int  > mCarPoseIndexBeforePatch;

  DebugPointList mDebugPoints;
  boost::posix_time::ptime mDebugPointTime;

  void removePatchFromList(PatchPtr patch, PatchPtrList* list);

  oadrive::core::Trajectory2d mTrajectory;
  MultiTrajectory mMultiTrajectory;
  oadrive::core::Trajectory2d mRawTrajectory;

  //! Stores a Trajectory which has as less as possible points due this the trajectory is not so accurate
  //! \ref setTrajectory
  oadrive::core::Trajectory2d mTrajectorySimple;

  int getNextTrajIndex();
  int getNextTrajIndex(const ExtendedPose2d other);


  //! \brief mTimer This timer is nessesary to remove objects which have a expiered time to live
  //! \note This timer has to be updated by the interface
  oadrive::util::Timer* mTimer;

  /*! Determines which traffic signs belong to which patches.
   * Should be called whenever a new patch of traffic sign has been found.
   * TODO: Currently thcs iterates over all patches and all traffic signs. This could be made
   * faster by only checking new traffic signs and new patches.*/
  void connectPatchesAndTrafficSigns();
  /*! Attempts to connect a traffic sign to a patch.
   * First removes any other connections.
   * Also checks if the new patch is already attached to the sign and if so, aborts.*/
  bool connect( TrafficSignPtr sign, PatchPtr patch );

  //! Adds an event region to a traffic sign if it's a parking sign
  void addEventRegionToTrafficSign(TrafficSignPtr sign);

  int getNextTrajIndexSimple();

  //! counts the number of saved trajectories
  int mTrajDebugCounter;

  EnvironmentPainter mEnvironmentPainter;

  bool mEndOfTrajReached;

  bool mCarHasBackedUp;
  bool mStorePosesBeforePatch;

  oadrive::obstacle::enumUSSensorLimits mCurrentUSSensorLimits;
  std::map< oadrive::obstacle::enumUSSensorLimits,
    oadrive::obstacle::usSensor> mUSSensorLimits;


public:
  // use a proper alignment when calling the constructor.
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    bool isCarAtTrajectoryEnd(const Trajectory2d &traj);

    bool isAtMultiTrajEnd(const MultiTrajectory &traj);

    bool isInFrontOf(ExtendedPose2d firstPose, ExtendedPose2d other, double offset);
};

}	// namespace
} // namespace

#endif
