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
 * \date    2015-12-23
 * 
 * \author  Simon Roesler <simon.roesler@student.kit.edu>
 * \date    2018
 * 
 * \author  Mark Hueneberg <hueneber@fzi.de>
 * \date    2018
 *
 */
//----------------------------------------------------------------------

#ifndef OADRIVE_WORLD_PATCH_H
#define OADRIVE_WORLD_PATCH_H

#include "EnvObject.h"
#include "EventRegion.h"
#include <oadrive_core/Trajectory2d.h>
#include <map>
#include <list>
#include <boost/shared_ptr.hpp>
#include "TrafficSign.h"


//#define EXP_MOVING_AVERAGE_PATCH_POSE 0.3

namespace oadrive
{
namespace world
{

enum drivingDirection {DD_STRAIGHT, DD_LEFT, DD_RIGHT, DD_NONE};
enum entryDirection {ED_WEST, ED_NORTH, ED_EAST, ED_SOUTH};

struct entryPoint {
  entryDirection dir;
  oadrive::core::ExtendedPose2d pose;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

typedef std::list<entryPoint, Eigen::aligned_allocator<entryPoint> > EntryPointList;

enum PatchType { STRAIGHT=0, CROSS_SECTION=1, PARKING=2 };
enum LaneType { LANE_RIGHT=0, LANE_LEFT=1, RESCUE_LANE=2, SWITCH_TARGET_LANE=3, SWITCH_BACK_TARGET_LANE=4, LANE_CENTER=5};
const int numPatchTypes = 3;

enum ParkingType { PARKING_TYPE_UNKNOWN=0, PARKING_TYPE_PARALLEL=1, PARKING_TYPE_CROSS=2 };

const float STREET_FIELD_WIDTH = 1.0;
//const float CROSSING_PATCH_LENGTH = 0.5;
const float STREET_SIDE_TO_CENTER = 0.5; //0.440 + 0.015 + 0.010;
const double STREET_SIDE_TO_MID_LANE = 0.22;
const float CROSSROAD_LINE_TO_CROSS_SECTION = 0.5 + 0.1;
const float LANE_WIDTH = 0.440;
const float PATCH_WIDTHS[numPatchTypes] = { 0.96, 0.96, 0.47 };
const double PATCH_LENGTHS[numPatchTypes] = { 0.25, 0.96, 0.7853 };

// PARKING_PATCH:
// Parallel: 0.45 x 0.7853
// Cross: 0.47 x 0.85
// For the StreetPatcher, define the precise positions:
const float PATCH_LENGTHS_PARKING_PARALLEL = 0.7853;
const float PATCH_WIDTHS_PARKING_PARALLEL = 0.45;
const float PATCH_LENGTHS_PARKING_CROSS = 0.85;
const float PATCH_WIDTHS_PARKING_CROSS = 0.47;

const float MINIMUM_PATCH_DIST = 0.25;
const float MAXIMUM_PATCH_DIST = 2;

class Patch;	// predefine so shared_ptr can use it
typedef boost::shared_ptr<Patch> PatchPtr;		// define before class because it uses the PatchPtr
typedef std::list<boost::shared_ptr<Patch> > PatchPtrList;

class TrafficSign;	// predefine so shared_ptr can use it
typedef boost::shared_ptr<TrafficSign> TrafficSignPtr;		// define before class because it uses the PatchPtr
typedef std::list<boost::shared_ptr<TrafficSign> > TrafficSignPtrList;

class Patch : public EnvObject
{
public:
  Patch(PatchType mType, core::ExtendedPose2d mPose);
  Patch();
  ~Patch();

  /**
   * Merge pose, type and direction from newPatch into this patch
   */
  void mergeFrom(const PatchPtr newPatch);


  //! New version of getTrajectory(ExtendedPose2d& startPose), which also can turn left and right. (get the info from the mission control)
  oadrive::core::Trajectory2d* getTrajectoryFromMC( oadrive::core::ExtendedPose2d& startPose , LaneType lane);

  oadrive::core::Trajectory2d* getTrajectory( oadrive::core::ExtendedPose2d& startPose, drivingDirection dd  , LaneType lane = LANE_RIGHT );

  /*! Overwrite parent to be able to update Corners and Trajectors: */
  void setPose(const oadrive::core::ExtendedPose2d &mPose);

  PatchType getPatchType() const { return mPatchType; }

  PatchPtr getSuccessor();

  /*! Adds newChild to the list of my children
                 * First, this function checks if newChild is already my child (if so, it aborts).
                 * \note The current patch also becomes the child of the newChild.
                 * \return true if the newChild was added to my list of children. */
  void setSuccessor(PatchPtr successor);

  void removeSuccessor();

  /*! Returns true if other is one of my children, false otherwise. */

  //! Set the action (trajectory) which should be taken an this patch
  bool setAction(drivingDirection dir);
  //! return which action for this patch is currently configured
  drivingDirection getAction();

  oadrive::core::ExtendedPose2d mOriginalPose;

  bool isSwitch();
  void setSwitch(bool is_switch);

  LaneType getLane();
  void setLane(LaneType lane);
  // sets the recommended lane determined by the patch obstacle detection
  void setRecommendedLane(LaneType lane);
  LaneType getRecommendedLane();

  void setFixed() { mIsFixed = true; }
  bool isFixed() { return mIsFixed; }
  std::string toJson();

  void setPatchID(int patchID);
  int getPatchID();

  void connectTrafficSign(const TrafficSignPtr trafficSign);
  const TrafficSignPtr getTrafficSign();
  bool hasTrafficSign();

private:

  /*! Calculates the trajectory points for the current position/orientation and type.
                 * Generates the predefined trajectory which lies entirely on this patch.
                 * \note MUST be called whenever the position or orientation of the patch changes! */


  core::Trajectory2d rotateAndMoveTrajectory(core::Trajectory2d &traj, core::ExtendedPose2d pose);

  PatchType mPatchType;


  drivingDirection mAction;

  PatchPtr mSuccessor;

  //! All possible entrypoints for this Path. Is filled by the updateTrajectory() method.


  //! Maps all possible entry and outgoing possiblies to the rigth Trajectory. Is filled by the updateTrajectory() method.
  //! \todo FIX THESE according to Eigen Guidelines!
  core::Trajectory2d mCurrentTrajectory;

  short mNumAngleRegions;

  boost::shared_ptr<TrafficSign> mTrafficSign;
  bool mHasTrafficSign;
  std::map<entryDirection, TrafficSignPtr > mOrderedTrafficSigns;

  bool mMonitored;

  //! Parking lot "position" and orientation votes. The median (by angle) is always used.
  oadrive::core::ExtendedPose2dVector mPoseVotes;

  //! Whether this patch will initiate a lane switch.
  bool mSwitch;
  LaneType mLane;
  LaneType mRecommendedLane;

  bool mIsFixed;
  double mMagicStreetOffset;

  // ID of patch. Consecutive patches have consecutive IDs.
  int mPatchID;

  // Store the accumulator for the exp moving average
  #ifdef EXP_MOVING_AVERAGE_PATCH_POSE
  struct {
    double x;
    double y;
    double yaw;
  } mExpMovingAvgPose;
  #endif

public:
  // use a proper alignment when calling the constructor.
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

}	// namespace
}	// namespace

#endif
