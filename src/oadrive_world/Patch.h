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
 * \author  Micha Pfeiffer <ueczz@student.kit.edu>
 * \date    2015-12-23
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

namespace oadrive{
namespace world{

enum drivingDirection {DD_STRAIGHT, DD_LEFT, DD_RIGHT, DD_NONE};
enum entryDirection {ED_WEST, ED_NORTH, ED_EAST, ED_SOUTH};

struct entryPoint {
  entryDirection dir;
  oadrive::core::ExtendedPose2d pose;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

typedef std::list<entryPoint, Eigen::aligned_allocator<entryPoint> > EntryPointList;

enum PatchType { STRAIGHT=0, CROSS_SECTION=1, PARKING=2 };
enum enumLane { LANE_RIGHT=0, LANE_LEFT=1 };
const int numPatchTypes = 3;

enum ParkingType { PARKING_TYPE_UNKNOWN=0, PARKING_TYPE_PARALLEL=1, PARKING_TYPE_CROSS=2 };

const float STREET_FIELD_WIDTH = 1.0;
//const float CROSSING_PATCH_LENGTH = 0.5;
const float STREET_SIDE_TO_CENTER = 0.440 + 0.015 + 0.010;
const double STREET_SIDE_TO_MID_LANE = 0.22 + 0.010;
const float CROSSROAD_LINE_TO_CROSS_SECTION = 0.5 + 0.1;
const float LANE_WIDTH = 0.440;
const float PATCH_WIDTHS[numPatchTypes] = { 1.0, 1.0, 0.47 };
const double PATCH_LENGTHS[numPatchTypes] = { 0.25, 1.0, 0.7853 };

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
  Patch( PatchType mType, const oadrive::core::ExtendedPose2d &mPose );
  Patch();
  ~Patch();

  oadrive::core::Trajectory2d* getTrajectory( oadrive::core::ExtendedPose2d& startPose );

  //! New version of getTrajectory(ExtendedPose2d& startPose), which also can turn left and right. (get the info from the mission control)
  oadrive::core::Trajectory2d* getTrajectoryFromMC( oadrive::core::ExtendedPose2d& startPose);
  oadrive::core::Trajectory2d* getTrajectoryFromMC( oadrive::core::ExtendedPose2d& startPose , enumLane lane);

  oadrive::core::Trajectory2d* getTrajectory( oadrive::core::ExtendedPose2d& startPose, drivingDirection dd  , enumLane lane = LANE_RIGHT );

  //bool isInFrontOf(PatchPtr other);

  /*! Overwrite parent to be able to update Corners and Trajectors: */
  void setPose(const oadrive::core::ExtendedPose2d &mPose);

  PatchType getPatchType() const { return mPatchType; }

  /*! Returns pointer to a child.
                 * \note Returns invalid pointer if the child doesn't exist! */
  PatchPtr getChild( unsigned int index );
  PatchPtrList getChildren();

  PatchPtr getChild( oadrive::core::ExtendedPose2d &mPose, drivingDirection dir );
  PatchPtr getChild( float goalAngle, drivingDirection dir );


  /*! Get child according to current action set by mission control */
  PatchPtr getNextChild( oadrive::core::ExtendedPose2d &mPose );
  PatchPtr getNextChild( float goalAngle );

  /*! Adds newChild to the list of my children
                 * First, this function checks if newChild is already my child (if so, it aborts).
                 * \note The current patch also becomes the child of the newChild.
                 * \return true if the newChild was added to my list of children. */
  bool addChild( PatchPtr newChild );
  void addChildren(PatchPtrList children);

  /*! Returns true if other is one of my children, false otherwise. */
  bool isChild( PatchPtr other );

  float couldBeMyChild( PatchPtr other );
  bool hasOpenSides();

  bool removeChild(PatchPtr other );

  //! Set the action (trajectory) which should be taken an this patch
  bool setAction(drivingDirection dir);
  //! return which action for this patch is currently configured
  drivingDirection getAction();

  float getAngleTo( PatchPtr other );

  oadrive::core::ExtendedPose2d mOriginalPose;

  bool addTrafficSign( TrafficSignPtr newSign );
  bool removeTrafficSign( TrafficSignPtr sign );

  void setMonitored(bool monitored);
  bool isMonitored();

  EventRegionPtr getObstacleRegion( const oadrive::core::ExtendedPose2d &pose , drivingDirection dir);

  //gives the traffic sign , on the oposite side of the patch (from the car)
  TrafficSignPtr getCorrespondingTrafficSign(const oadrive::core::ExtendedPose2d &pose );

  /*! Merge otherPatch into this patch.
   * \note The patch type of both patches should be the same.
   * \note This might do different things depending on patch type.
   * \note This leaves the patch untouched if setFixed() was already called on the patch.
   * \return true if merged, false otherwise
   * \sa getPatchType() */
  bool tryMerge( PatchPtr otherPatch );

  bool isSwitchPatch();
  int getSwitchType();
  void setSwitchType(int swType);

  enumLane getLane();
  void setLane(enumLane lane);

  void setFixed() { mIsFixed = true; }
  bool isFixed() { return mIsFixed; }
  std::string toJson();

private:

  int angle2AngleRegionID( float angle );
  float angleRegion2Angle( int regionID );



  /*! Calculates the trajectory points for the current position/orientation and type.
                 * Generates the predefined trajectory which lies entirely on this patch.
                 * \note MUST be called whenever the position or orientation of the patch changes! */
  void updateTrajectory();

  PatchType mPatchType;
  oadrive::core::Trajectory2dSequence mTrajectories;

  drivingDirection mAction;

  PatchPtrList mChildren;

  //! All possible entrypoints for this Path. Is filled by the updateTrajectory() method.
  EntryPointList entryPoints;

  //! Maps all possible entry and outgoing possiblies to the rigth Trajectory. Is filled by the updateTrajectory() method.
  //! \todo FIX THESE according to Eigen Guidelines!
  std::map<entryDirection, std::map<drivingDirection, oadrive::core::Trajectory2d> > trajMap;

  short mNumAngleRegions;

  TrafficSignPtrList mTrafficSigns;
  std::map<entryDirection, TrafficSignPtr > mOrderedTrafficSigns;

  bool mMonitored;
  bool hasTrajDB;

  //! Parking lot "position" and orientation votes. The median (by angle) is always used.
  oadrive::core::ExtendedPose2dVector mPoseVotes;

  int mSwitchPatch;
  enumLane mLane;

  bool mIsFixed;
  double mMagicStreetOffset;

public:
  // use a proper alignment when calling the constructor.
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

}	// namespace
}	// namespace

#endif
