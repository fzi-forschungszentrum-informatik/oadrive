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
 * \author  Micha Pfeiffer <ueczz@student.kit.edu>
 * \date    2015-12-24
 * 
 * \author  Simon Roesler <simon.roesler@student.kit.edu>
 * \date    2018
 * 
 * \author  Mark Hueneberg <hueneber@fzi.de>
 * \date    2018
 *
 */
//----------------------------------------------------------------------

#ifndef OADRIVE_WORLD_ROAD_H
#define OADRIVE_WORLD_ROAD_H

#include "oadrive_world/Patch.h"

namespace oadrive
{
namespace world
{
class Patch;
typedef boost::shared_ptr<Patch> PatchPtr;		// define before class because it uses the PatchPtr
typedef std::list<boost::shared_ptr<Patch> > PatchPtrList;

class Road
{

public:

  Road();

  virtual ~Road() {};

  void addRoadPatch(PatchPtr streetPatch);

  std::vector<int> updateHistory(const core::ExtendedPose2d &currentCarPose);

  const PatchPtrList* getRoad() const;

  const PatchPtrList* getCarPatchHistory() const;

  void deletePatchesUntilLastIntersection();

  const boost::shared_ptr<Patch> getNextIntersection();

  void addTrafficSign(const boost::shared_ptr<TrafficSign> trafficSign);

  void reset();

  void deleteOldPatches();

  void setLane(LaneType lane, int numChangedOldPatches = 1);

/******************************************************************************/
/**************************** class attributes ********************************/
/******************************************************************************/
private:
  /*! Contains all patches found so far */
  PatchPtrList mPatches;

  /*! Contains all patches which have open sides which means patches with still missing
   * predecessor or successor */
  PatchPtrList mOpenPatches;

  /*! Contains all patches the car is currently on */
  PatchPtrList mCurrentCarPatches;

  /*! Contains all patches the car is or was on */
  PatchPtrList mCarPatchHistory;

  float mMaximumAngle;
  float mMinimumDistBetweenCrossings;

  LaneType mCurrentLane = LANE_RIGHT;

  //EnvironmentPtr mEnv;

public:
  // use a proper alignment when calling the constructor.
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace
}  // namespace

#endif  // OADRIVE_WORLD_ROAD_H
